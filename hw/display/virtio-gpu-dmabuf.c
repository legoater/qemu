/*
 * Virtio GPU Device
 *
 * Copyright Red Hat, Inc. 2013-2014
 *
 * Authors:
 *     Dave Airlie <airlied@redhat.com>
 *     Gerd Hoffmann <kraxel@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/iov.h"
#include "ui/console.h"
#include "hw/virtio/virtio-gpu.h"
#include "hw/virtio/virtio-gpu-pixman.h"
#include "hw/vfio/vfio-device.h"
#include "trace.h"
#include "system/ramblock.h"
#include "system/hostmem.h"
#include <sys/ioctl.h>
#include <linux/memfd.h>
#include "qemu/memfd.h"
#include "standard-headers/linux/udmabuf.h"
#include "standard-headers/drm/drm_fourcc.h"

static int virtio_gpu_create_udmabuf(struct virtio_gpu_simple_resource *res,
                                     Error **errp)
{
    g_autofree struct udmabuf_create_list *list = NULL;
    RAMBlock *rb;
    ram_addr_t offset;
    int udmabuf, i, fd;

    udmabuf = udmabuf_fd();
    if (udmabuf < 0) {
        error_setg(errp, "udmabuf device not available or enabled");
        return VFIO_DMABUF_CREATE_ERR_UNSPEC;
    }

    list = g_try_malloc0(sizeof(struct udmabuf_create_list) +
                         sizeof(struct udmabuf_create_item) * res->iov_cnt);
    if (!list) {
        error_setg(errp, "failed to allocate udmabuf create list");
        return VFIO_DMABUF_CREATE_ERR_UNSPEC;
    }

    for (i = 0; i < res->iov_cnt; i++) {
        rb = qemu_ram_block_from_host(res->iov[i].iov_base, false, &offset);
        if (!rb || rb->fd < 0) {
            error_setg(errp, "IOV memory address incompatible with udmabuf ");
            return VFIO_DMABUF_CREATE_ERR_INVALID_IOV;
        }

        list->list[i].memfd  = rb->fd;
        list->list[i].offset = offset;
        list->list[i].size   = res->iov[i].iov_len;
    }

    list->count = res->iov_cnt;
    list->flags = UDMABUF_FLAGS_CLOEXEC;

    fd = ioctl(udmabuf, UDMABUF_CREATE_LIST, list);
    if (fd < 0) {
        error_setg_errno(errp, errno, "UDMABUF_CREATE_LIST: ioctl failed");
        if (errno == EINVAL || errno == EBADFD) {
            return VFIO_DMABUF_CREATE_ERR_INVALID_IOV;
        }
        return VFIO_DMABUF_CREATE_ERR_UNSPEC;
    }
    return fd;
}

static void *virtio_gpu_remap_dmabuf(struct virtio_gpu_simple_resource *res,
                                     Error **errp)
{
    void *map;

    map = mmap(NULL, res->blob_size, PROT_READ, MAP_SHARED, res->dmabuf_fd, 0);
    if (map == MAP_FAILED) {
        error_setg_errno(errp, errno, "dmabuf mmap failed");
        return NULL;
    }
    return map;
}

void virtio_gpu_fini_dmabuf(struct virtio_gpu_simple_resource *res)
{
    if (res->remapped) {
        munmap(res->remapped, res->blob_size);
        res->remapped = NULL;
    }
    if (res->dmabuf_fd >= 0) {
        close(res->dmabuf_fd);
        res->dmabuf_fd = -1;
        res->share_handle = SHAREABLE_NONE;
    }
    res->blob = NULL;
}

static int find_memory_backend_type(Object *obj, void *opaque)
{
    bool *memfd_backend = opaque;
    int ret;

    if (object_dynamic_cast(obj, TYPE_MEMORY_BACKEND)) {
        HostMemoryBackend *backend = MEMORY_BACKEND(obj);
        RAMBlock *rb = backend->mr.ram_block;

        if (rb && rb->fd > 0) {
            ret = fcntl(rb->fd, F_GET_SEALS);
            if (ret > 0) {
                *memfd_backend = true;
            }
        }
    }

    return 0;
}

bool virtio_gpu_have_udmabuf(void)
{
    Object *memdev_root;
    int udmabuf;
    bool memfd_backend = false;

    udmabuf = udmabuf_fd();
    if (udmabuf < 0) {
        return false;
    }

    memdev_root = object_resolve_path("/objects", NULL);
    object_child_foreach(memdev_root, find_memory_backend_type, &memfd_backend);

    return memfd_backend;
}

bool virtio_gpu_init_dmabuf(struct virtio_gpu_simple_resource *res)
{
    Error *local_err = NULL;
    void *pdata = NULL;

    res->dmabuf_fd = -1;
    if (res->iov_cnt == 1 &&
        res->iov[0].iov_len < 4096) {
        pdata = res->iov[0].iov_base;
    } else if (res->blob_size) {
        res->dmabuf_fd = virtio_gpu_create_udmabuf(res, &local_err);
        if (res->dmabuf_fd == VFIO_DMABUF_CREATE_ERR_INVALID_IOV) {
            error_free_or_abort(&local_err);

            res->dmabuf_fd = vfio_device_create_dmabuf_fd(res->iov,
                                                          res->iov_cnt,
                                                          &local_err);
            if (res->dmabuf_fd == VFIO_DMABUF_CREATE_ERR_INVALID_IOV) {
                error_free_or_abort(&local_err);
                qemu_log_mask(LOG_GUEST_ERROR,
                              "Cannot create dmabuf: incompatible memory\n");
                return false;
            }

            if (res->dmabuf_fd >= 0) {
                pdata = vfio_device_mmap_dmabuf(res->iov, res->iov_cnt,
                                                &local_err);
                if (!pdata) {
                    virtio_gpu_fini_dmabuf(res);
                }
            } else {
                res->dmabuf_fd = -1;
            }
        } else if (res->dmabuf_fd >= 0) {
            pdata = virtio_gpu_remap_dmabuf(res, &local_err);
            if (!pdata) {
                virtio_gpu_fini_dmabuf(res);
            }
        } else {
            res->dmabuf_fd = -1;
        }

        if (res->dmabuf_fd < 0) {
            error_report_err(local_err);
            return false;
        }
        res->share_handle = res->dmabuf_fd;
        res->remapped = pdata;
    }

    res->blob = pdata;

    return true;
}

static QemuDmaBuf *
virtio_gpu_create_dmabuf(struct virtio_gpu_simple_resource *res,
                         struct virtio_gpu_framebuffer *fb,
                         struct virtio_gpu_rect *r)
{
    uint32_t offset = 0;
    int fd;

    if (res->dmabuf_fd < 0) {
        return NULL;
    }

    fd = qemu_dup(res->dmabuf_fd);
    if (fd < 0) {
        return NULL;
    }

    return qemu_dmabuf_new(r->width, r->height,
                           &offset, &fb->stride,
                           r->x, r->y, fb->width, fb->height,
                           qemu_pixman_to_drm_format(fb->format),
                           DRM_FORMAT_MOD_INVALID, &fd,
                           1, true, false);
}

int virtio_gpu_update_dmabuf(VirtIOGPU *g,
                             uint32_t scanout_id,
                             struct virtio_gpu_simple_resource *res,
                             struct virtio_gpu_framebuffer *fb,
                             struct virtio_gpu_rect *r)
{
    struct virtio_gpu_scanout *scanout = &g->parent_obj.scanout[scanout_id];
    QemuDmaBuf *new_primary, *old_primary;
    uint32_t width, height;

    new_primary = virtio_gpu_create_dmabuf(res, fb, r);
    if (!new_primary) {
        return -EINVAL;
    }

    old_primary = scanout->dmabuf;

    width = qemu_dmabuf_get_width(new_primary);
    height = qemu_dmabuf_get_height(new_primary);
    scanout->dmabuf = new_primary;
    qemu_console_resize(scanout->con, width, height);
    qemu_console_gl_scanout_dmabuf(scanout->con, new_primary);

    if (old_primary) {
        qemu_console_gl_release_dmabuf(scanout->con, old_primary);
        qemu_dmabuf_free(old_primary);
    }

    return 0;
}
