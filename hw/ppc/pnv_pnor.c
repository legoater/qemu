/*
 * QEMU PowerNV PNOR related functions
 *
 * Copyright (c) 2015-2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "sysemu/block-backend.h"
#include "sysemu/blockdev.h"
#include "hw/loader.h"
#include "hw/ppc/ffs.h"
#include "hw/ppc/pnv_pnor.h"
#include "libxz/xz.h"

static uint32_t ffs_checksum(void *data, size_t size)
{
    uint32_t i, csum = 0;

    for (i = csum = 0; i < (size/4); i++) {
        csum ^= ((uint32_t *)data)[i];
    }
    return csum;
}

static int ffs_check_convert_header(struct ffs_hdr *dst, struct ffs_hdr *src)
{
    dst->magic = be32_to_cpu(src->magic);
    if (dst->magic != FFS_MAGIC) {
        return -1;
    }
    dst->version = be32_to_cpu(src->version);
    if (dst->version != FFS_VERSION_1) {
        return -1;
    }
    if (ffs_checksum(src, FFS_HDR_SIZE) != 0) {
        return -1;
    }
    dst->size = be32_to_cpu(src->size);
    dst->entry_size = be32_to_cpu(src->entry_size);
    dst->entry_count = be32_to_cpu(src->entry_count);
    dst->block_size = be32_to_cpu(src->block_size);
    dst->block_count = be32_to_cpu(src->block_count);

    return 0;
}

static int ffs_check_convert_entry(struct ffs_entry *dst, struct ffs_entry *src)
{
    if (ffs_checksum(src, FFS_ENTRY_SIZE) != 0) {
        return -1;
    }

    memcpy(dst->name, src->name, sizeof(dst->name));
    dst->base = be32_to_cpu(src->base);
    dst->size = be32_to_cpu(src->size);
    dst->pid = be32_to_cpu(src->pid);
    dst->id = be32_to_cpu(src->id);
    dst->type = be32_to_cpu(src->type);
    dst->flags = be32_to_cpu(src->flags);
    dst->actual = be32_to_cpu(src->actual);
    dst->user.datainteg = be16_to_cpu(src->user.datainteg);

    return 0;
}

static int decompress(void *dst, size_t dst_size, void *src, size_t src_size)
{
    struct xz_dec *s;
    struct xz_buf b;
    int ret = 0;

    /* Initialize the xz library first */
    xz_crc32_init();
    s = xz_dec_init(XZ_SINGLE, 0);
    if (!s) {
        error_report("pnv_pnor: failed to initialize xz");
        return -1;
    }

    /*
     * Source address : src
     * Source size : src_size
     * Destination address : dst
     * Destination size : dst_src
     */
    b.in = src;
    b.in_pos = 0;
    b.in_size = src_size;
    b.out = dst;
    b.out_pos = 0;
    b.out_size = dst_size;

    /* Start decompressing */
    ret = xz_dec_run(s, &b);
    if (ret != XZ_STREAM_END) {
        error_report("pnv_pnor: failed to decompress partition : %d", ret);
        ret = -1;
    } else {
        ret = 0;
    }

    /* Clean up memory */
    xz_dec_end(s);
    return ret;
}

int pnv_pnor_load_skiboot(DriveInfo *dinfo, hwaddr addr, size_t max_size,
                          Error **errp)
{
    BlockBackend *blk = blk_by_legacy_dinfo(dinfo);
    uint8_t *storage;
    int64_t size;
    struct ffs_hdr hdr;
    struct ffs_entry ent;
    uint32_t i;
    int rc;

    size = blk_getlength(blk);
    if (size <= 0) {
        error_setg(errp, "pnv_pnor: failed to get flash size");
        return -1;
    }

    storage = g_new0(uint8_t, size);
    rc = blk_pread(blk, 0, storage, size);
    if (rc < 0) {
        error_setg(errp, "pnv_pnor: failed to read the initial flash content");
        goto out;
    }

    rc = ffs_check_convert_header(&hdr, (struct ffs_hdr *) storage);
    if (rc) {
        error_setg(errp, "pnv_pnor: bad header\n");
        goto out;
    }

    for (i = 0; i < hdr.entry_count; i++) {
        uint32_t esize = hdr.entry_size;
        uint32_t offset = FFS_HDR_SIZE + i * esize;
        struct ffs_entry *src_ent = (struct ffs_entry *)(storage + offset);

        rc = ffs_check_convert_entry(&ent, src_ent);
        if (rc) {
            error_report("pnv_pnor: bad partition entry %d", i);
            continue;
        }

        if (!strcmp("PAYLOAD", ent.name)) {
            void *buffer = g_malloc0(max_size);

            rc = decompress(buffer, max_size, &storage[ent.base * 0x1000],
                            ent.size * 0x1000);
            if (rc == 0) {
                rom_add_blob_fixed("pnor.skiboot", buffer, max_size, addr);
            }
            g_free(buffer);
            goto out;
        }
    }

    error_setg(errp, "pnv_pnor: no skiboot partition !?");

out:
    g_free(storage);
    return rc;
}
