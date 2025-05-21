/*
 * Copyright (c) 2021-2025 Oracle and/or its affiliates.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include <sys/ioctl.h>
#include <linux/vfio.h>
#include "qemu/osdep.h"
#include "hw/vfio/vfio-container.h"
#include "hw/vfio/vfio-cpr.h"
#include "hw/vfio/vfio-device.h"
#include "hw/vfio/vfio-listener.h"
#include "migration/blocker.h"
#include "migration/cpr.h"
#include "migration/migration.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"

/**
 * @brief Unmaps all DMA mappings with virtual addresses in the VFIO container.
 *
 * Issues a VFIO_IOMMU_UNMAP_DMA ioctl with flags to unmap all DMA regions associated with virtual addresses in the specified container. On success, marks the container's vaddr_unmapped flag as true.
 *
 * @param container The VFIO container whose DMA mappings will be unmapped.
 * @param errp Pointer to an Error pointer for reporting errors.
 * @return true if all DMA mappings were successfully unmapped; false otherwise.
 */
static bool vfio_dma_unmap_vaddr_all(VFIOContainer *container, Error **errp)
{
    struct vfio_iommu_type1_dma_unmap unmap = {
        .argsz = sizeof(unmap),
        .flags = VFIO_DMA_UNMAP_FLAG_VADDR | VFIO_DMA_UNMAP_FLAG_ALL,
        .iova = 0,
        .size = 0,
    };
    if (ioctl(container->fd, VFIO_IOMMU_UNMAP_DMA, &unmap)) {
        error_setg_errno(errp, errno, "vfio_dma_unmap_vaddr_all");
        return false;
    }
    container->cpr.vaddr_unmapped = true;
    return true;
}

/**
 * @brief Maps a DMA region with a specified virtual address during CPR load.
 *
 * Updates the virtual address for a DMA mapping in the VFIO container when restoring state after checkpoint/restore (CPR). Requires that the container's CPR state is marked as reused.
 *
 * @param bcontainer Pointer to the base VFIO container.
 * @param iova The IO virtual address to map.
 * @param size The size of the region to map.
 * @param vaddr The virtual address to associate with the mapping.
 * @param readonly Indicates if the mapping should be read-only.
 * @return 0 on success, or a negative errno value on failure.
 */
static int vfio_legacy_cpr_dma_map(const VFIOContainerBase *bcontainer,
                                   hwaddr iova, ram_addr_t size, void *vaddr,
                                   bool readonly)
{
    const VFIOContainer *container = container_of(bcontainer, VFIOContainer,
                                                  bcontainer);
    struct vfio_iommu_type1_dma_map map = {
        .argsz = sizeof(map),
        .flags = VFIO_DMA_MAP_FLAG_VADDR,
        .vaddr = (__u64)(uintptr_t)vaddr,
        .iova = iova,
        .size = size,
    };

    assert(container->cpr.reused);

    if (ioctl(container->fd, VFIO_IOMMU_MAP_DMA, &map)) {
        return -errno;
    }

    return 0;
}

/**
 * @brief Remaps a memory region section for a VFIO container during CPR.
 *
 * Invoked as a memory listener callback to re-add a memory region section to the VFIO container's base container with remapping enabled, typically during checkpoint/restore operations.
 *
 * @param listener The memory listener triggering the callback.
 * @param section The memory region section to be remapped.
 */
static void vfio_region_remap(MemoryListener *listener,
                              MemoryRegionSection *section)
{
    VFIOContainer *container = container_of(listener, VFIOContainer,
                                            cpr.remap_listener);
    vfio_container_region_add(&container->bcontainer, section, true);
}

/**
 * @brief Checks if the VFIO container supports required CPR extensions.
 *
 * Verifies that the container supports both the VFIO_UPDATE_VADDR and VFIO_UNMAP_ALL extensions needed for legacy checkpoint/restore functionality.
 *
 * @param container The VFIO container to check.
 * @param errp Pointer to an Error object for reporting unsupported features.
 * @return true if both extensions are supported, false otherwise.
 */
static bool vfio_cpr_supported(VFIOContainer *container, Error **errp)
{
    if (!ioctl(container->fd, VFIO_CHECK_EXTENSION, VFIO_UPDATE_VADDR)) {
        error_setg(errp, "VFIO container does not support VFIO_UPDATE_VADDR");
        return false;

    } else if (!ioctl(container->fd, VFIO_CHECK_EXTENSION, VFIO_UNMAP_ALL)) {
        error_setg(errp, "VFIO container does not support VFIO_UNMAP_ALL");
        return false;

    } else {
        return true;
    }
}

/**
 * @brief Prepares the VFIO container for VM state saving by unmapping all DMA virtual addresses.
 *
 * Unmaps all DMA mappings with virtual addresses in the container before VM state is saved.
 * Reports errors and returns -1 on failure, 0 on success.
 *
 * @return 0 on success, -1 on failure.
 */
static int vfio_container_pre_save(void *opaque)
{
    VFIOContainer *container = opaque;
    Error *err = NULL;

    if (!vfio_dma_unmap_vaddr_all(container, &err)) {
        error_report_err(err);
        return -1;
    }
    return 0;
}

/**
 * @brief Post-load callback to restore VFIO container state after VM migration.
 *
 * Re-registers the VFIO memory listener, resets the CPR reused flags for the container and all devices,
 * and restores the original DMA map function for the IOMMU class after VM state has been loaded.
 *
 * @param opaque Pointer to the VFIOContainer.
 * @param version_id VM state version identifier.
 * @return 0 on success, -1 on failure.
 */
static int vfio_container_post_load(void *opaque, int version_id)
{
    VFIOContainer *container = opaque;
    VFIOContainerBase *bcontainer = &container->bcontainer;
    VFIOGroup *group;
    VFIODevice *vbasedev;
    Error *err = NULL;

    if (!vfio_listener_register(bcontainer, &err)) {
        error_report_err(err);
        return -1;
    }

    container->cpr.reused = false;

    QLIST_FOREACH(group, &container->group_list, container_next) {
        VFIOIOMMUClass *vioc = VFIO_IOMMU_GET_CLASS(bcontainer);

        /* Restore original dma_map function */
        vioc->dma_map = vfio_legacy_dma_map;

        QLIST_FOREACH(vbasedev, &group->device_list, next) {
            vbasedev->cpr.reused = false;
        }
    }
    return 0;
}

static const VMStateDescription vfio_container_vmstate = {
    .name = "vfio-container",
    .version_id = 0,
    .minimum_version_id = 0,
    .priority = MIG_PRI_LOW,  /* Must happen after devices and groups */
    .pre_save = vfio_container_pre_save,
    .post_load = vfio_container_post_load,
    .needed = cpr_needed_for_reuse,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

/**
 * @brief Handles recovery of DMA virtual address mappings after migration precopy failure.
 *
 * If a migration precopy failure occurs and DMA virtual addresses were previously unmapped,
 * temporarily registers a memory listener and redirects DMA mapping to restore lost virtual
 * address mappings for the VFIO container. After remapping, restores the original DMA map function.
 *
 * @return 0 Always returns 0.
 */
static int vfio_cpr_fail_notifier(NotifierWithReturn *notifier,
                                  MigrationEvent *e, Error **errp)
{
    VFIOContainer *container =
        container_of(notifier, VFIOContainer, cpr.transfer_notifier);
    VFIOContainerBase *bcontainer = &container->bcontainer;

    if (e->type != MIG_EVENT_PRECOPY_FAILED) {
        return 0;
    }

    if (container->cpr.vaddr_unmapped) {
        /*
         * Force a call to vfio_region_remap for each mapped section by
         * temporarily registering a listener, and temporarily diverting
         * dma_map to vfio_legacy_cpr_dma_map.  The latter restores vaddr.
         */

        VFIOIOMMUClass *vioc = VFIO_IOMMU_GET_CLASS(bcontainer);
        vioc->dma_map = vfio_legacy_cpr_dma_map;

        container->cpr.remap_listener = (MemoryListener) {
            .name = "vfio cpr recover",
            .region_add = vfio_region_remap
        };
        memory_listener_register(&container->cpr.remap_listener,
                                 bcontainer->space->as);
        memory_listener_unregister(&container->cpr.remap_listener);
        container->cpr.vaddr_unmapped = false;
        vioc->dma_map = vfio_legacy_dma_map;
    }
    return 0;
}

/**
 * @brief Registers legacy CPR support for a VFIO container.
 *
 * Adds migration notifiers and VM state registration for checkpoint/restore (CPR) support. If the container does not support required CPR features, adds a migration blocker for CPR transfer mode. During incoming CPR, diverts DMA mapping calls to the legacy CPR DMA map function.
 *
 * @param container The VFIO container to register for CPR support.
 * @param errp Pointer to an Error pointer for reporting errors.
 * @return true if registration succeeds or is blocked due to unsupported features; false if migration blocker registration fails.
 */
bool vfio_legacy_cpr_register_container(VFIOContainer *container, Error **errp)
{
    VFIOContainerBase *bcontainer = &container->bcontainer;
    Error **cpr_blocker = &container->cpr.blocker;

    migration_add_notifier_mode(&bcontainer->cpr_reboot_notifier,
                                vfio_cpr_reboot_notifier,
                                MIG_MODE_CPR_REBOOT);

    if (!vfio_cpr_supported(container, cpr_blocker)) {
        return migrate_add_blocker_modes(cpr_blocker, errp,
                                         MIG_MODE_CPR_TRANSFER, -1) == 0;
    }

    vmstate_register(NULL, -1, &vfio_container_vmstate, container);

    /* During incoming CPR, divert calls to dma_map. */
    if (container->cpr.reused) {
        VFIOIOMMUClass *vioc = VFIO_IOMMU_GET_CLASS(bcontainer);
        vioc->dma_map = vfio_legacy_cpr_dma_map;
    }

    migration_add_notifier_mode(&container->cpr.transfer_notifier,
                                vfio_cpr_fail_notifier,
                                MIG_MODE_CPR_TRANSFER);
    return true;
}

/**
 * @brief Unregisters legacy CPR support for a VFIO container.
 *
 * Removes migration notifiers, deletes migration blockers, and unregisters VM state associated with the container's legacy checkpoint/restore functionality.
 */
void vfio_legacy_cpr_unregister_container(VFIOContainer *container)
{
    VFIOContainerBase *bcontainer = &container->bcontainer;

    migration_remove_notifier(&bcontainer->cpr_reboot_notifier);
    migrate_del_blocker(&container->cpr.blocker);
    vmstate_unregister(NULL, &vfio_container_vmstate, container);
    migration_remove_notifier(&container->cpr.transfer_notifier);
}

/**
 * @brief Restores DMA virtual address mappings for a memory region section backed by a guest IOMMU.
 *
 * Finds the corresponding VFIOGuestIOMMU for the given memory region section and replays its IOMMU mappings to restore lost virtual address mappings, typically after a failed unmap during migration.
 */
void vfio_cpr_giommu_remap(VFIOContainerBase *bcontainer,
                           MemoryRegionSection *section)
{
    VFIOGuestIOMMU *giommu = NULL;
    hwaddr as_offset = section->offset_within_address_space;
    hwaddr iommu_offset = as_offset - section->offset_within_region;

    QLIST_FOREACH(giommu, &bcontainer->giommu_list, giommu_next) {
        if (giommu->iommu_mr == IOMMU_MEMORY_REGION(section->mr) &&
            giommu->iommu_offset == iommu_offset) {
            break;
        }
    }
    g_assert(giommu);
    memory_region_iommu_replay(giommu->iommu_mr, &giommu->n);
}

/**
 * @brief Restores DMA virtual address mappings for a memory region section managed by a RAM discard listener.
 *
 * Finds the RAM discard listener associated with the given container and memory region section,
 * and invokes its populate notification to replay DMA mappings using the legacy CPR mechanism.
 *
 * @param bcontainer The VFIO container base.
 * @param section The memory region section to restore mappings for.
 * @return true if mappings were successfully restored, false otherwise.
 */
bool vfio_cpr_ram_discard_register_listener(VFIOContainerBase *bcontainer,
                                            MemoryRegionSection *section)
{
    VFIORamDiscardListener *vrdl =
        vfio_find_ram_discard_listener(bcontainer, section);

    g_assert(vrdl);
    return vrdl->listener.notify_populate(&vrdl->listener, section) == 0;
}

/**
 * @brief Checks if two file descriptors refer to the same device.
 *
 * Compares the device IDs of the provided file descriptors using fstat.
 *
 * @param fd1 First file descriptor.
 * @param fd2 Second file descriptor.
 * @return true if both file descriptors refer to the same device, false otherwise.
 */
static bool same_device(int fd1, int fd2)
{
    struct stat st1, st2;

    return !fstat(fd1, &st1) && !fstat(fd2, &st2) && st1.st_dev == st2.st_dev;
}

/**
 * @brief Checks if a VFIO container's file descriptor matches or refers to the same device as a group's saved fd, deduplicating if necessary.
 *
 * If the container's fd and the provided fd are identical, returns true. If they refer to the same device but are different fds (due to duplication during CPR save), closes the duplicate, updates the saved fd to the container's fd, and returns true. Returns false if the fds refer to different devices.
 *
 * @param pfd Pointer to the group's saved file descriptor; updated if deduplication occurs.
 * @return true if the container matches or is deduplicated successfully; false otherwise.
 */
bool vfio_cpr_container_match(VFIOContainer *container, VFIOGroup *group,
                              int *pfd)
{
    if (container->fd == *pfd) {
        return true;
    }
    if (!same_device(container->fd, *pfd)) {
        return false;
    }
    /*
     * Same device, different fd.  This occurs when the container fd is
     * cpr_save'd multiple times, once for each groupid, so SCM_RIGHTS
     * produces duplicates.  De-dup it.
     */
    cpr_delete_fd("vfio_container_for_group", group->groupid);
    close(*pfd);
    cpr_save_fd("vfio_container_for_group", group->groupid, container->fd);
    *pfd = container->fd;
    return true;
}
