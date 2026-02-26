/*
 * Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved
 * NVIDIA Tegra241 CMDQ-Virtualization extension for SMMUv3
 *
 * Written by Nicolin Chen, Shameer Kolothum
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"

#include "hw/arm/smmuv3.h"
#include "smmuv3-accel.h"
#include "tegra241-cmdqv.h"

static inline uint32_t *tegra241_cmdqv_vintf_ptr(Tegra241CMDQV *cmdqv,
                                                 int index, hwaddr offset0)
{
    if (!cmdqv->vcmdq[index] || !cmdqv->vintf_page0) {
        return NULL;
    }

    return (uint32_t *)(cmdqv->vintf_page0 + (index * 0x80) +
                        (offset0 - 0x10000));
}
/*
 * Read a VCMDQ register using VCMDQ0_* offsets.
 *
 * The caller normalizes the MMIO offset such that @offset0 always refers
 * to a VCMDQ0_* register, while @index selects the VCMDQ instance.
 *
 * If the VCMDQ is allocated and VINTF page0 is mmap'ed, read directly
 * from the VINTF page0 backing. Otherwise, fall back to cached state.
 */
static uint64_t tegra241_cmdqv_read_vcmdq(Tegra241CMDQV *cmdqv, hwaddr offset0,
                                          int index)
{
    uint32_t *ptr = tegra241_cmdqv_vintf_ptr(cmdqv, index, offset0);

    if (ptr) {
        switch (offset0) {
        case A_VCMDQ0_CONS_INDX:
        case A_VCMDQ0_PROD_INDX:
        case A_VCMDQ0_CONFIG:
        case A_VCMDQ0_STATUS:
        case A_VCMDQ0_GERROR:
        case A_VCMDQ0_GERRORN:
            return *ptr;
        default:
            break;
        }
    }

    switch (offset0) {
    case A_VCMDQ0_CONS_INDX:
        return cmdqv->vcmdq_cons_indx[index];
    case A_VCMDQ0_PROD_INDX:
        return cmdqv->vcmdq_prod_indx[index];
    case A_VCMDQ0_CONFIG:
        return cmdqv->vcmdq_config[index];
    case A_VCMDQ0_STATUS:
        return cmdqv->vcmdq_status[index];
    case A_VCMDQ0_GERROR:
        return cmdqv->vcmdq_gerror[index];
    case A_VCMDQ0_GERRORN:
        return cmdqv->vcmdq_gerrorn[index];
    case A_VCMDQ0_BASE_L:
        return cmdqv->vcmdq_base[index];
    case A_VCMDQ0_BASE_H:
        return cmdqv->vcmdq_base[index] >> 32;
    case A_VCMDQ0_CONS_INDX_BASE_DRAM_L:
        return cmdqv->vcmdq_cons_indx_base[index];
    case A_VCMDQ0_CONS_INDX_BASE_DRAM_H:
        return cmdqv->vcmdq_cons_indx_base[index] >> 32;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s unhandled read access at 0x%" PRIx64 "\n",
                      __func__, offset0);
        return 0;
    }
}

static uint64_t tegra241_cmdqv_read_vintf(Tegra241CMDQV *cmdqv, hwaddr offset)
{
    int i;

    switch (offset) {
    case A_VINTF0_CONFIG:
        return cmdqv->vintf_config;
    case A_VINTF0_STATUS:
        return cmdqv->vintf_status;
    case A_VINTF0_SID_MATCH_0 ... A_VINTF0_SID_MATCH_15:
        i = (offset - A_VINTF0_SID_MATCH_0) / 4;
        return cmdqv->vintf_sid_match[i];
    case A_VINTF0_SID_REPLACE_0 ... A_VINTF0_SID_REPLACE_15:
        i = (offset - A_VINTF0_SID_REPLACE_0) / 4;
        return cmdqv->vintf_sid_replace[i];
    case A_VINTF0_LVCMDQ_ERR_MAP_0 ... A_VINTF0_LVCMDQ_ERR_MAP_3:
        i = (offset - A_VINTF0_LVCMDQ_ERR_MAP_0) / 4;
        return cmdqv->vintf_cmdq_err_map[i];
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled read access at 0x%" PRIx64 "\n",
                      __func__, offset);
        return 0;
    }
}

static uint64_t tegra241_cmdqv_read(void *opaque, hwaddr offset, unsigned size)
{
    Tegra241CMDQV *cmdqv = (Tegra241CMDQV *)opaque;
    int index;

    if (offset >= TEGRA241_CMDQV_IO_LEN) {
        qemu_log_mask(LOG_UNIMP,
                      "%s offset 0x%" PRIx64 " off limit (0x50000)\n", __func__,
                      offset);
        return 0;
    }

    switch (offset) {
    case A_CONFIG:
        return cmdqv->config;
    case A_PARAM:
        return cmdqv->param;
    case A_STATUS:
        return cmdqv->status;
    case A_VI_ERR_MAP ... A_VI_ERR_MAP_1:
        return cmdqv->vi_err_map[(offset - A_VI_ERR_MAP) / 4];
    case A_VI_INT_MASK ... A_VI_INT_MASK_1:
        return cmdqv->vi_int_mask[(offset - A_VI_INT_MASK) / 4];
    case A_CMDQ_ERR_MAP ... A_CMDQ_ERR_MAP_3:
        return cmdqv->cmdq_err_map[(offset - A_CMDQ_ERR_MAP) / 4];
    case A_CMDQ_ALLOC_MAP_0 ... A_CMDQ_ALLOC_MAP_1:
        return cmdqv->cmdq_alloc_map[(offset - A_CMDQ_ALLOC_MAP_0) / 4];
    case A_VINTF0_CONFIG ... A_VINTF0_LVCMDQ_ERR_MAP_3:
        return tegra241_cmdqv_read_vintf(cmdqv, offset);
    case A_VI_VCMDQ0_CONS_INDX ... A_VI_VCMDQ1_GERRORN:
        /*
         * VI_VCMDQ registers (VINTF logical view) have the same per-VCMDQ
         * layout as the global VCMDQ registers, but are based at 0x30000
         * instead of 0x10000.
         *
         * Subtract 0x20000 to translate a VI_VCMDQ offset into the equivalent
         * global VCMDQ offset, then fall through to reuse the common VCMDQ
         * decoding logic below.
         */
        offset -= 0x20000;
        QEMU_FALLTHROUGH;
    case A_VCMDQ0_CONS_INDX ... A_VCMDQ1_GERRORN:
        /*
         * Decode a per-VCMDQ register access.
         *
         * The hardware supports up to 128 identical VCMDQ instances; we
         * currently expose TEGRA241_CMDQV_MAX_CMDQ (= 2). Each VCMDQ
         * occupies a 0x80-byte window starting at 0x10000.
         *
         * The MMIO offset is decoded to extract the VCMDQ index and normalized
         * to the corresponding VCMDQ0_* register by subtracting index * 0x80.
         *
         * A single helper then services all VCMDQs, with @index selecting the
         * instance.
         */
        index = (offset - 0x10000) / 0x80;
        return tegra241_cmdqv_read_vcmdq(cmdqv, offset - index * 0x80, index);
    case A_VI_VCMDQ0_BASE_L ... A_VI_VCMDQ1_CONS_INDX_BASE_DRAM_H:
        /* Same decode logic as A_VI_VCMDQx_CONS_INDX case above */
        offset -= 0x20000;
        QEMU_FALLTHROUGH;
    case A_VCMDQ0_BASE_L ... A_VCMDQ1_CONS_INDX_BASE_DRAM_H:
        /* Same decode logic as A_VCMDQx_CONS_INDX case above */
        index = (offset - 0x20000) / 0x80;
        return tegra241_cmdqv_read_vcmdq(cmdqv, offset - index * 0x80, index);
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled read access at 0x%" PRIx64 "\n",
                      __func__, offset);
        return 0;
    }
}

static void tegra241_cmdqv_guest_unmap_vintf_page0(Tegra241CMDQV *cmdqv)
{
    if (!cmdqv->mr_vintf_page0) {
        return;
    }

    memory_region_del_subregion(&cmdqv->mmio_cmdqv, cmdqv->mr_vintf_page0);
    object_unparent(OBJECT(cmdqv->mr_vintf_page0));
    g_free(cmdqv->mr_vintf_page0);
    cmdqv->mr_vintf_page0 = NULL;
}

static void tegra241_cmdqv_guest_map_vintf_page0(Tegra241CMDQV *cmdqv)
{
    char *name;

    if (cmdqv->mr_vintf_page0) {
        return;
    }

    name = g_strdup_printf("%s vintf-page0",
                           memory_region_name(&cmdqv->mmio_cmdqv));
    cmdqv->mr_vintf_page0 = g_malloc0(sizeof(*cmdqv->mr_vintf_page0));
    memory_region_init_ram_device_ptr(cmdqv->mr_vintf_page0,
                                      memory_region_owner(&cmdqv->mmio_cmdqv),
                                      name, VINTF_PAGE_SIZE,
                                      cmdqv->vintf_page0);
    memory_region_add_subregion_overlap(&cmdqv->mmio_cmdqv, 0x30000,
                                        cmdqv->mr_vintf_page0, 1);
    g_free(name);
}

static void tegra241_cmdqv_free_vcmdq(Tegra241CMDQV *cmdqv, int index)
{
    SMMUv3AccelState *accel = cmdqv->s_accel;
    IOMMUFDViommu *viommu = accel->viommu;
    IOMMUFDHWqueue *vcmdq = cmdqv->vcmdq[index];

    if (!vcmdq) {
        return;
    }
    iommufd_backend_free_id(viommu->iommufd, vcmdq->hw_queue_id);
    g_free(vcmdq);
    cmdqv->vcmdq[index] = NULL;
}

static void tegra241_cmdqv_free_all_vcmdq(Tegra241CMDQV *cmdqv)
{
    /* Free in the reverse order to avoid "resource busy" error */
    for (int i = (TEGRA241_CMDQV_MAX_CMDQ - 1); i >= 0; i--) {
        tegra241_cmdqv_free_vcmdq(cmdqv, i);
    }
}

static bool tegra241_cmdqv_setup_vcmdq(Tegra241CMDQV *cmdqv, int index,
                                       Error **errp)
{
    SMMUv3AccelState *accel = cmdqv->s_accel;
    uint64_t base_mask = (uint64_t)R_VCMDQ0_BASE_L_ADDR_MASK |
                         (uint64_t)R_VCMDQ0_BASE_H_ADDR_MASK << 32;
    uint64_t addr = cmdqv->vcmdq_base[index] & base_mask;
    uint64_t log2 = cmdqv->vcmdq_base[index] & R_VCMDQ0_BASE_L_LOG2SIZE_MASK;
    uint64_t size = 1ULL << (log2 + 4);
    IOMMUFDViommu *viommu = accel->viommu;
    IOMMUFDHWqueue *hw_queue;
    uint32_t hw_queue_id;

    /* Ignore any invalid address. This may come as part of reset etc */
    if (!address_space_is_ram(&address_space_memory, addr) ||
        !address_space_is_ram(&address_space_memory, addr + size - 1)) {
        return true;
    }

    if (!tegra241_cmdq_enabled(cmdqv) || !tegra241_vintf_enabled(cmdqv)) {
        return true;
    }

    tegra241_cmdqv_free_vcmdq(cmdqv, index);

    if (!iommufd_backend_alloc_hw_queue(viommu->iommufd, viommu->viommu_id,
                                        IOMMU_HW_QUEUE_TYPE_TEGRA241_CMDQV,
                                        index, addr, size, &hw_queue_id,
                                        errp)) {
        return false;
    }
    hw_queue = g_new(IOMMUFDHWqueue, 1);
    hw_queue->hw_queue_id = hw_queue_id;
    hw_queue->viommu = viommu;
    cmdqv->vcmdq[index] = hw_queue;

    tegra241_cmdqv_guest_map_vintf_page0(cmdqv);
    return true;
}

static bool
tegra241_cmdqv_munmap_vintf_page0(Tegra241CMDQV *cmdqv, Error **errp)
{
    if (!cmdqv->vintf_page0) {
        return true;
    }

    if (munmap(cmdqv->vintf_page0, VINTF_PAGE_SIZE) < 0) {
        error_setg_errno(errp, errno, "Failed to unmap VINTF page0");
        return false;
    }
    cmdqv->vintf_page0 = NULL;
    return true;
}

static bool tegra241_cmdqv_mmap_vintf_page0(Tegra241CMDQV *cmdqv, Error **errp)
{
    IOMMUFDViommu *viommu = cmdqv->s_accel->viommu;

    if (cmdqv->vintf_page0) {
        return true;
    }

    if (!iommufd_backend_viommu_mmap(viommu->iommufd, viommu->viommu_id,
                                     VINTF_PAGE_SIZE,
                                     cmdqv->cmdqv_data.out_vintf_mmap_offset,
                                     &cmdqv->vintf_page0, errp)) {
        return false;
    }

    return true;
}

/*
 * Write a VCMDQ register using VCMDQ0_* offsets.
 *
 * The caller normalizes the MMIO offset such that @offset0 always refers
 * to a VCMDQ0_* register, while @index selects the VCMDQ instance.
 *
 * If the VCMDQ is allocated and VINTF page0 is mmap'ed, write directly
 * to the VINTF page0 backing. Otherwise, update cached state.
 */
static void
tegra241_cmdqv_write_vcmdq(Tegra241CMDQV *cmdqv, hwaddr offset0, int index,
                           uint64_t value, unsigned size, Error **errp)
{
    uint32_t *ptr = tegra241_cmdqv_vintf_ptr(cmdqv, index, offset0);

    if (ptr) {
        switch (offset0) {
        case A_VCMDQ0_CONS_INDX:
        case A_VCMDQ0_PROD_INDX:
        case A_VCMDQ0_CONFIG:
        case A_VCMDQ0_GERRORN:
            *ptr = (uint32_t)value;
            return;
        default:
            break;
        }
    }

    switch (offset0) {
    case A_VCMDQ0_CONS_INDX:
        cmdqv->vcmdq_cons_indx[index] = value;
        return;
    case A_VCMDQ0_PROD_INDX:
        cmdqv->vcmdq_prod_indx[index] = (uint32_t)value;
        return;
    case A_VCMDQ0_CONFIG:
        if (value & R_VCMDQ0_CONFIG_CMDQ_EN_MASK) {
            cmdqv->vcmdq_status[index] |= R_VCMDQ0_STATUS_CMDQ_EN_OK_MASK;
        } else {
            cmdqv->vcmdq_status[index] &= ~R_VCMDQ0_STATUS_CMDQ_EN_OK_MASK;
        }
        cmdqv->vcmdq_config[index] = (uint32_t)value;
        return;
    case A_VCMDQ0_GERRORN:
        cmdqv->vcmdq_gerrorn[index] = (uint32_t)value;
        return;
    case A_VCMDQ0_BASE_L:
        if (size == 8) {
            cmdqv->vcmdq_base[index] = value;
        } else if (size == 4) {
            cmdqv->vcmdq_base[index] =
                (cmdqv->vcmdq_base[index] & 0xffffffff00000000ULL) |
                (value & 0xffffffffULL);
        }
        tegra241_cmdqv_setup_vcmdq(cmdqv, index, errp);
        return;
    case A_VCMDQ0_BASE_H:
        cmdqv->vcmdq_base[index] =
            (cmdqv->vcmdq_base[index] & 0xffffffffULL) |
            ((uint64_t)value << 32);
        tegra241_cmdqv_setup_vcmdq(cmdqv, index, errp);
        return;
    case A_VCMDQ0_CONS_INDX_BASE_DRAM_L:
        if (size == 8) {
            cmdqv->vcmdq_cons_indx_base[index] = value;
        } else if (size == 4) {
            cmdqv->vcmdq_cons_indx_base[index] =
                (cmdqv->vcmdq_cons_indx_base[index] & 0xffffffff00000000ULL) |
                (value & 0xffffffffULL);
        }
        return;
    case A_VCMDQ0_CONS_INDX_BASE_DRAM_H:
        cmdqv->vcmdq_cons_indx_base[index] =
            (cmdqv->vcmdq_cons_indx_base[index] & 0xffffffffULL) |
            ((uint64_t)value << 32);
        return;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s unhandled write access at 0x%" PRIx64 "\n",
                      __func__, offset0);
        return;
    }
}

static void tegra241_cmdqv_write_vintf(Tegra241CMDQV *cmdqv, hwaddr offset,
                                       uint64_t value, Error **errp)
{
    int i;

    switch (offset) {
    case A_VINTF0_CONFIG:
        /* Strip off HYP_OWN setting from guest kernel */
        value &= ~R_VINTF0_CONFIG_HYP_OWN_MASK;

        cmdqv->vintf_config = value;
        if (value & R_VINTF0_CONFIG_ENABLE_MASK) {
            tegra241_cmdqv_mmap_vintf_page0(cmdqv, errp);
            cmdqv->vintf_status |= R_VINTF0_STATUS_ENABLE_OK_MASK;
        } else {
            tegra241_cmdqv_guest_unmap_vintf_page0(cmdqv);
            tegra241_cmdqv_free_all_vcmdq(cmdqv);
            tegra241_cmdqv_munmap_vintf_page0(cmdqv, errp);
            cmdqv->vintf_status &= ~R_VINTF0_STATUS_ENABLE_OK_MASK;
        }
        break;
    case A_VINTF0_SID_MATCH_0 ... A_VINTF0_SID_MATCH_15:
        i = (offset - A_VINTF0_SID_MATCH_0) / 4;
        cmdqv->vintf_sid_match[i] = value;
        break;
    case A_VINTF0_SID_REPLACE_0 ... A_VINTF0_SID_REPLACE_15:
        i = (offset - A_VINTF0_SID_REPLACE_0) / 4;
        cmdqv->vintf_sid_replace[i] = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled write access at 0x%" PRIx64 "\n",
                      __func__, offset);
        return;
    }
}

static void tegra241_cmdqv_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    Tegra241CMDQV *cmdqv = (Tegra241CMDQV *)opaque;
    Error *local_err = NULL;
    int index;

    if (offset >= TEGRA241_CMDQV_IO_LEN) {
        qemu_log_mask(LOG_UNIMP,
                      "%s offset 0x%" PRIx64 " off limit (0x50000)\n", __func__,
                      offset);
        return;
    }

    switch (offset) {
    case A_CONFIG:
        cmdqv->config = value;
        if (value & R_CONFIG_CMDQV_EN_MASK) {
            cmdqv->status |= R_STATUS_CMDQV_ENABLED_MASK;
        } else {
            tegra241_cmdqv_guest_unmap_vintf_page0(cmdqv);
            tegra241_cmdqv_free_all_vcmdq(cmdqv);
            cmdqv->status &= ~R_STATUS_CMDQV_ENABLED_MASK;
        }
        break;
    case A_VI_INT_MASK ... A_VI_INT_MASK_1:
        cmdqv->vi_int_mask[(offset - A_VI_INT_MASK) / 4] = value;
        break;
    case A_CMDQ_ALLOC_MAP_0 ... A_CMDQ_ALLOC_MAP_1:
        cmdqv->cmdq_alloc_map[(offset - A_CMDQ_ALLOC_MAP_0) / 4] = value;
        break;
    case A_VINTF0_CONFIG ... A_VINTF0_LVCMDQ_ERR_MAP_3:
        tegra241_cmdqv_write_vintf(cmdqv, offset, value, &local_err);
        break;
    case A_VI_VCMDQ0_CONS_INDX ... A_VI_VCMDQ1_GERRORN:
        /* Same decoding as read() case: See comments above */
        offset -= 0x20000;
        QEMU_FALLTHROUGH;
    case A_VCMDQ0_CONS_INDX ... A_VCMDQ1_GERRORN:
        index = (offset - 0x10000) / 0x80;
        tegra241_cmdqv_write_vcmdq(cmdqv, offset - 0x80 * index, index, value,
                                   size, &local_err);
        break;
    case A_VI_VCMDQ0_BASE_L ... A_VI_VCMDQ1_CONS_INDX_BASE_DRAM_H:
        /* Same decoding as read() case: See comments above */
        offset -= 0x20000;
        QEMU_FALLTHROUGH;
    case A_VCMDQ0_BASE_L ... A_VCMDQ1_CONS_INDX_BASE_DRAM_H:
        index = (offset - 0x20000) / 0x80;
        tegra241_cmdqv_write_vcmdq(cmdqv, offset - 0x80 * index, index, value,
                                   size, &local_err);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled write access at 0x%" PRIx64 "\n",
                      __func__, offset);
    }

    if (local_err) {
        error_report_err(local_err);
    }
}

static void tegra241_cmdqv_free_veventq(SMMUv3State *s)
{
    SMMUv3AccelState *accel = s->s_accel;
    Tegra241CMDQV *cmdqv = accel->cmdqv;
    IOMMUFDVeventq *veventq = cmdqv->veventq;

    if (!veventq) {
        return;
    }
    close(veventq->veventq_fd);
    iommufd_backend_free_id(veventq->viommu->iommufd, veventq->veventq_id);
    g_free(veventq);
    cmdqv->veventq = NULL;
}

static bool tegra241_cmdqv_alloc_veventq(SMMUv3State *s, Error **errp)
{
    SMMUv3AccelState *accel = s->s_accel;
    IOMMUFDViommu *viommu = accel->viommu;
    Tegra241CMDQV *cmdqv = accel->cmdqv;
    IOMMUFDVeventq *veventq;
    uint32_t veventq_id;
    uint32_t veventq_fd;

    if (cmdqv->veventq) {
        return true;
    }

    if (!iommufd_backend_alloc_veventq(viommu->iommufd, viommu->viommu_id,
                                       IOMMU_VEVENTQ_TYPE_TEGRA241_CMDQV,
                                       1 << 16, &veventq_id, &veventq_fd,
                                       errp)) {
        error_append_hint(errp, "Tegra241 CMDQV: failed to alloc veventq");
        return false;
    }

    veventq = g_new(IOMMUFDVeventq, 1);
    veventq->veventq_id = veventq_id;
    veventq->veventq_fd = veventq_fd;
    veventq->viommu = accel->viommu;
    cmdqv->veventq = veventq;

    return true;
}

static void tegra241_cmdqv_free_viommu(SMMUv3State *s)
{
    SMMUv3AccelState *accel = s->s_accel;
    IOMMUFDViommu *viommu = accel->viommu;

    if (!viommu) {
        return;
    }
    iommufd_backend_free_id(viommu->iommufd, viommu->viommu_id);
}

static bool
tegra241_cmdqv_alloc_viommu(SMMUv3State *s, HostIOMMUDeviceIOMMUFD *idev,
                            uint32_t *out_viommu_id, Error **errp)
{
    Tegra241CMDQV *cmdqv = s->s_accel->cmdqv;

    if (!iommufd_backend_alloc_viommu(idev->iommufd, idev->devid,
                                      IOMMU_VIOMMU_TYPE_TEGRA241_CMDQV,
                                      idev->hwpt_id, &cmdqv->cmdqv_data,
                                      sizeof(cmdqv->cmdqv_data), out_viommu_id,
                                      errp)) {
        return false;
    }
    return true;
}

static void tegra241_cmdqv_reset(SMMUv3State *s)
{
}

static const MemoryRegionOps mmio_cmdqv_ops = {
    .read = tegra241_cmdqv_read,
    .write = tegra241_cmdqv_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static bool tegra241_cmdqv_init(SMMUv3State *s, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(OBJECT(s));
    SMMUv3AccelState *accel = s->s_accel;
    Tegra241CMDQV *cmdqv;

    cmdqv = g_new0(Tegra241CMDQV, 1);
    memory_region_init_io(&cmdqv->mmio_cmdqv, OBJECT(s), &mmio_cmdqv_ops, cmdqv,
                          "tegra241-cmdqv", TEGRA241_CMDQV_IO_LEN);
    sysbus_init_mmio(sbd, &cmdqv->mmio_cmdqv);
    sysbus_init_irq(sbd, &cmdqv->irq);
    cmdqv->s_accel = accel;
    accel->cmdqv = cmdqv;
    return true;
}

static bool tegra241_cmdqv_probe(SMMUv3State *s, HostIOMMUDeviceIOMMUFD *idev,
                                 Error **errp)
{
    uint32_t data_type = IOMMU_HW_INFO_TYPE_TEGRA241_CMDQV;
    struct iommu_hw_info_tegra241_cmdqv cmdqv_info;
    uint64_t caps;

    if (!iommufd_backend_get_device_info(idev->iommufd, idev->devid, &data_type,
                                         &cmdqv_info, sizeof(cmdqv_info), &caps,
                                         NULL, errp)) {
        return false;
    }
    if (data_type != IOMMU_HW_INFO_TYPE_TEGRA241_CMDQV) {
        error_setg(errp, "Host CMDQV: unexpected data type %u (expected %u)",
                   data_type, IOMMU_HW_INFO_TYPE_TEGRA241_CMDQV);
        return false;
    }
    if (cmdqv_info.version != TEGRA241_CMDQV_VERSION) {
        error_setg(errp, "Host CMDQV: unsupported version %u (expected %u)",
                   cmdqv_info.version, TEGRA241_CMDQV_VERSION);
        return false;
    }
    if (cmdqv_info.log2vcmdqs < TEGRA241_CMDQV_NUM_CMDQ_LOG2) {
        error_setg(errp, "Host CMDQV: insufficient vCMDQs log2=%u (need >= %u)",
                   cmdqv_info.log2vcmdqs, TEGRA241_CMDQV_NUM_CMDQ_LOG2);
        return false;
    }
    if (cmdqv_info.log2vsids < TEGRA241_CMDQV_NUM_SID_PER_VM_LOG2) {
        error_setg(errp, "Host CMDQV: insufficient SIDs log2=%u (need >= %u)",
                   cmdqv_info.log2vsids, TEGRA241_CMDQV_NUM_SID_PER_VM_LOG2);
        return false;
    }
    return true;
}

static const SMMUv3AccelCmdqvOps tegra241_cmdqv_ops = {
    .probe = tegra241_cmdqv_probe,
    .init = tegra241_cmdqv_init,
    .alloc_viommu = tegra241_cmdqv_alloc_viommu,
    .free_viommu = tegra241_cmdqv_free_viommu,
    .alloc_veventq = tegra241_cmdqv_alloc_veventq,
    .free_veventq = tegra241_cmdqv_free_veventq,
    .reset = tegra241_cmdqv_reset,
};

const SMMUv3AccelCmdqvOps *tegra241_cmdqv_get_ops(void)
{
    return &tegra241_cmdqv_ops;
}
