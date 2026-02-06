/*
 * Copyright (C) 2025, NVIDIA CORPORATION
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

static bool tegra241_cmdqv_mmap_vintf_page0(Tegra241CMDQV *cmdqv, Error **errp)
{
    IOMMUFDViommu *viommu = cmdqv->s_accel->viommu;

    if (!viommu) {
        return true;
    }

    g_assert(!cmdqv->vintf_page0);
    if (!iommufd_backend_viommu_mmap(viommu->iommufd, viommu->viommu_id,
                                     VINTF_REG_PAGE_SIZE,
                                     cmdqv->cmdqv_data.out_vintf_mmap_offset,
                                     &cmdqv->vintf_page0, errp)) {
        return false;
    }

    return true;
}

/*
 * Read a VCMDQ register using VCMDQ0_* offsets.
 *
 * The caller normalizes the MMIO offset such that @offset0 always refers
 * to a VCMDQ0_* register, while @index selects the VCMDQ instance.
 *
 * All VCMDQ accesses are currently trapped. Use cached registers
 */
static uint64_t tegra241_cmdqv_read_vcmdq(Tegra241CMDQV *cmdqv, hwaddr offset0,
                                          int index)
{
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
    Error *local_err = NULL;
    int index;

    if (!cmdqv->vintf_page0) {
        if (!tegra241_cmdqv_mmap_vintf_page0(cmdqv, &local_err)) {
            error_report_err(local_err);
            local_err = NULL;
        }
    }

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
    case A_CMDQ_ALLOC_MAP_0 ... A_CMDQ_ALLOC_MAP_127:
        return cmdqv->cmdq_alloc_map[(offset - A_CMDQ_ALLOC_MAP_0) / 4];
    case A_VINTF0_CONFIG ... A_VINTF0_LVCMDQ_ERR_MAP_3:
        return tegra241_cmdqv_read_vintf(cmdqv, offset);
    case A_VI_VCMDQ0_CONS_INDX ... A_VI_VCMDQ127_GERRORN:
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
    case A_VCMDQ0_CONS_INDX ... A_VCMDQ127_GERRORN:
        /*
         * Decode a per-VCMDQ register access.
         *
         * VCMDQs are 128 identical instances, each occupying a 0x80-byte window
         * starting at 0x10000. The MMIO offset is decoded to extract the VCMDQ
         * index, and the per-instance offset is normalized to a VCMDQ0_*
         * register (@offset0 = offset - 0x80 * index).
         *
         * A single helper then services all VCMDQs, with @index selecting the
         * instance.
         */
        index = (offset - 0x10000) / 0x80;
        return tegra241_cmdqv_read_vcmdq(cmdqv, offset - 0x80 * index, index);
    case A_VI_VCMDQ0_BASE_L ... A_VI_VCMDQ127_CONS_INDX_BASE_DRAM_H:
        /* Same as A_VI_VCMDQ0_CONS_INDX ... A_VI_VCMDQ127_GERRORN case above */
        offset -= 0x20000;
        QEMU_FALLTHROUGH;
    case A_VCMDQ0_BASE_L ... A_VCMDQ127_CONS_INDX_BASE_DRAM_H:
        /* Same as A_VCMDQ0_CONS_INDX ... A_VCMDQ127_GERRORN case above */
        index = (offset - 0x20000) / 0x80;
        return tegra241_cmdqv_read_vcmdq(cmdqv, offset - 0x80 * index, index);
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled read access at 0x%" PRIx64 "\n",
                      __func__, offset);
        return 0;
    }
}

static void tegra241_cmdqv_write_vintf(Tegra241CMDQV *cmdqv, hwaddr offset,
                                       uint64_t value)
{
    switch (offset) {
    case A_VINTF0_CONFIG:
        /* Strip off HYP_OWN setting from guest kernel */
        value &= ~R_VINTF0_CONFIG_HYP_OWN_MASK;

        cmdqv->vintf_config = value;
        if (value & R_VINTF0_CONFIG_ENABLE_MASK) {
            cmdqv->vintf_status |= R_VINTF0_STATUS_ENABLE_OK_MASK;
        } else {
            cmdqv->vintf_status &= ~R_VINTF0_STATUS_ENABLE_OK_MASK;
        }
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

    if (!cmdqv->vintf_page0) {
        if (!tegra241_cmdqv_mmap_vintf_page0(cmdqv, &local_err)) {
            error_report_err(local_err);
            local_err = NULL;
        }
    }

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
            cmdqv->status &= ~R_STATUS_CMDQV_ENABLED_MASK;
        }
        break;
    case A_VI_INT_MASK ... A_VI_INT_MASK_1:
        cmdqv->vi_int_mask[(offset - A_VI_INT_MASK) / 4] = value;
        break;
    case A_CMDQ_ALLOC_MAP_0 ... A_CMDQ_ALLOC_MAP_127:
        cmdqv->cmdq_alloc_map[(offset - A_CMDQ_ALLOC_MAP_0) / 4] = value;
        break;
    case A_VINTF0_CONFIG ... A_VINTF0_LVCMDQ_ERR_MAP_3:
        tegra241_cmdqv_write_vintf(cmdqv, offset, value);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled write access at 0x%" PRIx64 "\n",
                      __func__, offset);
    }
}

static void tegra241_cmdqv_free_veventq(SMMUv3State *s)
{
}

static bool tegra241_cmdqv_alloc_veventq(SMMUv3State *s, Error **errp)
{
    error_setg(errp, "NVIDIA Tegra241 CMDQV is unsupported");
    return false;
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
        error_append_hint(errp, "Tegra241 CMDQV support unavailable");
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

    if (!accel) {
        error_setg(errp, "Tegra241 CMDQV requires SMMUv3 acceleration");
        return false;
    }

    cmdqv = g_new0(Tegra241CMDQV, 1);
    memory_region_init_io(&cmdqv->mmio_cmdqv, OBJECT(s), &mmio_cmdqv_ops, cmdqv,
                          "tegra241-cmdqv", TEGRA241_CMDQV_IO_LEN);
    sysbus_init_mmio(sbd, &cmdqv->mmio_cmdqv);
    sysbus_init_irq(sbd, &cmdqv->irq);
    cmdqv->s_accel = accel;
    accel->cmdqv = cmdqv;
    return true;
}

static const SMMUv3AccelCmdqvOps tegra241_cmdqv_ops_impl = {
    .init = tegra241_cmdqv_init,
    .alloc_viommu = tegra241_cmdqv_alloc_viommu,
    .free_viommu = tegra241_cmdqv_free_viommu,
    .alloc_veventq = tegra241_cmdqv_alloc_veventq,
    .free_veventq = tegra241_cmdqv_free_veventq,
    .reset = tegra241_cmdqv_reset,
};

const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void)
{
    return &tegra241_cmdqv_ops_impl;
}
