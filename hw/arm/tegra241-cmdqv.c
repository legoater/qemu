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
    default:
        qemu_log_mask(LOG_UNIMP, "%s unhandled read access at 0x%" PRIx64 "\n",
                      __func__, offset);
        return 0;
    }
}

static void tegra241_cmdqv_write_vintf(Tegra241CMDQV *cmdqv, hwaddr offset,
                                       uint64_t value)
{
    int i;

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
    case A_CMDQ_ALLOC_MAP_0 ... A_CMDQ_ALLOC_MAP_1:
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
    .reset = tegra241_cmdqv_reset,
};

const SMMUv3AccelCmdqvOps *tegra241_cmdqv_get_ops(void)
{
    return &tegra241_cmdqv_ops;
}
