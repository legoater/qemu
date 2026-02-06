/*
 * Copyright (C) 2025, NVIDIA CORPORATION
 * NVIDIA Tegra241 CMDQ-Virtualization extension for SMMUv3
 *
 * Written by Nicolin Chen, Shameer Kolothum
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "hw/arm/smmuv3.h"
#include "smmuv3-accel.h"
#include "tegra241-cmdqv.h"

static uint64_t tegra241_cmdqv_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static void tegra241_cmdqv_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
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
