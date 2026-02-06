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
}

static bool
tegra241_cmdqv_alloc_viommu(SMMUv3State *s, HostIOMMUDeviceIOMMUFD *idev,
                            uint32_t *out_viommu_id, Error **errp)
{
    error_setg(errp, "NVIDIA Tegra241 CMDQV is unsupported");
    return false;
}

static void tegra241_cmdqv_reset(SMMUv3State *s)
{
}

static bool tegra241_cmdqv_init(SMMUv3State *s, Error **errp)
{
    error_setg(errp, "NVIDIA Tegra241 CMDQV is unsupported");
    return false;
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
