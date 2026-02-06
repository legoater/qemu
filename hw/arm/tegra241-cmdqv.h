/*
 * Copyright (C) 2025, NVIDIA CORPORATION
 * NVIDIA Tegra241 CMDQ-Virtualiisation extension for SMMUv3
 *
 * Written by Nicolin Chen, Shameer Kolothum
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TEGRA241_CMDQV_H
#define HW_TEGRA241_CMDQV_H

#include "smmuv3-accel.h"
#include CONFIG_DEVICES

/*
 * Tegra241 CMDQV MMIO layout (64KB pages):
 *
 * 0x00000: Global CMDQV registers
 * 0x10000: Global VCMDQ registers, page 0
 * 0x20000: Global VCMDQ registers, page 1
 * 0x30000: VINTF0 logical VCMDQ registers, page 0
 * 0x40000: VINTF0 logical VCMDQ registers, page 1
 */
#define TEGRA241_CMDQV_IO_LEN 0x50000

typedef struct Tegra241CMDQV {
    struct iommu_viommu_tegra241_cmdqv cmdqv_data;
    SMMUv3AccelState *s_accel;
    MemoryRegion mmio_cmdqv;
    qemu_irq irq;
    void *vintf_page0;
} Tegra241CMDQV;

#define VINTF_REG_PAGE_SIZE 0x10000

#ifdef CONFIG_TEGRA241_CMDQV
const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void);
#else
static inline const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void)
{
    return NULL;
}
#endif
#endif /* HW_TEGRA241_CMDQV_H */
