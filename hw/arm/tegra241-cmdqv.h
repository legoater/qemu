/*
 * Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved
 * NVIDIA Tegra241 CMDQ-Virtualiisation extension for SMMUv3
 *
 * Written by Nicolin Chen, Shameer Kolothum
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_TEGRA241_CMDQV_H
#define HW_ARM_TEGRA241_CMDQV_H

#define TEGRA241_CMDQV_VERSION             1
#define TEGRA241_CMDQV_NUM_CMDQ_LOG2       1
#define TEGRA241_CMDQV_MAX_CMDQ            (1U << TEGRA241_CMDQV_NUM_CMDQ_LOG2)
#define TEGRA241_CMDQV_NUM_SID_PER_VM_LOG2 4

/*
 * Tegra241 CMDQV MMIO layout (64KB pages)
 *
 * 0x00000  TEGRA241_CMDQV_CFG    (Global CMDQV configuration)
 * 0x10000  TEGRA241_VCMDQ_PAGE0  (Virtual CMDQ page 0)
 * 0x20000  TEGRA241_VCMDQ_PAGE1  (Virtual CMDQ page 1)
 * 0x30000  TEGRA241_VINTF0_PAGE0 (Virtual interface 0, page 0)
 * 0x40000  TEGRA241_VINTF0_PAGE1 (Virtual interface 0, page 1)
 */
#define TEGRA241_CMDQV_IO_LEN 0x50000

typedef struct Tegra241CMDQV {
    struct iommu_viommu_tegra241_cmdqv cmdqv_data;
    SMMUv3AccelState *s_accel;
    MemoryRegion mmio_cmdqv;
    qemu_irq irq;
} Tegra241CMDQV;

const SMMUv3AccelCmdqvOps *tegra241_cmdqv_get_ops(void);

#endif /* HW_ARM_TEGRA241_CMDQV_H */
