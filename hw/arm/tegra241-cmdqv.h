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

#include CONFIG_DEVICES

#ifdef CONFIG_TEGRA241_CMDQV
const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void);
#else
static inline const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void)
{
    return NULL;
}
#endif
#endif /* HW_TEGRA241_CMDQV_H */
