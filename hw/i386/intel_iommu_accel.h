/*
 * Intel IOMMU acceleration with nested translation
 *
 * Copyright (C) 2025 Intel Corporation.
 *
 * Authors: Zhenzhong Duan <zhenzhong.duan@intel.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I386_INTEL_IOMMU_ACCEL_H
#define HW_I386_INTEL_IOMMU_ACCEL_H
#include CONFIG_DEVICES

#ifdef CONFIG_VTD_ACCEL
bool vtd_check_hiod_accel(IntelIOMMUState *s, VTDHostIOMMUDevice *vtd_hiod,
                          Error **errp);
VTDHostIOMMUDevice *vtd_find_hiod_iommufd(VTDAddressSpace *as);
#else
static inline bool vtd_check_hiod_accel(IntelIOMMUState *s,
                                        VTDHostIOMMUDevice *vtd_hiod,
                                        Error **errp)
{
    error_setg(errp,
               "host IOMMU is incompatible with guest first stage translation");
    return false;
}

static inline VTDHostIOMMUDevice *vtd_find_hiod_iommufd(VTDAddressSpace *as)
{
    return NULL;
}
#endif
#endif
