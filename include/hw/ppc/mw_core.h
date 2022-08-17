/*
 * QEMU PowerPC Microwatt CPU Core model
 *
 * Copyright (c) 2021, IBM Corporation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef PPC_MW_CORE_H
#define PPC_MW_CORE_H

#include "hw/cpu/core.h"
#include "target/ppc/cpu.h"
#include "qom/object.h"

typedef struct MwSoCState MwSoCState;

#define TYPE_MW_CORE "mw-cpu-core"
OBJECT_DECLARE_TYPE(MwCore, MwCoreClass, MW_CORE)

struct MwCore {
    /*< private >*/
    CPUCore parent_obj;

    /*< public >*/
    PowerPCCPU **threads;
    uint32_t pir;

    MwSoCState *soc;
};

#define MW_CORE_TYPE_SUFFIX "-" TYPE_MW_CORE
#define MW_CORE_TYPE_NAME(cpu_model) cpu_model MW_CORE_TYPE_SUFFIX

typedef struct MwCPUState {
    Object *intc;
} MwCPUState;

static inline MwCPUState *mw_cpu_state(PowerPCCPU *cpu)
{
    return (MwCPUState *)cpu->machine_data;
}
#endif /* PPC_MW_CORE_H */
