/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2019 IBM Corp.
 *
 * IBM Common FRU Access Macro
 */
#ifndef FSI_CFAM_H
#define FSI_CFAM_H

#include "exec/memory.h"

#include "hw/fsi/fsi-slave.h"
#include "hw/fsi/lbus.h"

#define TYPE_CFAM "cfam"
#define CFAM(obj) OBJECT_CHECK(CFAMState, (obj), TYPE_CFAM)

#define CFAM_NR_REGS ((0x2e0 >> 2) + 1)

struct CFAMState {
    /* < private > */
    FSISlaveState parent;

    MemoryRegion mr;
    AddressSpace as;

    CFAMConfig config;
    CFAMPeek peek;

    LBus lbus;
};

#endif /* FSI_CFAM_H */
