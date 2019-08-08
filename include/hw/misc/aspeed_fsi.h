/*
 *  ASPEED FSI Controller
 *
 *  Copyright (C) 2019 IBM Corp.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef ASPEED_FSI_H
#define ASPEED_FSI_H

#include "hw/sysbus.h"

#define TYPE_ASPEED_FSI "aspeed.fsi"
#define ASPEED_FSI(obj) OBJECT_CHECK(AspeedFsiState, (obj), TYPE_ASPEED_FSI)

#define ASPEED_FSI_NR_REGS (0x100 >> 2)

typedef struct AspeedFsiState {
    /* <private> */
    SysBusDevice parent;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[ASPEED_FSI_NR_REGS];
} AspeedFsiState;

#endif /* _ASPEED_FSI_H_ */
