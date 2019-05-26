#ifndef HW_ASPEED_MBOX_H
#define HW_ASPEED_MBOX_H
/*
 * QEMU ASPEED MBOX Controller
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "hw/isa/isa.h"

/*
 * MBOX controller
 */
#define TYPE_ASPEED_MBOX  "aspeed.mbox"
#define ASPEED_MBOX(obj) OBJECT_CHECK(AspeedMbox, (obj), TYPE_ASPEED_MBOX)

#define ASPEED_MBOX_NR_REG  0x20

typedef struct AspeedMbox {
    ISADevice parent_obj;

    uint8_t regs[ASPEED_MBOX_NR_REG];

    uint32_t iobase;
    MemoryRegion io;
} AspeedMbox;

extern AspeedMbox *aspeed_mbox_create(ISABus *bus);

#endif /* HW_ASPEED_MBOX_H */
