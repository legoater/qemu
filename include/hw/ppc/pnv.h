#ifndef _HW_LPC_H
#define _HW_LPC_H
/*
 * QEMU PowerNV various definitions
 *
 * Copyright (c) 2014 BenH
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/hw.h"
typedef struct XScomBus XScomBus;
typedef struct ISABus ISABus;
typedef struct PnvLpcController PnvLpcController;
typedef struct XICSState XICSState;

/* Should we turn that into a QOjb of some sort ? */
typedef struct PnvChip {
    uint32_t         chip_id;
    XScomBus         *xscom;
    PnvLpcController *lpc;
    ISABus           *lpc_bus;
} PnvChip;

typedef struct PnvSystem {
    XICSState *xics;
    uint32_t  num_chips;
#define PNV_MAX_CHIPS		1
    PnvChip   chips[PNV_MAX_CHIPS];
} PnvSystem;

extern void pnv_lpc_create(PnvChip *chip, bool has_serirq);
#endif /* _HW_PNV_LPC_H */

