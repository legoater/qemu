/*
 * QEMU PowerNV PNOR simple model
 *
 * Copyright (c) 2019, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */
#ifndef _PPC_PNV_PNOR_H
#define _PPC_PNV_PNOR_H
#include "qom/object.h"

/*
 * PNOR offset on the LPC FW address space
 */
#define PNOR_SPI_OFFSET         0x0c000000UL

#define TYPE_PNV_PNOR  "pnv-pnor"
OBJECT_DECLARE_SIMPLE_TYPE(PnvPnor, PNV_PNOR)

struct PnvPnor {
    SysBusDevice   parent_obj;

    BlockBackend   *blk;

    uint8_t        *storage;
    int64_t        size;
    MemoryRegion   mmio;

    uint32_t       skiboot_addr;
    uint32_t       skiboot_size;
};

extern int pnv_pnor_load_skiboot(PnvPnor *pnor, hwaddr addr, size_t max_size,
                                 Error **errp);

#endif /* _PPC_PNV_PNOR_H */
