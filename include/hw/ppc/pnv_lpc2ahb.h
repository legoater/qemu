#ifndef _HW_PNV_LPC2AHB_H
#define _HW_PNV_LPC2AHB_H
/*
 * QEMU PowerNV SuperIO iLPC2AHB bridge device
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "hw/isa/aspeed_sio.h"
#include "hw/misc/aspeed_scu.h"
#include "hw/misc/aspeed_lpc.h"
#include "hw/ssi/aspeed_smc.h"

#define TYPE_PNV_LPC2AHB  "pnv-lpc2ahb"
#define PNV_LPC2AHB(obj) \
    OBJECT_CHECK(PnvLpc2Ahb, (obj), TYPE_PNV_LPC2AHB)

typedef struct PnvLpc2Ahb {
    DeviceState    parent_obj;

    AspeedSCUState scu;
    AspeedLPCState lpc;
    AspeedSMCState spi;
} PnvLpc2Ahb;

#define PNV_LPC2AHB_CLASS(klass)                                         \
    OBJECT_CLASS_CHECK(PnvLpc2AhbClass, (klass), TYPE_PNV_LPC2AHB)
#define PNV_LPC2AHB_GET_CLASS(obj)                               \
    OBJECT_GET_CLASS(PnvLpc2AhbClass, (obj), TYPE_PNV_LPC2AHB)

typedef struct AspeedSoCInfo {
    const char *name;
    uint32_t silicon_rev;
    const char *spi_typename;
    const char *spi_model;
} AspeedSoCInfo;

typedef struct PnvLpc2AhbClass {
    DeviceClass parent_class;
    AspeedSoCInfo *soc;
} PnvLpc2AhbClass;

/*
 * PNOR offset on the LPC FW address space
 */
#define PNOR_SPI_OFFSET         0x0c000000

PnvLpc2Ahb *pnv_lpc2ahb_create(AspeedSio *sio, const char *soc_name,
                               Error **errp);

#endif /* _HW_PNV_LPC2AHB_H */
