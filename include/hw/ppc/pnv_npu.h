/*
 * QEMU PowerPC PowerNV Processor Nvlink Processing Unit (NPU) model
 *
 * Copyright (c) 2019-2020, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#ifndef PPC_PNV_NPU_H
#define PPC_PNV_NPU_H

#include "hw/ppc/pnv_xscom.h"

#define TYPE_PNV_NPU2 "pnv-npu2"
#define PNV_NPU2(obj) OBJECT_CHECK(PnvNpu2, (obj), TYPE_PNV_NPU2)

#define NPU2_STACK_COUNT 3
#define NPU2_BRICK_COUNT 6

typedef struct PnvNpu2 {
    DeviceState parent;

    struct PnvChip *chip;

    MemoryRegion xscom_obus0_regs;
    MemoryRegion xscom_obus0_indirect_regs;
    uint64_t xscom_obus0_indirect_addr;

    MemoryRegion xscom_regs1;
    uint64_t scom[PNV9_XSCOM_NPU_SIZE1];
    MemoryRegion xscom_regs2;

    MemoryRegion genid_mr[NPU2_STACK_COUNT];
    uint64_t genid_bar[NPU2_STACK_COUNT];

    bool fence_state[NPU2_BRICK_COUNT];
} PnvNpu2;

#endif /* PPC_PNV_NPU_H */
