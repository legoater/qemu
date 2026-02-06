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

#include "hw/core/registerfields.h"
#include "smmuv3-accel.h"
#include CONFIG_DEVICES

/*
 * Tegra241 CMDQV MMIO layout (64KB pages):
 *
 * 0x00000: Global CMDQV registers
 * 0x10000: Global VCMDQ registers, page 0
 * 0x20000: Global VCMDQ registers, page 1
 * 0x30000: VINTF0 logical VCMDQ registers, page 0
 * 0x40000: VINTF0 logical VCMDQ registers, page 1
 */
#define TEGRA241_CMDQV_IO_LEN 0x50000

typedef struct Tegra241CMDQV {
    struct iommu_viommu_tegra241_cmdqv cmdqv_data;
    SMMUv3AccelState *s_accel;
    MemoryRegion mmio_cmdqv;
    qemu_irq irq;
    void *vintf_page0;
    MemoryRegion mmio_vintf_page0;
    bool vintf_page0_mapped;
    IOMMUFDHWqueue *vcmdq[128];
    IOMMUFDVeventq *veventq;
    uint32_t last_event_seq;
    bool event_start;

    /* Register Cache */
    uint32_t config;
    uint32_t param;
    uint32_t status;
    uint32_t vi_err_map[2];
    uint32_t vi_int_mask[2];
    uint32_t cmdq_err_map[4];
    uint32_t cmdq_alloc_map[128];
    uint32_t vintf_config;
    uint32_t vintf_status;
    uint32_t vintf_cmdq_err_map[4];
    uint32_t vcmdq_cons_indx[128];
    uint32_t vcmdq_prod_indx[128];
    uint32_t vcmdq_config[128];
    uint32_t vcmdq_status[128];
    uint32_t vcmdq_gerror[128];
    uint32_t vcmdq_gerrorn[128];
    uint64_t vcmdq_base[128];
    uint64_t vcmdq_cons_indx_base[128];
} Tegra241CMDQV;

/* Global CMDQV MMIO registers (offset 0x00000) */
REG32(CONFIG, 0x0)
FIELD(CONFIG, CMDQV_EN, 0, 1)
FIELD(CONFIG, CMDQV_PER_CMD_OFFSET, 1, 3)
FIELD(CONFIG, CMDQ_MAX_CLK_BATCH, 4, 8)
FIELD(CONFIG, CMDQ_MAX_CMD_BATCH, 12, 8)
FIELD(CONFIG, CONS_DRAM_EN, 20, 1)

REG32(PARAM, 0x4)
FIELD(PARAM, CMDQV_VER, 0, 4)
FIELD(PARAM, CMDQV_NUM_CMDQ_LOG2, 4, 4)
FIELD(PARAM, CMDQV_NUM_VM_LOG2, 8, 4)
FIELD(PARAM, CMDQV_NUM_SID_PER_VM_LOG2, 12, 4)

REG32(STATUS, 0x8)
FIELD(STATUS, CMDQV_ENABLED, 0, 1)

#define A_VI_ERR_MAP 0x14
#define A_VI_ERR_MAP_1 0x18
#define V_VI_ERR_MAP_NO_ERROR (0)
#define V_VI_ERR_MAP_ERROR (1)

#define A_VI_INT_MASK 0x1c
#define A_VI_INT_MASK_1 0x20
#define V_VI_INT_MASK_NOT_MASKED (0)
#define V_VI_INT_MASK_MASKED (1)

#define A_CMDQ_ERR_MAP 0x24
#define A_CMDQ_ERR_MAP_1 0x28
#define A_CMDQ_ERR_MAP_2 0x2c
#define A_CMDQ_ERR_MAP_3 0x30

/* i = [0, 127] */
#define A_CMDQ_ALLOC_MAP_(i)                 \
    REG32(CMDQ_ALLOC_MAP_##i, 0x200 + i * 4) \
    FIELD(CMDQ_ALLOC_MAP_##i, ALLOC, 0, 1)   \
    FIELD(CMDQ_ALLOC_MAP_##i, LVCMDQ, 1, 7)  \
    FIELD(CMDQ_ALLOC_MAP_##i, VIRT_INTF_INDX, 15, 6)

A_CMDQ_ALLOC_MAP_(0)
/* Omitting 1~126 as not being directly called */
A_CMDQ_ALLOC_MAP_(127)


/* i = [0, 0] */
#define A_VINTFi_CONFIG(i)                       \
    REG32(VINTF##i##_CONFIG, 0x1000 + i * 0x100) \
    FIELD(VINTF##i##_CONFIG, ENABLE, 0, 1)       \
    FIELD(VINTF##i##_CONFIG, VMID, 1, 16)        \
    FIELD(VINTF##i##_CONFIG, HYP_OWN, 17, 1)

A_VINTFi_CONFIG(0)

#define A_VINTFi_STATUS(i)                       \
    REG32(VINTF##i##_STATUS, 0x1004 + i * 0x100) \
    FIELD(VINTF##i##_STATUS, ENABLE_OK, 0, 1)    \
    FIELD(VINTF##i##_STATUS, STATUS, 1, 3)       \
    FIELD(VINTF##i##_STATUS, VI_NUM_LVCMDQ, 16, 8)

A_VINTFi_STATUS(0)

#define V_VINTF_STATUS_NO_ERROR (0 << 1)
#define V_VINTF_STATUS_VCMDQ_EROR (1 << 1)

/* i = [0, 0], j = [0, 3] */
#define A_VINTFi_LVCMDQ_ERR_MAP_(i, j)                               \
    REG32(VINTF##i##_LVCMDQ_ERR_MAP_##j, 0x10c0 + j * 4 + i * 0x100) \
    FIELD(VINTF##i##_LVCMDQ_ERR_MAP_##j, LVCMDQ_ERR_MAP, 0, 32)

A_VINTFi_LVCMDQ_ERR_MAP_(0, 0)
/* Omitting [0][1~2] as not being directly called */
A_VINTFi_LVCMDQ_ERR_MAP_(0, 3)

/*
 * VCMDQ register windows.
 *
 * Page 0 @ 0x10000: VCMDQ control and status registers
 * Page 1 @ 0x20000: VCMDQ base and DRAM address registers
 */
#define A_VCMDQi_CONS_INDX(i)                       \
    REG32(VCMDQ##i##_CONS_INDX, 0x10000 + i * 0x80) \
    FIELD(VCMDQ##i##_CONS_INDX, RD, 0, 20)          \
    FIELD(VCMDQ##i##_CONS_INDX, ERR, 24, 7)

A_VCMDQi_CONS_INDX(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_CONS_INDX(127)

#define V_VCMDQ_CONS_INDX_ERR_CERROR_NONE 0
#define V_VCMDQ_CONS_INDX_ERR_CERROR_ILL_OPCODE 1
#define V_VCMDQ_CONS_INDX_ERR_CERROR_ABT 2
#define V_VCMDQ_CONS_INDX_ERR_CERROR_ATC_INV_SYNC 3
#define V_VCMDQ_CONS_INDX_ERR_CERROR_ILL_ACCESS 4

#define A_VCMDQi_PROD_INDX(i)                             \
    REG32(VCMDQ##i##_PROD_INDX, 0x10000 + 0x4 + i * 0x80) \
    FIELD(VCMDQ##i##_PROD_INDX, WR, 0, 20)

A_VCMDQi_PROD_INDX(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_PROD_INDX(127)

#define A_VCMDQi_CONFIG(i)                             \
    REG32(VCMDQ##i##_CONFIG, 0x10000 + 0x8 + i * 0x80) \
    FIELD(VCMDQ##i##_CONFIG, CMDQ_EN, 0, 1)

A_VCMDQi_CONFIG(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_CONFIG(127)

#define A_VCMDQi_STATUS(i)                             \
    REG32(VCMDQ##i##_STATUS, 0x10000 + 0xc + i * 0x80) \
    FIELD(VCMDQ##i##_STATUS, CMDQ_EN_OK, 0, 1)

A_VCMDQi_STATUS(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_STATUS(127)

#define A_VCMDQi_GERROR(i)                               \
    REG32(VCMDQ##i##_GERROR, 0x10000 + 0x10 + i * 0x80)  \
    FIELD(VCMDQ##i##_GERROR, CMDQ_ERR, 0, 1)             \
    FIELD(VCMDQ##i##_GERROR, CONS_DRAM_WR_ABT_ERR, 1, 1) \
    FIELD(VCMDQ##i##_GERROR, CMDQ_INIT_ERR, 2, 1)

A_VCMDQi_GERROR(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_GERROR(127)

#define A_VCMDQi_GERRORN(i)                               \
    REG32(VCMDQ##i##_GERRORN, 0x10000 + 0x14 + i * 0x80)  \
    FIELD(VCMDQ##i##_GERRORN, CMDQ_ERR, 0, 1)             \
    FIELD(VCMDQ##i##_GERRORN, CONS_DRAM_WR_ABT_ERR, 1, 1) \
    FIELD(VCMDQ##i##_GERRORN, CMDQ_INIT_ERR, 2, 1)

A_VCMDQi_GERRORN(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_GERRORN(127)

#define A_VCMDQi_BASE_L(i)                       \
    REG32(VCMDQ##i##_BASE_L, 0x20000 + i * 0x80) \
    FIELD(VCMDQ##i##_BASE_L, LOG2SIZE, 0, 5)     \
    FIELD(VCMDQ##i##_BASE_L, ADDR, 5, 27)

A_VCMDQi_BASE_L(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_BASE_L(127)

#define A_VCMDQi_BASE_H(i)                             \
    REG32(VCMDQ##i##_BASE_H, 0x20000 + 0x4 + i * 0x80) \
    FIELD(VCMDQ##i##_BASE_H, ADDR, 0, 16)

A_VCMDQi_BASE_H(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_BASE_H(127)

#define A_VCMDQi_CONS_INDX_BASE_DRAM_L(i)                             \
    REG32(VCMDQ##i##_CONS_INDX_BASE_DRAM_L, 0x20000 + 0x8 + i * 0x80) \
    FIELD(VCMDQ##i##_CONS_INDX_BASE_DRAM_L, ADDR, 0, 32)

A_VCMDQi_CONS_INDX_BASE_DRAM_L(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_CONS_INDX_BASE_DRAM_L(127)

#define A_VCMDQi_CONS_INDX_BASE_DRAM_H(i)                             \
    REG32(VCMDQ##i##_CONS_INDX_BASE_DRAM_H, 0x20000 + 0xc + i * 0x80) \
    FIELD(VCMDQ##i##_CONS_INDX_BASE_DRAM_H, ADDR, 0, 16)

A_VCMDQi_CONS_INDX_BASE_DRAM_H(0)
/* Omitting [1~126] as not being directly called */
A_VCMDQi_CONS_INDX_BASE_DRAM_H(127)

#define VINTF_REG_PAGE_SIZE 0x10000
/*
 * VI_VCMDQ register windows (VCMDQs mapped via VINTF).
 *
 * Page 0 @ 0x30000: VI_VCMDQ control and status registers
 * Page 1 @ 0x40000: VI_VCMDQ base and DRAM address registers
 */
#define A_VI_VCMDQi_CONS_INDX(i)                       \
    REG32(VI_VCMDQ##i##_CONS_INDX, 0x30000 + i * 0x80) \
    FIELD(VI_VCMDQ##i##_CONS_INDX, RD, 0, 20)          \
    FIELD(VI_VCMDQ##i##_CONS_INDX, ERR, 24, 7)

A_VI_VCMDQi_CONS_INDX(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_CONS_INDX(127)

#define A_VI_VCMDQi_PROD_INDX(i)                             \
    REG32(VI_VCMDQ##i##_PROD_INDX, 0x30000 + 0x4 + i * 0x80) \
    FIELD(VI_VCMDQ##i##_PROD_INDX, WR, 0, 20)

A_VI_VCMDQi_PROD_INDX(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_PROD_INDX(127)

#define A_VI_VCMDQi_CONFIG(i)                             \
    REG32(VI_VCMDQ##i##_CONFIG, 0x30000 + 0x8 + i * 0x80) \
    FIELD(VI_VCMDQ##i##_CONFIG, CMDQ_EN, 0, 1)

A_VI_VCMDQi_CONFIG(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_CONFIG(127)

#define A_VI_VCMDQi_STATUS(i)                             \
    REG32(VI_VCMDQ##i##_STATUS, 0x30000 + 0xc + i * 0x80) \
    FIELD(VI_VCMDQ##i##_STATUS, CMDQ_EN_OK, 0, 1)

A_VI_VCMDQi_STATUS(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_STATUS(127)

#define A_VI_VCMDQi_GERROR(i)                               \
    REG32(VI_VCMDQ##i##_GERROR, 0x30000 + 0x10 + i * 0x80)  \
    FIELD(VI_VCMDQ##i##_GERROR, CMDQ_ERR, 0, 1)             \
    FIELD(VI_VCMDQ##i##_GERROR, CONS_DRAM_WR_ABT_ERR, 1, 1) \
    FIELD(VI_VCMDQ##i##_GERROR, CMDQ_INIT_ERR, 2, 1)

A_VI_VCMDQi_GERROR(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_GERROR(127)

#define A_VI_VCMDQi_GERRORN(i)                               \
    REG32(VI_VCMDQ##i##_GERRORN, 0x30000 + 0x14 + i * 0x80)  \
    FIELD(VI_VCMDQ##i##_GERRORN, CMDQ_ERR, 0, 1)             \
    FIELD(VI_VCMDQ##i##_GERRORN, CONS_DRAM_WR_ABT_ERR, 1, 1) \
    FIELD(VI_VCMDQ##i##_GERRORN, CMDQ_INIT_ERR, 2, 1)

A_VI_VCMDQi_GERRORN(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_GERRORN(127)

#define A_VI_VCMDQi_BASE_L(i)                       \
    REG32(VI_VCMDQ##i##_BASE_L, 0x40000 + i * 0x80) \
    FIELD(VI_VCMDQ##i##_BASE_L, LOG2SIZE, 0, 5)     \
    FIELD(VI_VCMDQ##i##_BASE_L, ADDR, 5, 27)

A_VI_VCMDQi_BASE_L(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_BASE_L(127)

#define A_VI_VCMDQi_BASE_H(i)                             \
    REG32(VI_VCMDQ##i##_BASE_H, 0x40000 + 0x4 + i * 0x80) \
    FIELD(VI_VCMDQ##i##_BASE_H, ADDR, 0, 16)

A_VI_VCMDQi_BASE_H(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_BASE_H(127)

#define A_VI_VCMDQi_CONS_INDX_BASE_DRAM_L(i)                             \
    REG32(VI_VCMDQ##i##_CONS_INDX_BASE_DRAM_L, 0x40000 + 0x8 + i * 0x80) \
    FIELD(VI_VCMDQ##i##_CONS_INDX_BASE_DRAM_L, ADDR, 0, 32)

A_VI_VCMDQi_CONS_INDX_BASE_DRAM_L(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_CONS_INDX_BASE_DRAM_L(127)

#define A_VI_VCMDQi_CONS_INDX_BASE_DRAM_H(i)                             \
    REG32(VI_VCMDQ##i##_CONS_INDX_BASE_DRAM_H, 0x40000 + 0xc + i * 0x80) \
    FIELD(VI_VCMDQ##i##_CONS_INDX_BASE_DRAM_H, ADDR, 0, 16)

A_VI_VCMDQi_CONS_INDX_BASE_DRAM_H(0)
/* Omitting [1~126] as not being directly called */
A_VI_VCMDQi_CONS_INDX_BASE_DRAM_H(127)

#ifdef CONFIG_TEGRA241_CMDQV
const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void);
#else
static inline const SMMUv3AccelCmdqvOps *tegra241_cmdqv_ops(void)
{
    return NULL;
}
#endif
#endif /* HW_TEGRA241_CMDQV_H */
