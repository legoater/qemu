/*
 * QEMU PowerPC Microwatt
 *
 * Copyright (c) 2021, IBM Corporation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef PPC_MICROWATT_H
#define PPC_MICROWATT_H

#include "qom/object.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/ppc/mw_core.h"
#include "hw/ppc/xics.h"
#include "hw/net/liteeth.h"
#include "hw/ssi/lite_spi.h"
#include "hw/misc/unimp.h"

struct MwSoCState;
struct MwICPState {
    SysBusDevice parent;

    struct MwSoCState   *soc;
    MemoryRegion mmio;
};

#define TYPE_MW_ICP "mw-icp"
OBJECT_DECLARE_SIMPLE_TYPE(MwICPState, MW_ICP)

#define MW_SYSCON_REG(reg) ((reg) >> 3)

struct MwSysConState {
    SysBusDevice parent;

    struct MwSoCState   *soc;
    uint64_t regs[MW_SYSCON_REG(0x100)];
    uint64_t info;
    uint64_t ram_size;
    MemoryRegion mmio;
};

#define TYPE_MW_SYSCON "mw-syscon"
OBJECT_DECLARE_SIMPLE_TYPE(MwSysConState, MW_SYSCON)

struct MwICSState {
    SysBusDevice parent;

    ICSState ics;
    MemoryRegion mmio;
    MemoryRegion reg_mmio;
    MemoryRegion xive_mmio;
};

#define TYPE_MW_ICS "mw-ics"
OBJECT_DECLARE_SIMPLE_TYPE(MwICSState, MW_ICS)

#define MW_SOC_MAX_CPUS 1

struct MwSoCState {
    SysBusDevice  parent_obj;

    MwCore        cores[MW_SOC_MAX_CPUS];

    MwSysConState syscon;
    MwICPState    icp;
    MwICSState    ics;
    XICSFabric    *xics;
    LiteEthState  eth;
    LiteSPIState  spi;
    UnimplementedDeviceState gpio;

    MemoryRegion  dram_init;

    struct {
        uint64_t     entry;
        uint64_t     dt_base;
    } boot_info;
};

#define TYPE_MW_SOC "mw-soc"
OBJECT_DECLARE_TYPE(MwSoCState, MwSoCClass, MW_SOC)

struct MwSoCClass {
    SysBusDeviceClass parent_class;

    const char *cpu_type;
    uint32_t num_cpus;

    /* Interrupt presenter abstraction. Probably unnecessary */
    void (*intc_create)(MwSoCState *soc, PowerPCCPU *cpu, Error **errp);
    void (*intc_reset)(MwSoCState *soc, PowerPCCPU *cpu);
    void (*intc_destroy)(MwSoCState *soc, PowerPCCPU *cpu);
    void (*intc_print_info)(MwSoCState *soc, PowerPCCPU *cpu, Monitor *mon);
 };

#define MW_TIMEBASE_FREQ     100 * 1000 * 1000

enum {
    MW_DEV_IOMEM,
    MW_DEV_SYSCON,
    MW_DEV_UART0,
    MW_DEV_UART1,
    MW_DEV_ICP,
    MW_DEV_ICS,
    MW_DEV_SPI,
    MW_DEV_GPIO,
    MW_DEV_DRAM,
    MW_DEV_ETH,
    MW_DEV_ETH_BUF,
    MW_DEV_SD,
};

#define MW_SOC_MEMORY_BASE      0x00000000 /* Block RAM or DRAM */
#define MW_SOC_DRAM_BASE        0x40000000
#define MW_SOC_BRAM_BASE        0x80000000
#define MW_SOC_IOMEM_BASE       0xC0000000
#define MW_SOC_IOMEM_SIZE       0x30000000

#define MW_SOC_EXT_IO_BASE      0xC8000000

#define MW_SOC_FLASH_BASE       0xF0000000
#define MW_SOC_DRAM_INIT        0xFF000000

#endif
