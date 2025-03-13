/*
 * ASPEED SoC 2700 family
 *
 * Copyright (C) 2025 ASPEED Technology Inc.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "system/block-backend.h"
#include "system/system.h"
#include "hw/arm/aspeed.h"
#include "hw/boards.h"
#include "hw/qdev-clock.h"
#include "hw/arm/aspeed_soc.h"
#include "hw/loader.h"
#include "hw/arm/boot.h"
#include "hw/block/flash.h"


#define TYPE_AST2700A0FC MACHINE_TYPE_NAME("ast2700fc-a0")
OBJECT_DECLARE_SIMPLE_TYPE(Ast2700a0FCState, AST2700A0FC);
#define TYPE_AST2700A1FC MACHINE_TYPE_NAME("ast2700fc-a1")
OBJECT_DECLARE_SIMPLE_TYPE(Ast2700a1FCState, AST2700A1FC);

static struct arm_boot_info ast2700fc_board_info = {
    .board_id = -1, /* device-tree-only board */
};

struct Ast2700a0FCState {
    MachineState parent_obj;

    MemoryRegion ca35_memory;
    MemoryRegion ca35_dram;
    MemoryRegion ssp_memory;
    MemoryRegion tsp_memory;

    Clock *ssp_sysclk;
    Clock *tsp_sysclk;

    Aspeed27x0SoCState ca35;
    Aspeed27x0SSPSoCState ssp;
    Aspeed27x0TSPSoCState tsp;

    bool mmio_exec;
};

struct Ast2700a1FCState {
    MachineState parent_obj;

    MemoryRegion ca35_memory;
    MemoryRegion ca35_dram;
    MemoryRegion ssp_memory;
    MemoryRegion tsp_memory;

    Clock *ssp_sysclk;
    Clock *tsp_sysclk;

    Aspeed27x0SoCState ca35;
    Aspeed27x0SSPSoCState ssp;
    Aspeed27x0TSPSoCState tsp;

    bool mmio_exec;
};

#define AST2700FC_BMC_RAM_SIZE (2 * GiB)
#define AST2700FC_CM4_DRAM_SIZE (32 * MiB)

#define AST2700FC_HW_STRAP1 0x000000C0
#define AST2700FC_HW_STRAP2 0x00000003
#define AST2700FC_FMC_MODEL "w25q01jvq"
#define AST2700FC_SPI_MODEL "w25q512jv"

static void ast2700a0fc_ca35_init(MachineState *machine)
{
    Ast2700a0FCState *s = AST2700A0FC(machine);
    AspeedSoCState *soc;
    AspeedSoCClass *sc;

    object_initialize_child(OBJECT(s), "ca35", &s->ca35, "ast2700-a0");
    soc = ASPEED_SOC(&s->ca35);
    sc = ASPEED_SOC_GET_CLASS(soc);

    memory_region_init(&s->ca35_memory, OBJECT(&s->ca35), "ca35-memory",
                       UINT64_MAX);

    if (!memory_region_init_ram(&s->ca35_dram, OBJECT(&s->ca35), "ca35-dram",
                                AST2700FC_BMC_RAM_SIZE, &error_abort)) {
        return;
    }
    if (!object_property_set_link(OBJECT(&s->ca35), "memory",
                                  OBJECT(&s->ca35_memory),
                                  &error_abort)) {
        return;
    };
    if (!object_property_set_link(OBJECT(&s->ca35), "dram",
                                  OBJECT(&s->ca35_dram), &error_abort)) {
        return;
    }
    if (!object_property_set_int(OBJECT(&s->ca35), "ram-size",
                                 AST2700FC_BMC_RAM_SIZE, &error_abort)) {
        return;
    }
    if (!object_property_set_int(OBJECT(&s->ca35), "hw-strap1",
                                 AST2700FC_HW_STRAP1, &error_abort)) {
        return;
    }
    if (!object_property_set_int(OBJECT(&s->ca35), "hw-strap2",
                                 AST2700FC_HW_STRAP2, &error_abort)) {
        return;
    }
    aspeed_soc_uart_set_chr(soc, ASPEED_DEV_UART12, serial_hd(0));
    if (!qdev_realize(DEVICE(&s->ca35), NULL, &error_abort)) {
        return;
    }
    /*
     * AST2700 EVB has a LM75 temperature sensor on I2C bus 0 at address 0x4d.
     */
    i2c_slave_create_simple(aspeed_i2c_get_bus(&soc->i2c, 0), "tmp105", 0x4d);

    aspeed_board_init_flashes(&soc->fmc, AST2700FC_FMC_MODEL, 2, 0);
    aspeed_board_init_flashes(&soc->spi[0], AST2700FC_SPI_MODEL, 1, 2);

    ast2700fc_board_info.ram_size = machine->ram_size;
    ast2700fc_board_info.loader_start = sc->memmap[ASPEED_DEV_SDRAM];

    arm_load_kernel(ARM_CPU(first_cpu), machine, &ast2700fc_board_info);
}

static void ast2700a1fc_ca35_init(MachineState *machine)
{
    Ast2700a1FCState *s = AST2700A1FC(machine);
    AspeedSoCState *soc;
    AspeedSoCClass *sc;

    object_initialize_child(OBJECT(s), "ca35", &s->ca35, "ast2700-a1");
    soc = ASPEED_SOC(&s->ca35);
    sc = ASPEED_SOC_GET_CLASS(soc);

    memory_region_init(&s->ca35_memory, OBJECT(&s->ca35), "ca35-memory",
                       UINT64_MAX);

    if (!memory_region_init_ram(&s->ca35_dram, OBJECT(&s->ca35), "ca35-dram",
                                AST2700FC_BMC_RAM_SIZE, &error_abort)) {
        return;
    }
    if (!object_property_set_link(OBJECT(&s->ca35), "memory",
                                  OBJECT(&s->ca35_memory),
                                  &error_abort)) {
        return;
    };
    if (!object_property_set_link(OBJECT(&s->ca35), "dram",
                                  OBJECT(&s->ca35_dram), &error_abort)) {
        return;
    }
    if (!object_property_set_int(OBJECT(&s->ca35), "ram-size",
                                 AST2700FC_BMC_RAM_SIZE, &error_abort)) {
        return;
    }
    if (!object_property_set_int(OBJECT(&s->ca35), "hw-strap1",
                                 AST2700FC_HW_STRAP1, &error_abort)) {
        return;
    }
    if (!object_property_set_int(OBJECT(&s->ca35), "hw-strap2",
                                 AST2700FC_HW_STRAP2, &error_abort)) {
        return;
    }
    aspeed_soc_uart_set_chr(soc, ASPEED_DEV_UART12, serial_hd(0));
    if (!qdev_realize(DEVICE(&s->ca35), NULL, &error_abort)) {
        return;
    }

    /*
     * AST2700 EVB has a LM75 temperature sensor on I2C bus 0 at address 0x4d.
     */
    i2c_slave_create_simple(aspeed_i2c_get_bus(&soc->i2c, 0), "tmp105", 0x4d);

    aspeed_board_init_flashes(&soc->fmc, AST2700FC_FMC_MODEL, 2, 0);
    aspeed_board_init_flashes(&soc->spi[0], AST2700FC_SPI_MODEL, 1, 2);

    ast2700fc_board_info.ram_size = machine->ram_size;
    ast2700fc_board_info.loader_start = sc->memmap[ASPEED_DEV_SDRAM];

    arm_load_kernel(ARM_CPU(first_cpu), machine, &ast2700fc_board_info);
}

static void ast2700a0fc_ssp_init(MachineState *machine)
{
    AspeedSoCState *soc;
    Ast2700a0FCState *s = AST2700A0FC(machine);
    s->ssp_sysclk = clock_new(OBJECT(s), "SSP_SYSCLK");
    clock_set_hz(s->ssp_sysclk, 200000000ULL);

    object_initialize_child(OBJECT(s), "ssp", &s->ssp, "ast2700ssp-a0");
    memory_region_init(&s->ssp_memory, OBJECT(&s->ssp), "ssp-memory",
                       UINT64_MAX);

    qdev_connect_clock_in(DEVICE(&s->ssp), "sysclk", s->ssp_sysclk);
    if (!object_property_set_link(OBJECT(&s->ssp), "memory",
                                  OBJECT(&s->ssp_memory), &error_abort)) {
        return;
    }

    soc = ASPEED_SOC(&s->ssp);
    aspeed_soc_uart_set_chr(soc, ASPEED_DEV_UART4, serial_hd(1));
    if (!qdev_realize(DEVICE(&s->ssp), NULL, &error_abort)) {
        return;
    }
}

static void ast2700a1fc_ssp_init(MachineState *machine)
{
    AspeedSoCState *soc;
    Ast2700a1FCState *s = AST2700A1FC(machine);
    s->ssp_sysclk = clock_new(OBJECT(s), "SSP_SYSCLK");
    clock_set_hz(s->ssp_sysclk, 200000000ULL);

    object_initialize_child(OBJECT(s), "ssp", &s->ssp, "ast2700ssp-a1");
    memory_region_init(&s->ssp_memory, OBJECT(&s->ssp), "ssp-memory",
                       UINT64_MAX);

    qdev_connect_clock_in(DEVICE(&s->ssp), "sysclk", s->ssp_sysclk);
    if (!object_property_set_link(OBJECT(&s->ssp), "memory",
                                  OBJECT(&s->ssp_memory), &error_abort)) {
        return;
    }

    soc = ASPEED_SOC(&s->ssp);
    aspeed_soc_uart_set_chr(soc, ASPEED_DEV_UART4, serial_hd(1));
    if (!qdev_realize(DEVICE(&s->ssp), NULL, &error_abort)) {
        return;
    }
}

static void ast2700a0fc_tsp_init(MachineState *machine)
{
    AspeedSoCState *soc;
    Ast2700a0FCState *s = AST2700A0FC(machine);
    s->tsp_sysclk = clock_new(OBJECT(s), "TSP_SYSCLK");
    clock_set_hz(s->tsp_sysclk, 200000000ULL);

    object_initialize_child(OBJECT(s), "tsp", &s->tsp, "ast2700tsp-a0");
    memory_region_init(&s->tsp_memory, OBJECT(&s->tsp), "tsp-memory",
                       UINT64_MAX);

    qdev_connect_clock_in(DEVICE(&s->tsp), "sysclk", s->tsp_sysclk);
    if (!object_property_set_link(OBJECT(&s->tsp), "memory",
                                  OBJECT(&s->tsp_memory), &error_abort)) {
        return;
    }

    soc = ASPEED_SOC(&s->tsp);
    aspeed_soc_uart_set_chr(soc, ASPEED_DEV_UART4, serial_hd(2));
    if (!qdev_realize(DEVICE(&s->tsp), NULL, &error_abort)) {
        return;
    }
}

static void ast2700a1fc_tsp_init(MachineState *machine)
{
    AspeedSoCState *soc;
    Ast2700a1FCState *s = AST2700A1FC(machine);
    s->tsp_sysclk = clock_new(OBJECT(s), "TSP_SYSCLK");
    clock_set_hz(s->tsp_sysclk, 200000000ULL);

    object_initialize_child(OBJECT(s), "tsp", &s->tsp, "ast2700tsp-a1");
    memory_region_init(&s->tsp_memory, OBJECT(&s->tsp), "tsp-memory",
                       UINT64_MAX);

    qdev_connect_clock_in(DEVICE(&s->tsp), "sysclk", s->tsp_sysclk);
    if (!object_property_set_link(OBJECT(&s->tsp), "memory",
                                  OBJECT(&s->tsp_memory), &error_abort)) {
        return;
    }

    soc = ASPEED_SOC(&s->tsp);
    aspeed_soc_uart_set_chr(soc, ASPEED_DEV_UART7, serial_hd(2));
    if (!qdev_realize(DEVICE(&s->tsp), NULL, &error_abort)) {
        return;
    }
}

static void ast2700a0fc_init(MachineState *machine)
{
    ast2700a0fc_ca35_init(machine);
    ast2700a0fc_ssp_init(machine);
    ast2700a0fc_tsp_init(machine);
}

static void ast2700a1fc_init(MachineState *machine)
{
    ast2700a1fc_ca35_init(machine);
    ast2700a1fc_ssp_init(machine);
    ast2700a1fc_tsp_init(machine);
}

static void ast2700a0fc_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ast2700-a0 full core support";
    mc->init = ast2700a0fc_init;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 6;
}

static void ast2700a1fc_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->alias = "ast2700fc";
    mc->desc = "ast2700-a1 full core support";
    mc->init = ast2700a1fc_init;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 6;
}

static const TypeInfo ast2700fc_types[] = {
    {
        .name           = MACHINE_TYPE_NAME("ast2700fc-a0"),
        .parent         = TYPE_MACHINE,
        .class_init     = ast2700a0fc_class_init,
        .instance_size  = sizeof(Ast2700a0FCState),
    },
    {
        .name           = MACHINE_TYPE_NAME("ast2700fc-a1"),
        .parent         = TYPE_MACHINE,
        .class_init     = ast2700a1fc_class_init,
        .instance_size  = sizeof(Ast2700a1FCState),
    },
};

DEFINE_TYPES(ast2700fc_types)
