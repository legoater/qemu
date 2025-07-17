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
#include "qemu/datadir.h"
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


#define TYPE_AST2700A1FC MACHINE_TYPE_NAME("ast2700fc")
OBJECT_DECLARE_SIMPLE_TYPE(Ast2700FCState, AST2700A1FC);

static struct arm_boot_info ast2700fc_board_info = {
    .board_id = -1, /* device-tree-only board */
};

struct Ast2700FCState {
    MachineState parent_obj;

    MemoryRegion ca35_memory;
    MemoryRegion ca35_dram;
    MemoryRegion ca35_boot_rom;

    Aspeed27x0SoCState ca35;

    bool mmio_exec;
};

#define AST2700FC_BMC_RAM_SIZE (1 * GiB)

#define AST2700FC_HW_STRAP1 0x000000C0
#define AST2700FC_HW_STRAP2 0x00000003
#define AST2700FC_FMC_MODEL "w25q01jvq"
#define AST2700FC_SPI_MODEL "w25q512jv"
#define VBOOTROM_FILE_NAME  "ast27x0_bootrom.bin"

static void ast2700fc_ca35_load_vbootrom(AspeedSoCState *soc,
                                         const char *bios_name, Error **errp)
{
    g_autofree char *filename = NULL;
    int ret;

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (!filename) {
        error_setg(errp, "Could not find vbootrom image '%s'", bios_name);
        return;
    }

    ret = load_image_mr(filename, &soc->vbootrom);
    if (ret < 0) {
        error_setg(errp, "Failed to load vbootrom image '%s'", bios_name);
        return;
    }
}

static void ast2700fc_ca35_write_boot_rom(DriveInfo *dinfo, hwaddr addr,
                                         size_t rom_size, Error **errp)
{
    BlockBackend *blk = blk_by_legacy_dinfo(dinfo);
    g_autofree void *storage = NULL;
    int64_t size;

    /*
     * The block backend size should have already been 'validated' by
     * the creation of the m25p80 object.
     */
    size = blk_getlength(blk);
    if (size <= 0) {
        error_setg(errp, "failed to get flash size");
        return;
    }

    if (rom_size > size) {
        rom_size = size;
    }

    storage = g_malloc0(rom_size);
    if (blk_pread(blk, 0, rom_size, storage, 0) < 0) {
        error_setg(errp, "failed to read the initial flash content");
        return;
    }

    rom_add_blob_fixed("aspeed.boot_rom", storage, rom_size, addr);
}

static void ast2700fc_ca35_init(MachineState *machine)
{
    Ast2700FCState *s = AST2700A1FC(machine);
    const char *bios_name = NULL;
    AspeedSoCState *soc;
    AspeedSoCClass *sc;
    uint64_t rom_size;
    DriveInfo *mtd0;

    object_initialize_child(OBJECT(s), "ca35", &s->ca35, "ast2700-a1");
    soc = ASPEED_SOC(&s->ca35);
    sc = ASPEED_SOC_GET_CLASS(soc);

    memory_region_init(&s->ca35_memory, OBJECT(&s->ca35), "ca35-memory",
                       UINT64_MAX);
    memory_region_add_subregion(get_system_memory(), 0, &s->ca35_memory);

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

    for (int i = 0; i < sc->macs_num; i++) {
        if (!qemu_configure_nic_device(DEVICE(&soc->ftgmac100[i]),
                                       true, NULL)) {
            break;
        }
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
    aspeed_soc_uart_set_chr(ASPEED_SOC(&s->ca35.ssp), ASPEED_DEV_UART4,
                            serial_hd(1));
    aspeed_soc_uart_set_chr(ASPEED_SOC(&s->ca35.tsp), ASPEED_DEV_UART7,
                            serial_hd(2));
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

    /* Install first FMC flash content as a boot rom. */
    if (!s->mmio_exec) {
        mtd0 = drive_get(IF_MTD, 0, 0);

        if (mtd0) {
            rom_size = memory_region_size(&soc->spi_boot);
            memory_region_init_rom(&s->ca35_boot_rom, NULL, "aspeed.boot_rom",
                                   rom_size, &error_abort);
            memory_region_add_subregion_overlap(&soc->spi_boot_container, 0,
                                                &s->ca35_boot_rom, 1);
            ast2700fc_ca35_write_boot_rom(mtd0,
                                          sc->memmap[ASPEED_DEV_SPI_BOOT],
                                          rom_size, &error_abort);
        }
    }

    /* VBOOTROM */
    bios_name = machine->firmware ?: VBOOTROM_FILE_NAME;
    ast2700fc_ca35_load_vbootrom(soc, bios_name, &error_abort);

    arm_load_kernel(ARM_CPU(first_cpu), machine, &ast2700fc_board_info);
}

static void ast2700fc_init(MachineState *machine)
{
    ast2700fc_ca35_init(machine);
}

static void ast2700fc_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->alias = "ast2700fc";
    mc->desc = "ast2700 full core support";
    mc->init = ast2700fc_init;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 6;
}

static const TypeInfo ast2700fc_types[] = {
    {
        .name           = MACHINE_TYPE_NAME("ast2700fc"),
        .parent         = TYPE_MACHINE,
        .class_init     = ast2700fc_class_init,
        .instance_size  = sizeof(Ast2700FCState),
    },
};

DEFINE_TYPES(ast2700fc_types)
