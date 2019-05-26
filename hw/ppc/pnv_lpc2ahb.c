/*
 * QEMU PowerNV SuperIO iLPC2AHB bridge device
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "sysemu/blockdev.h"
#include "hw/ppc/pnv_lpc2ahb.h"

/*
 * BMC SoC definitions
*/
static const AspeedSoCInfo aspeed_socs[] = {
    {
        .name         = "palmetto-bmc",
        .silicon_rev  = AST2400_A1_SILICON_REV,
        .spi_typename = "aspeed.smc.spi",
        .spi_model    = "mx25l25635e",
    },
    {
        .name         = "witherspoon-bmc",
        .silicon_rev  = AST2500_A1_SILICON_REV,
        .spi_typename = "aspeed.smc.ast2500-spi1",
        .spi_model    = "mx66l1g45g",
    },
};

#define SCU_BASE                0x1e6e2000
#define LPC_BASE                0x1e789000
#define SPI_BASE                0x1e630000

static void pnv_lpc2ahb_instance_init(Object *obj)
{
    PnvLpc2Ahb *s = PNV_LPC2AHB(obj);
    PnvLpc2AhbClass *plac = PNV_LPC2AHB_GET_CLASS(obj);
    AspeedSoCInfo *soc = plac->soc;

    object_initialize(&s->scu, sizeof(s->scu), TYPE_ASPEED_SCU);
    object_property_add_child(obj, "scu", OBJECT(&s->scu), NULL);
    qdev_set_parent_bus(DEVICE(&s->scu), sysbus_get_default());
    qdev_prop_set_uint32(DEVICE(&s->scu), "silicon-rev", soc->silicon_rev);

    object_initialize(&s->lpc, sizeof(s->lpc), TYPE_ASPEED_LPC);
    object_property_add_child(obj, "lpc", OBJECT(&s->lpc), NULL);
    qdev_set_parent_bus(DEVICE(&s->lpc), sysbus_get_default());

    object_initialize(&s->spi, sizeof(s->spi), soc->spi_typename);
    object_property_add_child(obj, "spi", OBJECT(&s->spi), NULL);
    qdev_set_parent_bus(DEVICE(&s->spi), sysbus_get_default());
}

static void pnv_lpc2ahb_realize(DeviceState *dev, Error **errp)
{
    PnvLpc2Ahb *s = PNV_LPC2AHB(dev);
    Error *local_err = NULL;
    Object *obj;
    AspeedSioLpc2Ahb *sio_lpc2ahb;

    obj = object_property_get_link(OBJECT(dev), "sio", &local_err);
    if (!obj) {
        error_propagate(errp, local_err);
        error_prepend(errp, "required link 'sio' not found: ");
        return;
    }
    sio_lpc2ahb = &ASPEED_SIO(obj)->lpc2ahb;

    object_property_set_bool(OBJECT(&s->scu), true, "realized", &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    memory_region_add_subregion(&sio_lpc2ahb->ahb_mr, SCU_BASE, &s->scu.iomem);

    object_property_set_int(OBJECT(&s->lpc), PNOR_SPI_OFFSET >> 16, "hicr7",
                            &error_abort);
    object_property_set_bool(OBJECT(&s->lpc), true, "realized", &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    memory_region_add_subregion(&sio_lpc2ahb->ahb_mr, LPC_BASE, &s->lpc.iomem);

    object_property_set_int(OBJECT(&s->spi), 1, "num-cs",
                            &error_abort);
    object_property_set_bool(OBJECT(&s->spi), true, "realized", &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    memory_region_add_subregion(&sio_lpc2ahb->ahb_mr, SPI_BASE, &s->spi.mmio);
}

static void pnv_lpc2ahb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PnvLpc2AhbClass *plac = PNV_LPC2AHB_CLASS(klass);

    dc->realize = pnv_lpc2ahb_realize;
    plac->soc = (AspeedSoCInfo *) data;
}

static const TypeInfo pnv_lpc2ahb_info = {
    .name          = TYPE_PNV_LPC2AHB,
    .parent        = TYPE_DEVICE,
    .instance_init = pnv_lpc2ahb_instance_init,
    .instance_size = sizeof(PnvLpc2Ahb),
    .class_init    = pnv_lpc2ahb_class_init,
    .class_size     = sizeof(PnvLpc2AhbClass),
    .abstract       = true,
};

static void pnv_lpc2ahb_register_types(void)
{
    int i;

    type_register_static(&pnv_lpc2ahb_info);
    for (i = 0; i < ARRAY_SIZE(aspeed_socs); ++i) {
        TypeInfo ti = {
            .name       = aspeed_socs[i].name,
            .parent     = TYPE_PNV_LPC2AHB,
            .class_data = (void *) &aspeed_socs[i],
            .class_init    = pnv_lpc2ahb_class_init,
        };
        type_register(&ti);
    }
}

type_init(pnv_lpc2ahb_register_types)

/*
 * Exact same routine on Aspeed machines
 */
static void aspeed_board_init_flashes(AspeedSMCState *s, const char *flashtype,
                                      Error **errp)
{
    int i ;

    for (i = 0; i < s->num_cs; ++i) {
        AspeedSMCFlash *fl = &s->flashes[0];
        DriveInfo *dinfo = drive_get_next(IF_MTD);
        qemu_irq cs_line;

        fl->flash = ssi_create_slave_no_init(s->spi, flashtype);
        if (dinfo) {
            qdev_prop_set_drive(fl->flash, "drive", blk_by_legacy_dinfo(dinfo),
                            errp);
        }
        qdev_init_nofail(fl->flash);

        cs_line = qdev_get_gpio_in_named(fl->flash, SSI_GPIO_CS, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(s), 1, cs_line);
    }
}

PnvLpc2Ahb *pnv_lpc2ahb_create(AspeedSio *sio, const char *soc_name,
                               Error **errp)
{
    Object *obj;
    Error *local_err = NULL;
    PnvLpc2Ahb *lpc2ahb;
    AspeedSoCInfo *soc;

    obj = object_new(soc_name);
    object_property_add_const_link(obj, "sio", OBJECT(sio), &error_abort);
    object_property_set_bool(obj, true, "realized", &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return NULL;
    }
    lpc2ahb = PNV_LPC2AHB(obj);
    soc = PNV_LPC2AHB_GET_CLASS(obj)->soc;

    aspeed_board_init_flashes(&lpc2ahb->spi, soc->spi_model, &error_fatal);
    return lpc2ahb;
}
