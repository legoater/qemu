/*
 * ASPEED AST2700 UFS Host Controller
 *
 * The AST2700 host controller (aspeed,ufshc-m31-16nm) is the generic sysbus
 * UFS controller, so this model only overrides the UFSHCI version.
 *
 * The clock/reset wrapper at 0x12c08000 (aspeed,ast2700-ufscnr) is modelled
 * elsewhere as an UnimplementedDevice.
 *
 * Copyright 2026 IBM Corp.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/ufs/aspeed_ufs.h"

/* UFSHCI 2.0, against the 4.1 the core reports */
#define ASPEED_UFS_HCI_VERSION 0x00000200

OBJECT_DECLARE_TYPE(AspeedUfsState, AspeedUfsClass, ASPEED_UFS)

struct AspeedUfsState {
    SysbusUfsState parent_obj;
};

struct AspeedUfsClass {
    DeviceClass parent_class;

    DeviceRealize parent_realize;
};

static void aspeed_ufs_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    AspeedUfsClass *ac = ASPEED_UFS_GET_CLASS(dev);
    SysbusUfsState *s = SYSBUS_UFS(dev);

    ac->parent_realize(dev, errp);
    if (*errp) {
        return;
    }

    s->ufs.reg.ver = ASPEED_UFS_HCI_VERSION;
}

static void aspeed_ufs_class_init(ObjectClass *oc, const void *data)
{
    AspeedUfsClass *ac = ASPEED_UFS_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_parent_realize(dc, aspeed_ufs_realize,
                                    &ac->parent_realize);
    dc->desc = "ASPEED UFS Host Controller";
}

static const TypeInfo aspeed_ufs_types[] = {
    {
        .name          = TYPE_ASPEED_UFS,
        .parent        = TYPE_SYSBUS_UFS,
        .instance_size = sizeof(AspeedUfsState),
        .class_size    = sizeof(AspeedUfsClass),
        .class_init    = aspeed_ufs_class_init,
    }
};

DEFINE_TYPES(aspeed_ufs_types)
