/*
 * QEMU ASPEED MBOX Controller
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */


#include "qemu/osdep.h"
#include "qemu-common.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "exec/address-spaces.h"
#include "hw/isa/aspeed_mbox.h"

#define BMC_MBOX_READ_REGS 16

#define MBOX_FLAG_REG 0x0f
#define MBOX_STATUS_0 0x10
#define MBOX_STATUS_1 0x11
#define   MBOX_STATUS_1_ATTN (1 << 7)
#define   MBOX_STATUS_1_RESP (1 << 5)
#define MBOX_BMC_CTRL 0x12
#define   MBOX_CTRL_INT_STATUS (1 << 7)
#define   MBOX_CTRL_INT_MASK (1 << 1)
#define   MBOX_CTRL_INT_PING (1 << 0)
#define   MBOX_CTRL_INT_SEND (MBOX_CTRL_INT_PING | MBOX_CTRL_INT_MASK)
#define MBOX_HOST_CTRL 0x13
#define MBOX_BMC_INT_EN_0 0x14
#define MBOX_BMC_INT_EN_1 0x15
#define MBOX_HOST_INT_EN_0 0x16
#define MBOX_HOST_INT_EN_1 0x17

#define MBOX_MAX_QUEUE_LEN 5

static void aspeed_mbox_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    AspeedMbox *mbox = opaque;

    switch (addr) {
    case 0x0 ... ASPEED_MBOX_NR_REG:
        mbox->regs[addr] = val;
        break;
    }
}

static uint64_t aspeed_mbox_read(void *opaque, hwaddr addr, unsigned size)
{
    AspeedMbox *mbox = opaque;

    return mbox->regs[addr];
}

static const MemoryRegionOps aspeed_mbox_io_ops = {
    .read = aspeed_mbox_read,
    .write = aspeed_mbox_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void aspeed_mbox_reset(DeviceState *dev)
{
    AspeedMbox *mbox = ASPEED_MBOX(dev);

    memset(mbox->regs, 0, sizeof mbox->regs);
}

static void aspeed_mbox_realize(DeviceState *dev, Error **errp)
{
    ISADevice *isadev = ISA_DEVICE(dev);
    AspeedMbox *mbox = ASPEED_MBOX(isadev);
    int iobase = 0x1000;

    memory_region_init_io(&mbox->io, OBJECT(mbox), &aspeed_mbox_io_ops, mbox,
                          "aspeed-mbox", ASPEED_MBOX_NR_REG);
    qdev_set_legacy_instance_id(dev, iobase, 3);

    isa_register_ioport(isadev, &mbox->io, iobase);

    isa_init_irq(ISA_DEVICE(dev), &mbox->irq, mbox->isairq);
}

static const VMStateDescription aspeed_mbox_vmstate = {
    .name = TYPE_ASPEED_MBOX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, AspeedMbox, ASPEED_MBOX_NR_REG),
        VMSTATE_END_OF_LIST()
    }
};

static Property aspeed_mbox_properties[] = {
    DEFINE_PROP_UINT32("irq", AspeedMbox, isairq, 9),
    DEFINE_PROP_END_OF_LIST(),
};

static void aspeed_mbox_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_mbox_realize;
    dc->reset   = aspeed_mbox_reset;
    dc->vmsd    = &aspeed_mbox_vmstate;
    device_class_set_props(dc, aspeed_mbox_properties);
}

static const TypeInfo aspeed_mbox_info = {
    .name          = TYPE_ASPEED_MBOX,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(AspeedMbox),
    .class_init    = aspeed_mbox_class_init,
};

static void aspeed_mbox_register_types(void)
{
    type_register_static(&aspeed_mbox_info);
}

type_init(aspeed_mbox_register_types)

AspeedMbox *aspeed_mbox_create(ISABus *isabus)
{
    ISADevice *isadev;

    isadev = isa_new(TYPE_ASPEED_MBOX);
    isa_realize_and_unref(isadev, isabus, &error_fatal);
    return ASPEED_MBOX(isadev);
}
