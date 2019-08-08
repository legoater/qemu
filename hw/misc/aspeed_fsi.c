/*
 *  ASPEED FSI Controller
 *
 *  Copyright (C) 2019 IBM Corp.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/misc/aspeed_fsi.h"
#include "qapi/error.h"

#define TO_REG(offset) ((offset) >> 2)

#define FSI_VERSION          TO_REG(0x00)

static uint64_t aspeed_fsi_read(void *opaque, hwaddr offset, unsigned size)
{
    AspeedFsiState *s = ASPEED_FSI(opaque);
    int reg = TO_REG(offset);

    if (reg >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    qemu_log_mask(LOG_UNIMP, "%s: read @0x%" HWADDR_PRIx " size=%d\n",
                  __func__, offset, size);
    return s->regs[reg];
}

static void aspeed_fsi_write(void *opaque, hwaddr offset, uint64_t data,
                             unsigned int size)
{
    AspeedFsiState *s = ASPEED_FSI(opaque);
    int reg = TO_REG(offset);

    if (reg >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }

    qemu_log_mask(LOG_UNIMP, "%s: write @0x%" HWADDR_PRIx " size=%d "
                  "value=%"PRIx64"\n", __func__, offset, size, data);
    s->regs[reg] = data;
}

static const MemoryRegionOps aspeed_fsi_ops = {
    .read = aspeed_fsi_read,
    .write = aspeed_fsi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aspeed_fsi_reset(DeviceState *dev)
{
    struct AspeedFsiState *s = ASPEED_FSI(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void aspeed_fsi_realize(DeviceState *dev, Error **errp)
{
    AspeedFsiState *s = ASPEED_FSI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_fsi_ops, s,
            TYPE_ASPEED_FSI, 0x100);

    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_aspeed_fsi = {
    .name = TYPE_ASPEED_FSI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedFsiState, ASPEED_FSI_NR_REGS),
        VMSTATE_END_OF_LIST(),
    }
};

static void aspeed_fsi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_fsi_realize;
    dc->reset = aspeed_fsi_reset;
    dc->desc = "Aspeed FSI Controller",
    dc->vmsd = &vmstate_aspeed_fsi;
}

static const TypeInfo aspeed_fsi_info = {
    .name = TYPE_ASPEED_FSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedFsiState),
    .class_init = aspeed_fsi_class_init,
};

static void aspeed_fsi_register_types(void)
{
    type_register_static(&aspeed_fsi_info);
}

type_init(aspeed_fsi_register_types);
