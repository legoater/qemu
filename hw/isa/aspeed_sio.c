/*
 * QEMU ASPEED SuperIO Controller
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
#include "exec/address-spaces.h"
#include "hw/isa/aspeed_sio.h"

/*
 * SuperIO Controller
 */
static void aspeed_sio_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    AspeedSio *sio = opaque;
    AspeedSioDeviceId dev_id = sio->regs[ASPEED_SIO_REG_SELECT];
    AspeedSioDevice *dev;
    AspeedSioDeviceClass *sdc;

    val &= 0xff;

    /* Register port */

    if ((addr & 1) == 0) {
        switch (val) {
        case 0xa5:  /* write twice to unlock */
            if (sio->locked) {
                sio->locked--;
            }

            /* Reset logical device settings */
            if (!sio->locked) {
                sio->regs[ASPEED_SIO_REG_SELECT] = 0;
                sio->regs[ASPEED_SIO_REG_ENABLE] = 0;
            }
            break;
        case 0xaa:  /* write once to lock */
            sio->locked = 2;
            break;
        default:
            sio->reg = val;
            break;
        }
        return;
    };

    /* Data port */

    if (sio->locked) {
        qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: controller is locked\n");
        return;
    }

    /* Controller Registers */
    switch (sio->reg) {
    case ASPEED_SIO_REG_SELECT:
        if (val > 0xf) {
            qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: invalid device "
                          " number 0x%"PRIx64"\n", val);
            return;
        }
        sio->regs[ASPEED_SIO_REG_SELECT] = val & 0xf;
        sio->regs[ASPEED_SIO_REG_ENABLE] = 0;
        return;

    case 0x20:          /* interrupt bit and status */
    case 0x21 ... 0x27: /* SIO to BMC scratch registers */
    case 0x28 ... 0x2F: /* BMC to SIO scratch registers */
        sio->regs[sio->reg] = val;
        return;

    case ASPEED_SIO_REG_ENABLE:
        sio->regs[ASPEED_SIO_REG_ENABLE] = val & 0x1;
        break;

    /*
     * All devices have SerIRQ registers. Not modeled yet
     */
    case 0x70: /* SerIRQ id */
    case 0x71: /* SerIRQ type */
    case 0x72: /* SerIRQ id */
    case 0x73: /* SerIRQ type */
        sio->regs[sio->reg] = val;
        return;
    default:
        break;
    }

    dev = sio->devices[dev_id];
    if (!dev) {
        qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: unknow device %x\n",
                      dev_id);
        return;
    }

    sdc = ASPEED_SIO_DEVICE_GET_CLASS(dev);
    if (!sdc->is_enabled(dev)) {
        qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: device %x is disabled\n",
                      dev_id);
        return;
    }

    sdc->write(dev, sio->reg, val & 0xff);
}

static uint64_t aspeed_sio_read(void *opaque, hwaddr addr, unsigned size)
{
    AspeedSio *sio = ASPEED_SIO(opaque);
    AspeedSioDeviceId dev_id = sio->regs[ASPEED_SIO_REG_SELECT];
    AspeedSioDevice *dev;
    AspeedSioDeviceClass *sdc;

    if ((addr & 1) == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "aspeed_sio: register port should not be read\n");
        return -1;
    }

    /* allow scratch registers to be read even if device is locked */
    if (sio->reg < ASPEED_SIO_REG_ENABLE) {
        return sio->regs[sio->reg];
    }

    if (sio->locked) {
        qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: controller is locked\n");
        return -1;
    }

    dev = sio->devices[dev_id];
    if (!dev) {
        qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: unknown device %x\n",
                      dev_id);
        return -1;
    }

    sdc = ASPEED_SIO_DEVICE_GET_CLASS(dev);
    if (!sdc->is_enabled(dev)) {
        qemu_log_mask(LOG_GUEST_ERROR, "aspeed_sio: device %x is disabled\n",
                      dev_id);
        return -1;
    }

    return sdc->read(dev, sio->reg);
}

static const MemoryRegionOps aspeed_sio_io_ops = {
    .read = aspeed_sio_read,
    .write = aspeed_sio_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void aspeed_sio_reset(DeviceState *dev)
{
    AspeedSio *sio = ASPEED_SIO(dev);

    memset(sio->regs, 0, sizeof sio->regs);

    sio->locked = 2;
}

static void aspeed_sio_instance_init(Object *obj)
{
    AspeedSio *sio = ASPEED_SIO(obj);

    object_initialize(&sio->lpc2ahb, sizeof(sio->lpc2ahb),
                      TYPE_ASPEED_SIO_LPC2AHB);
    object_property_add_child(obj, "lpc2ahb", OBJECT(&sio->lpc2ahb), NULL);
}

static void aspeed_sio_realize(DeviceState *dev, Error **errp)
{
    ISADevice *isadev = ISA_DEVICE(dev);
    AspeedSio *sio = ASPEED_SIO(isadev);
    Error *local_err = NULL;

    qdev_set_legacy_instance_id(dev, sio->iobase, 3);

    memory_region_init_io(&sio->io, OBJECT(sio), &aspeed_sio_io_ops, sio,
                          "aspeed-sio", 2);
    isa_register_ioport(isadev, &sio->io, sio->iobase);

    /* iLPC2AHB device */
    object_property_add_const_link(OBJECT(&sio->lpc2ahb), "sio",
                                   OBJECT(sio), &error_fatal);
    object_property_set_bool(OBJECT(&sio->lpc2ahb), true, "realized",
                             &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
}

static const VMStateDescription aspeed_sio_vmstate = {
    .name = TYPE_ASPEED_SIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, AspeedSio, ASPEED_SIO_NR_REG),
        VMSTATE_UINT8(reg, AspeedSio),
        VMSTATE_UINT8(locked, AspeedSio),
        VMSTATE_END_OF_LIST()
    }
};

static Property aspeed_sio_properties[] = {
    DEFINE_PROP_UINT32("iobase", AspeedSio, iobase, 0x2e),
    DEFINE_PROP_END_OF_LIST(),
};

static void aspeed_sio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_sio_realize;
    dc->reset   = aspeed_sio_reset;
    dc->props   = aspeed_sio_properties;
    dc->vmsd    = &aspeed_sio_vmstate;
}

static const TypeInfo aspeed_sio_info = {
    .name          = TYPE_ASPEED_SIO,
    .parent        = TYPE_ISA_DEVICE,
    .instance_init = aspeed_sio_instance_init,
    .instance_size = sizeof(AspeedSio),
    .class_init    = aspeed_sio_class_init,
};

/*
 * SuperIO Device
 */

static bool aspeed_sio_device_is_enabled(AspeedSioDevice *s)
{
    return s->sio->regs[ASPEED_SIO_REG_ENABLE] & 0x1;
}

static void aspeed_sio_device_realize(DeviceState *dev, Error **errp)
{
    AspeedSioDevice *s = ASPEED_SIO_DEVICE(dev);
    AspeedSioDeviceClass *sdc = ASPEED_SIO_DEVICE_GET_CLASS(dev);
    Error *local_err = NULL;
    Object *obj;

    obj = object_property_get_link(OBJECT(dev), "sio", &local_err);
    if (!obj) {
        error_propagate(errp, local_err);
        error_prepend(errp, "required link 'sio' not found: ");
        return;
    }
    s->sio = ASPEED_SIO(obj);

    /* Link the device in the controller */
    s->sio->devices[sdc->id] = s;

    if (sdc->realize) {
        sdc->realize(dev, errp);
    }
}

static void aspeed_sio_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedSioDeviceClass *sdc = ASPEED_SIO_DEVICE_CLASS(klass);

    dc->realize = aspeed_sio_device_realize;

    /*
     * Default settings to be overridden by inheriting classes.
     */
    sdc->id = ASPEED_SIO_DEV_NONE;
    sdc->is_enabled = aspeed_sio_device_is_enabled;
}

static const TypeInfo aspeed_sio_device_info = {
    .name          = TYPE_ASPEED_SIO_DEVICE,
    .parent        = TYPE_DEVICE,
    .abstract      = true,
    .instance_size = sizeof(AspeedSioDevice),
    .class_init    = aspeed_sio_device_class_init,
    .class_size    = sizeof(AspeedSioDeviceClass),
};

/*
 * SuperIO iLPC2AHB bridge device
 */

static uint64_t aspeed_sio_lpc2ahb_rw(AspeedSioDevice *s, bool write)
{
    AspeedSio *sio = s->sio;
    AspeedSioLpc2Ahb *lpc2ahb = ASPEED_SIO_LPC2AHB(s);
    bool success;
    uint32_t addr;
    uint32_t data = -1;
    int sz = 1 << (sio->regs[0xf8] & 0x3);

    addr = (sio->regs[0xf0] << 24 |
            sio->regs[0xf1] << 16 |
            sio->regs[0xf2] << 8 |
            sio->regs[0xf3]);

    if (write) {
        data = (sio->regs[0xf4] << 24 |
                sio->regs[0xf5] << 16 |
                sio->regs[0xf6] << 8  |
                sio->regs[0xf7]);
    }

    success = !address_space_rw(&lpc2ahb->ahb_as, addr, MEMTXATTRS_UNSPECIFIED,
                                (uint8_t *) &data, sz, write);
    if (!success) {
        qemu_log_mask(LOG_GUEST_ERROR, "lpc2ahb: %s to address 0x%08x failed\n",
                      write ? "write" : "read", addr);
        return -1;
    }

    if (!write) {
        sio->regs[0xf4] = (data >> 24) & 0xff;
        sio->regs[0xf5] = (data >> 16) & 0xff;
        sio->regs[0xf6] = (data >> 8) & 0xff;
        sio->regs[0xf7] = (data >> 0) & 0xff;
    }
    return 0;
}

static uint64_t aspeed_sio_lpc2ahb_read(AspeedSioDevice *s, uint8_t reg)
{
    AspeedSio *sio = s->sio;
    uint64_t val = -1;

    switch (reg) {
    case ASPEED_SIO_REG_ENABLE:
    case 0xf0 ... 0xf3:  /* address */
    case 0xf4 ... 0xf7:  /* data */
    case 0xf8:           /* data size (and more) */
        val = sio->regs[reg];
        break;

    case 0xfe: /* trigger read on AHB bus */
        val = aspeed_sio_lpc2ahb_rw(s, false);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "lpc2ahb: invalid register 0x%x\n", reg);
    }

    return val;
}

static void aspeed_sio_lpc2ahb_write(AspeedSioDevice *s, uint8_t reg,
                                     uint8_t val)
{
    AspeedSio *sio = s->sio;

    switch (reg) {
    case ASPEED_SIO_REG_ENABLE: /* more enablement */
        sio->regs[ASPEED_SIO_REG_ENABLE] = val;
        return;
    case 0xf0 ... 0xf3:  /* address */
    case 0xf4 ... 0xf7:  /* data */
    case 0xf8:           /* data size */
        sio->regs[reg] = val;
        return;

    case 0xfe: /* trigger write on AHB bus */
        if (val == 0xcf) {
            aspeed_sio_lpc2ahb_rw(s, true);
        }
        return;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "lpc2ahb: invalid register 0x%x\n", reg);
    }
}

/*
 * Memory Space layout on the AST2400 and AST2500 SoCs
 *
 *   00000000 - 0FFF:FFFF   FMC Flash Memory (bootup)
 *   1E600000 - 1FFF:FFFF   Controller's registers
 *   20000000 - 2FFF:FFFF   FMC Flash Memory
 *   30000000 - 3FFF:FFFF   SPI Flash Memory
 *   40000000 - 5FFF:FFFF   SDRAM (AST2400)
 *   60000000 - 6FFF:FFFF   AHB BUS to LPC Bus Bridge
 *   70000000 - 7FFF:FFFF   AHB BUS to LPC+ Bus Bridge
 *   80000000 - BFFF:FFFF   SDRAM (AST2500)
 *
 */
static void aspeed_sio_lpc2ahb_realize(DeviceState *dev, Error **errp)
{
    AspeedSioLpc2Ahb *s = ASPEED_SIO_LPC2AHB(dev);

    /*
     * Provide a 2GiB address space, enough to access most of the
     * memory space (only excluding AST2500 SDRAM)
     */
    memory_region_init(&s->ahb_mr, OBJECT(dev), "lpc-ahb", 0x80000000ull);
    address_space_init(&s->ahb_as, &s->ahb_mr, "lpc-ahb");
}

static void aspeed_sio_lpc2ahb_class_init(ObjectClass *klass, void *data)
{
    AspeedSioDeviceClass *sdc = ASPEED_SIO_DEVICE_CLASS(klass);

    sdc->realize = aspeed_sio_lpc2ahb_realize;
    sdc->read    = aspeed_sio_lpc2ahb_read;
    sdc->write   = aspeed_sio_lpc2ahb_write;
    sdc->id      = ASPEED_SIO_DEV_LPC2AHB;
}

static const TypeInfo aspeed_sio_lpc2ahb_info = {
    .name          = TYPE_ASPEED_SIO_LPC2AHB,
    .parent        = TYPE_ASPEED_SIO_DEVICE,
    .instance_size = sizeof(AspeedSioLpc2Ahb),
    .class_init    = aspeed_sio_lpc2ahb_class_init,
};

static void aspeed_sio_register_types(void)
{
    type_register_static(&aspeed_sio_info);
    type_register_static(&aspeed_sio_device_info);
    type_register_static(&aspeed_sio_lpc2ahb_info);
}

type_init(aspeed_sio_register_types)

AspeedSio *aspeed_sio_create(ISABus *isabus)
{
    DeviceState *dev;
    ISADevice *isadev;

    isadev = isa_create(isabus, TYPE_ASPEED_SIO);
    dev = DEVICE(isadev);

    qdev_init_nofail(dev);
    return ASPEED_SIO(isadev);
}
