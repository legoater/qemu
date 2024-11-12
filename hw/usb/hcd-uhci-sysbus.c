/*
 * QEMU USB UHCI Emulation
 * Copyright (c) 2006 Openedhand Ltd.
 * Copyright (c) 2010 CodeSourcery
 * Copyright (c) 2024 Red Hat, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/usb/uhci-regs.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/usb.h"
#include "migration/vmstate.h"
#include "hw/sysbus.h"
#include "hw/qdev-dma.h"
#include "hw/qdev-properties.h"
#include "trace.h"
#include "hcd-uhci.h"
#include "hcd-uhci-sysbus.h"

static void uhci_sysbus_reset(UHCIState *uhci)
{
    uhci_state_reset(uhci);
}

static void uhci_sysbus_realize(DeviceState *dev, Error **errp)
{
    UHCISysBusState *s = SYSBUS_UHCI(dev);
    UHCIState *uhci = &s->uhci;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Error *err = NULL;

    uhci->masterbus = s->masterbus;
    uhci->firstport = s->firstport;
    uhci->maxframes = s->maxframes;
    uhci->frame_bandwidth = s->frame_bandwidth;
    uhci->as = &address_space_memory;
    uhci->uhci_reset = uhci_sysbus_reset;

    usb_uhci_init(uhci, dev, &err);

    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_init_irq(sbd, &uhci->irq);
    sysbus_init_mmio(sbd, &uhci->mem);
}

static void uhci_sysbus_reset_sysbus(DeviceState *dev)
{
    UHCISysBusState *s = SYSBUS_UHCI(dev);
    UHCIState *uhci = &s->uhci;

    uhci_sysbus_reset(uhci);
}

static const Property uhci_sysbus_properties[] = {
    DEFINE_PROP_STRING("masterbus", UHCISysBusState, masterbus),
    DEFINE_PROP_UINT32("firstport", UHCISysBusState, firstport, 0),
    DEFINE_PROP_UINT32("bandwidth", UHCISysBusState, frame_bandwidth, 1280),
    DEFINE_PROP_UINT32("maxframes", UHCISysBusState, maxframes, 128),
};

static void uhci_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = uhci_sysbus_realize;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    dc->desc = "UHCI USB Controller";
    device_class_set_legacy_reset(dc, uhci_sysbus_reset_sysbus);
}

static hwaddr aspeed_uhci_chip_to_uhci(hwaddr addr)
{
    switch (addr) {
    case 0x00:
        return UHCI_USBCMD;
    case 0x04:
        return UHCI_USBSTS;
    case 0x08:
        return UHCI_USBINTR;
    case 0x0c:
        return UHCI_USBFLBASEADD;
    case 0x80:
        return UHCI_USBFRNUM;
    case 0x84:
        return UHCI_USBSOF;
    case 0x88:
        return UHCI_USBPORTSC1;
    case 0x8c:
        return UHCI_USBPORTSC2;
    case 0x90:
        return UHCI_USBPORTSC3;
    case 0x94:
        return UHCI_USBPORTSC4;
    default:        /* unimplemented */
        qemu_log_mask(LOG_UNIMP, "Unimplemented Aspeed UHCI register 0x%"
                      HWADDR_PRIx "\n", addr);
        return 0x20;
    }
}

/*
 * Aspeed UHCI registers are 32 bit wide.
 * Convert to 16 bit to access standard UHCI code.
 */
static uint64_t aspeed_uhci_port_read(void *opaque, hwaddr addr, unsigned size)
{
    UHCIState *uhci = opaque;
    MemoryRegion *mr = &uhci->mem;
    hwaddr uaddr = aspeed_uhci_chip_to_uhci(addr);

    if (uaddr == UHCI_USBFLBASEADD) {
        return mr->ops->read(opaque, uaddr, 2) |
               mr->ops->read(opaque, uaddr + 2, 2) << 16;
    }
    return mr->ops->read(opaque, uaddr, 2);
}

static void aspeed_uhci_port_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    UHCIState *uhci = opaque;
    MemoryRegion *mr = &uhci->mem;
    hwaddr uaddr = aspeed_uhci_chip_to_uhci(addr);

    if (uaddr == UHCI_USBFLBASEADD) {
        mr->ops->write(opaque, uaddr, val & 0xffff, 2);
        mr->ops->write(opaque, uaddr + 2, val >> 16, 2);
    } else {
        mr->ops->write(opaque, uaddr, val, 2);
    }
}

static const MemoryRegionOps aspeed_uhci_mmio_ops = {
    .read = aspeed_uhci_port_read,
    .write = aspeed_uhci_port_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void uhci_sysbus_aspeed_realize(DeviceState *dev, Error **errp)
{
    UHCISysBusState *s = SYSBUS_UHCI(dev);
    ASPEEDUHCIState *f = ASPEED_UHCI(dev);
    UHCIState *uhci = &s->uhci;

    uhci_sysbus_realize(dev, errp);

    memory_region_init_io(&f->mem_aspeed, OBJECT(f), &aspeed_uhci_mmio_ops,
                          uhci, "aspeed", 0x100);
    memory_region_add_subregion(&uhci->mem, 0, &f->mem_aspeed);
}

static void uhci_sysbus_aspeed_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = uhci_sysbus_aspeed_realize;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    dc->desc = "ASPEED UHCI USB Controller";
    device_class_set_legacy_reset(dc, uhci_sysbus_reset_sysbus);
    device_class_set_props(dc, uhci_sysbus_properties);
    dc->user_creatable = false;
}

static const TypeInfo uhci_sysbus_types[] = {
    {
        .name          = TYPE_SYSBUS_UHCI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(UHCISysBusState),
        .class_init    = uhci_sysbus_class_init,
    },
    {
        .name          = TYPE_ASPEED_UHCI,
        .parent        = TYPE_SYSBUS_UHCI,
        .instance_size = sizeof(ASPEEDUHCIState),
        .class_init    = uhci_sysbus_aspeed_class_init,
    },
};

DEFINE_TYPES(uhci_sysbus_types);
