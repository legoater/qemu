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
#include "qapi/error.h"
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

static Property uhci_sysbus_properties[] = {
    DEFINE_PROP_STRING("masterbus", UHCISysBusState, masterbus),
    DEFINE_PROP_UINT32("firstport", UHCISysBusState, firstport, 0),
    DEFINE_PROP_UINT32("bandwidth", UHCISysBusState, frame_bandwidth, 1280),
    DEFINE_PROP_UINT32("maxframes", UHCISysBusState, maxframes, 128),
    DEFINE_PROP_END_OF_LIST(),
};

static void uhci_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = uhci_sysbus_realize;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    dc->desc = "UHCI USB Controller";
    device_class_set_props(dc, uhci_sysbus_properties);
    device_class_set_legacy_reset(dc, uhci_sysbus_reset_sysbus);
}

static const TypeInfo uhci_sysbus_types[] = {
    {
        .name          = TYPE_SYSBUS_UHCI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(UHCISysBusState),
        .class_init    = uhci_sysbus_class_init,
    },
};

DEFINE_TYPES(uhci_sysbus_types);
