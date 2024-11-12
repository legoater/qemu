/*
 * USB UHCI controller emulation
 * PCI code
 *
 * Copyright (c) 2005 Fabrice Bellard
 *
 * Copyright (c) 2008 Max Krasnyansky
 *     Magor rewrite of the UHCI data structures parser and frame processor
 *     Support for fully async operation and multiple outstanding transactions
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/usb.h"
#include "migration/vmstate.h"
#include "hw/pci/pci.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hcd-uhci-pci.h"

struct UHCIPCIDeviceClass {
    PCIDeviceClass parent_class;
    UHCIPCIInfo info;
};

static const VMStateDescription vmstate_uhci = {
    .name = "pci_uhci",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, UHCIPCIState),
        VMSTATE_STRUCT(state, UHCIPCIState, 1, vmstate_uhci_state, UHCIState),
        VMSTATE_END_OF_LIST()
    }
};

static void uhci_pci_reset(UHCIState *uhci)
{
    UHCIPCIState *pstate = container_of(uhci, UHCIPCIState, state);
    PCIDevice *d = &pstate->dev;

    d->config[0x6a] = 0x01; /* usb clock */
    d->config[0x6b] = 0x00;

    uhci_state_reset(uhci);
}

void usb_uhci_common_realize_pci(PCIDevice *dev, Error **errp)
{
    Error *err = NULL;
    UHCIPCIDeviceClass *u = UHCI_PCI_GET_CLASS(dev);
    UHCIPCIState *uhci = UHCI_PCI(dev);
    UHCIState *s = &uhci->state;
    uint8_t *pci_conf = dev->config;

    pci_conf[PCI_CLASS_PROG] = 0x00;
    /* TODO: reset value should be 0. */
    pci_conf[USB_SBRN] = USB_RELEASE_1; /* release number */
    pci_config_set_interrupt_pin(pci_conf, u->info.irq_pin + 1);

    s->irq = pci_allocate_irq(dev);
    s->masterbus = uhci->masterbus;
    s->firstport = uhci->firstport;
    s->maxframes = uhci->maxframes;
    s->frame_bandwidth = uhci->frame_bandwidth;
    s->as = pci_get_address_space(dev);
    s->uhci_reset = uhci_pci_reset;

    usb_uhci_init(s, DEVICE(dev), &err);

    /*
     * Use region 4 for consistency with real hardware.  BSD guests seem
     * to rely on this.
     */
    pci_register_bar(dev, 4, PCI_BASE_ADDRESS_SPACE_IO, &s->mem);
}

static void uhci_pci_reset_pci(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    UHCIPCIState *uhci = UHCI_PCI(d);

    uhci_pci_reset(&uhci->state);
}

static void usb_uhci_pci_exit(PCIDevice *dev)
{
    UHCIPCIState *uhci = UHCI_PCI(dev);
    UHCIState *s = &uhci->state;

    usb_uhci_exit(s);

    qemu_free_irq(s->irq);
}

static Property uhci_properties_companion[] = {
    DEFINE_PROP_STRING("masterbus", UHCIPCIState, masterbus),
    DEFINE_PROP_UINT32("firstport", UHCIPCIState, firstport, 0),
    DEFINE_PROP_UINT32("bandwidth", UHCIPCIState, frame_bandwidth, 1280),
    DEFINE_PROP_UINT32("maxframes", UHCIPCIState, maxframes, 128),
    DEFINE_PROP_END_OF_LIST(),
};
static Property uhci_properties_standalone[] = {
    DEFINE_PROP_UINT32("bandwidth", UHCIPCIState, frame_bandwidth, 1280),
    DEFINE_PROP_UINT32("maxframes", UHCIPCIState, maxframes, 128),
    DEFINE_PROP_END_OF_LIST(),
};

static void uhci_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->class_id  = PCI_CLASS_SERIAL_USB;
    dc->vmsd = &vmstate_uhci;
    device_class_set_legacy_reset(dc, uhci_pci_reset_pci);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo uhci_pci_type_info = {
    .name = TYPE_UHCI_PCI,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(UHCIPCIState),
    .class_size    = sizeof(UHCIPCIDeviceClass),
    .class_init    = uhci_pci_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

void uhci_pci_data_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    UHCIPCIDeviceClass *u = UHCI_PCI_CLASS(klass);
    UHCIPCIInfo *info = data;

    k->realize = info->realize ? info->realize : usb_uhci_common_realize_pci;
    k->exit = info->unplug ? usb_uhci_pci_exit : NULL;
    k->vendor_id = info->vendor_id;
    k->device_id = info->device_id;
    k->revision  = info->revision;
    if (!info->unplug) {
        /* uhci controllers in companion setups can't be hotplugged */
        dc->hotpluggable = false;
        device_class_set_props(dc, uhci_properties_companion);
    } else {
        device_class_set_props(dc, uhci_properties_standalone);
    }
    if (info->notuser) {
        dc->user_creatable = false;
    }
    u->info = *info;
}

static UHCIPCIInfo uhci_pci_info[] = {
    {
        .name      = TYPE_PIIX3_USB_UHCI,
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82371SB_2,
        .revision  = 0x01,
        .irq_pin   = 3,
        .unplug    = true,
    },{
        .name      = TYPE_PIIX4_USB_UHCI,
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82371AB_2,
        .revision  = 0x01,
        .irq_pin   = 3,
        .unplug    = true,
    },{
        .name      = TYPE_ICH9_USB_UHCI(1), /* 00:1d.0 */
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801I_UHCI1,
        .revision  = 0x03,
        .irq_pin   = 0,
        .unplug    = false,
    },{
        .name      = TYPE_ICH9_USB_UHCI(2), /* 00:1d.1 */
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801I_UHCI2,
        .revision  = 0x03,
        .irq_pin   = 1,
        .unplug    = false,
    },{
        .name      = TYPE_ICH9_USB_UHCI(3), /* 00:1d.2 */
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801I_UHCI3,
        .revision  = 0x03,
        .irq_pin   = 2,
        .unplug    = false,
    },{
        .name      = TYPE_ICH9_USB_UHCI(4), /* 00:1a.0 */
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801I_UHCI4,
        .revision  = 0x03,
        .irq_pin   = 0,
        .unplug    = false,
    },{
        .name      = TYPE_ICH9_USB_UHCI(5), /* 00:1a.1 */
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801I_UHCI5,
        .revision  = 0x03,
        .irq_pin   = 1,
        .unplug    = false,
    },{
        .name      = TYPE_ICH9_USB_UHCI(6), /* 00:1a.2 */
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801I_UHCI6,
        .revision  = 0x03,
        .irq_pin   = 2,
        .unplug    = false,
    }
};

static void uhci_pci_register_types(void)
{
    TypeInfo type_info = {
        .parent        = TYPE_UHCI_PCI,
        .class_init    = uhci_pci_data_class_init,
    };
    int i;

    type_register_static(&uhci_pci_type_info);

    for (i = 0; i < ARRAY_SIZE(uhci_pci_info); i++) {
        type_info.name = uhci_pci_info[i].name;
        type_info.class_data = uhci_pci_info + i;
        type_register(&type_info);
    }
}

type_init(uhci_pci_register_types)
