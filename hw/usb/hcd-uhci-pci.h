/*
 * USB UHCI controller emulation
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
#ifndef HW_USB_HCD_UHCI_PCI_H
#define HW_USB_HCD_UHCI_PCI_H

#include "hcd-uhci.h"

#define TYPE_UHCI_PCI "pci-uhci"

struct UHCIPCIState {
    PCIDevice dev;
    UHCIState state;

    /* Properties */
    char *masterbus;
    uint32_t firstport;
    uint32_t frame_bandwidth;
    uint32_t maxframes;
    uint32_t num_ports;
};

OBJECT_DECLARE_TYPE(UHCIPCIState, UHCIPCIDeviceClass, UHCI_PCI)

typedef struct UHCIPCIInfo {
    const char *name;
    uint16_t   vendor_id;
    uint16_t   device_id;
    uint8_t    revision;
    uint8_t    irq_pin;
    void       (*realize)(PCIDevice *dev, Error **errp);
    bool       unplug;
    bool       notuser; /* disallow user_creatable */
} UHCIPCIInfo;

void usb_uhci_common_realize_pci(PCIDevice *dev, Error **errp);
void uhci_pci_data_class_init(ObjectClass *klass, void *data);

#endif /* HW_USB_HCD_UHCI_PCI_H */
