#ifndef HW_USB_HCD_UHCI_SYSBUS_H
#define HW_USB_HCD_UHCI_SYSBUS_H

#include "hcd-uhci.h"

#define TYPE_SYSBUS_UHCI "sysbus-uhci"

OBJECT_DECLARE_SIMPLE_TYPE(UHCISysBusState, SYSBUS_UHCI)

struct UHCISysBusState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    UHCIState uhci;

    char *masterbus;
    uint32_t firstport;
    uint32_t frame_bandwidth;
    uint32_t maxframes;
    uint32_t num_ports;
};

#endif /* HW_USB_HCD_UHCI_SYSBUS_H */
