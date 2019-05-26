#ifndef _HW_ASPEED_SIO_H
#define _HW_ASPEED_SIO_H
/*
 * QEMU ASPEED SuperIO Controller
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "hw/isa/isa.h"

/*
 * SuperIO Controller Logical Devices IDs
 */
typedef enum {
    ASPEED_SIO_DEV_NONE    = 0,
    ASPEED_SIO_DEV_UART1   = 2,
    ASPEED_SIO_DEV_UART2   = 3,
    ASPEED_SIO_DEV_SWC     = 4,
    ASPEED_SIO_DEV_KBC     = 5,
    ASPEED_SIO_DEV_P80     = 7,
    ASPEED_SIO_DEV_UART3   = 0xb,
    ASPEED_SIO_DEV_UART4   = 0xc,
    ASPEED_SIO_DEV_LPC2AHB = 0xd,
    ASPEED_SIO_DEV_MBOX    = 0xe,
} AspeedSioDeviceId;

#define ASPEED_SIO_NR_DEV    0x10 /* 4 bits select */

typedef struct AspeedSio AspeedSio;

#define TYPE_ASPEED_SIO_DEVICE  "aspeed.sio.device"
#define ASPEED_SIO_DEVICE(obj) \
    OBJECT_CHECK(AspeedSioDevice, (obj), TYPE_ASPEED_SIO_DEVICE)

typedef struct AspeedSioDevice {
    DeviceState parent_obj;

    AspeedSio *sio;
} AspeedSioDevice;

#define ASPEED_SIO_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(AspeedSioDeviceClass, (klass), TYPE_ASPEED_SIO_DEVICE)
#define ASPEED_SIO_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(AspeedSioDeviceClass, (obj), TYPE_ASPEED_SIO_DEVICE)

typedef struct AspeedSioDeviceClass {
    DeviceClass parent_class;

    AspeedSioDeviceId id;

    void (*realize)(DeviceState *dev, Error **errp);
    bool (*is_enabled)(AspeedSioDevice *s);
    uint64_t (*read)(AspeedSioDevice *s, uint8_t reg);
    void (*write)(AspeedSioDevice *s, uint8_t reg, uint8_t val);

} AspeedSioDeviceClass;

/*
 * SuperIO controller
 */
#define TYPE_ASPEED_SIO  "aspeed.sio"
#define ASPEED_SIO(obj) OBJECT_CHECK(AspeedSio, (obj), TYPE_ASPEED_SIO)

#define ASPEED_SIO_NR_REG  0x100

typedef struct AspeedSio {
    ISADevice parent_obj;

    uint8_t regs[ASPEED_SIO_NR_REG];
    uint8_t reg;
    uint8_t locked;

    uint32_t iobase;
    MemoryRegion io;

    AspeedSioDevice *devices[ASPEED_SIO_NR_DEV];
} AspeedSio;

/*
 * SuperIO common registers
 */
#define ASPEED_SIO_REG_SELECT  0x07
#define ASPEED_SIO_REG_ENABLE  0x30


extern AspeedSio *aspeed_sio_create(ISABus *bus);

#endif /* _HW_ASPEED_SIO_H */
