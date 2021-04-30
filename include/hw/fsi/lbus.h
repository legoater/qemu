/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2019 IBM Corp.
 *
 * IBM Common FRU Access Macro
 */
#ifndef FSI_LBUS_H
#define FSI_LBUS_H

#include "exec/memory.h"
#include "hw/qdev-core.h"

#include "hw/fsi/bits.h"

#define ENGINE_CONFIG_NEXT              BE_BIT(0)
#define ENGINE_CONFIG_VPD               BE_BIT(1)
#define ENGINE_CONFIG_SLOTS             BE_GENMASK(8, 15)
#define ENGINE_CONFIG_VERSION           BE_GENMASK(16, 19)
#define ENGINE_CONFIG_TYPE              BE_GENMASK(20, 27)
#define   ENGINE_CONFIG_TYPE_PEEK       (0x02 << 4)
#define   ENGINE_CONFIG_TYPE_FSI        (0x03 << 4)
#define   ENGINE_CONFIG_TYPE_SCRATCHPAD (0x06 << 4)
#define ENGINE_CONFIG_CRC              BE_GENMASK(28, 31)

#define TYPE_CFAM_CONFIG "cfam.config"
#define CFAM_CONFIG(obj) \
    OBJECT_CHECK(CFAMConfig, (obj), TYPE_CFAM_CONFIG)
/* P9-ism */
#define CFAM_CONFIG_NR_REGS 0x28

typedef struct CFAMState CFAMState;

/* TODO: Generalise this accommodate different CFAM configurations */
typedef struct CFAMConfig {
    DeviceState parent;

    MemoryRegion iomem;
} CFAMConfig;

#define TYPE_CFAM_PEEK "cfam.peek"
#define CFAM_PEEK(obj) \
    OBJECT_CHECK(CFAMPeek, (obj), TYPE_CFAM_PEEK)
#define CFAM_PEEK_NR_REGS ((0x130 >> 2) + 1)

typedef struct CFAMPeek {
    DeviceState parent;

    MemoryRegion iomem;
} CFAMPeek;

#define TYPE_LBUS_DEVICE "lbus.device"
#define LBUS_DEVICE(obj) \
    OBJECT_CHECK(LBusDevice, (obj), TYPE_LBUS_DEVICE)
#define LBUS_DEVICE_CLASS(klass) \
    OBJECT_CLASS_CHECK(LBusDeviceClass, (klass), TYPE_LBUS_DEVICE)
#define LBUS_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(LBusDeviceClass, (obj), TYPE_LBUS_DEVICE)

typedef struct LBusDevice {
    DeviceState parent;

    MemoryRegion iomem;
    uint32_t address;
} LBusDevice;

typedef struct LBusDeviceClass {
    DeviceClass parent;

    uint32_t config;
} LBusDeviceClass;

typedef struct LBusNode {
    LBusDevice *ldev;

    QLIST_ENTRY(LBusNode) next;
} LBusNode;

#define TYPE_LBUS "lbus"
#define LBUS(obj) OBJECT_CHECK(LBus, (obj), TYPE_LBUS)
#define LBUS_CLASS(klass) \
    OBJECT_CLASS_CHECK(LBusClass, (klass), TYPE_LBUS)
#define LBUS_GET_CLASS(obj) \
    OBJECT_GET_CLASS(LBusClass, (obj), TYPE_LBUS)

typedef struct LBus {
    BusState bus;

    MemoryRegion mr;

    QLIST_HEAD(, LBusNode) devices;
} LBus;

DeviceState *lbus_create_device(LBus *bus, const char *type, uint32_t addr);
int lbus_add_device(LBus *bus, LBusDevice *dev);
#endif /* FSI_LBUS_H */
