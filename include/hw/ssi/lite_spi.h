/*
 * QEMU PowerPC Lite SPI model
 *
 * Copyright (c) 2021, IBM Corporation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SPI_LITESPI_H
#define HW_SPI_LITESPI_H

#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_LITESPI "litespi"
OBJECT_DECLARE_SIMPLE_TYPE(LiteSPIState, LITESPI)

struct LiteSPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_flash;

    qemu_irq cs_lines[1];

    SSIBus *spi;

    uint32_t regs[0x100];
};

#endif /* LITESPI_H */
