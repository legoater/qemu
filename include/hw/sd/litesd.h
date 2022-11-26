/*
 * LiteSD Host Controller
 *
 * Copyright (C) 2022 Code Construct
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef LITESD_H
#define LITESD_H

#include "hw/sd/sdhci.h"

struct LiteSDState {
    SysBusDevice parent;
    SDBus sdbus;

    MemoryRegion iomem;
    MemoryRegion iomem_phy;
    MemoryRegion iomem_core;
    MemoryRegion iomem_reader;
    MemoryRegion iomem_writer;
    MemoryRegion iomem_irq;

    MemoryRegion *dma_region;
    AddressSpace dma_as;
    size_t dma_buf_size;
    uint8_t *dma_buf;

    uint32_t phy_regs[2];
    uint32_t core_regs[16];
    uint32_t reader_regs[8];
    uint32_t writer_regs[8];
    uint32_t irq_regs[4];

    qemu_irq irq;
};

#define TYPE_LITESD "litesd"
OBJECT_DECLARE_SIMPLE_TYPE(LiteSDState, LITESD)

#endif /* LITESD_H */
