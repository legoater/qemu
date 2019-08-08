/*
 * Aspeed SD Host Controller
 * Eddie James <eajames@linux.ibm.com>
 *
 * Copyright (C) 2019 IBM Corp
 * SPDX-License-Identifer: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/sd/aspeed_sdhci.h"
#include "qapi/error.h"

#define ASPEED_SDHCI_INFO            0x00
#define  ASPEED_SDHCI_INFO_RESET     0x00030000
#define ASPEED_SDHCI_DEBOUNCE        0x04
#define  ASPEED_SDHCI_DEBOUNCE_RESET 0x00000005
#define ASPEED_SDHCI_BUS             0x08
#define ASPEED_SDHCI_SDIO_140        0x10
#define ASPEED_SDHCI_SDIO_148        0x18
#define ASPEED_SDHCI_SDIO_240        0x20
#define ASPEED_SDHCI_SDIO_248        0x28
#define ASPEED_SDHCI_WP_POL          0xec
#define ASPEED_SDHCI_CARD_DET        0xf0
#define ASPEED_SDHCI_IRQ_STAT        0xfc

#define TO_REG(addr) ((addr) / sizeof(uint32_t))

static uint64_t aspeed_sdhci_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint32_t val = 0;
    AspeedSDHCIState *sdhci = opaque;

    switch (addr) {
    case ASPEED_SDHCI_SDIO_140:
        val = (uint32_t)sdhci->slots[0].sdhci.capareg;
        break;
    case ASPEED_SDHCI_SDIO_148:
        val = (uint32_t)sdhci->slots[0].sdhci.maxcurr;
        break;
    case ASPEED_SDHCI_SDIO_240:
        val = (uint32_t)sdhci->slots[1].sdhci.capareg;
        break;
    case ASPEED_SDHCI_SDIO_248:
        val = (uint32_t)sdhci->slots[1].sdhci.maxcurr;
        break;
    default:
        if (addr < ASPEED_SDHCI_REG_SIZE) {
            val = sdhci->regs[TO_REG(addr)];
        }
    }

    return (uint64_t)val;
}

static void aspeed_sdhci_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned int size)
{
    AspeedSDHCIState *sdhci = opaque;

    switch (addr) {
    case ASPEED_SDHCI_SDIO_140:
        sdhci->slots[0].sdhci.capareg = (uint64_t)(uint32_t)val;
        break;
    case ASPEED_SDHCI_SDIO_148:
        sdhci->slots[0].sdhci.maxcurr = (uint64_t)(uint32_t)val;
        break;
    case ASPEED_SDHCI_SDIO_240:
        sdhci->slots[1].sdhci.capareg = (uint64_t)(uint32_t)val;
        break;
    case ASPEED_SDHCI_SDIO_248:
        sdhci->slots[1].sdhci.maxcurr = (uint64_t)(uint32_t)val;
        break;
    default:
        if (addr < ASPEED_SDHCI_REG_SIZE) {
            sdhci->regs[TO_REG(addr)] = (uint32_t)val;
        }
    }
}

static const MemoryRegionOps aspeed_sdhci_ops = {
    .read = aspeed_sdhci_read,
    .write = aspeed_sdhci_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aspeed_sdhci_irq_notify(SDHCIState *s, int level)
{
    AspeedSDHCISlotState *as = container_of(s, AspeedSDHCISlotState, sdhci);
    AspeedSDHCIState *sdhci = container_of(as, AspeedSDHCIState,
                                           slots[as->slot]);

    if (level) {
        sdhci->regs[TO_REG(ASPEED_SDHCI_IRQ_STAT)] |= BIT(as->slot);

        qemu_irq_raise(sdhci->irq);
    } else {
        sdhci->regs[TO_REG(ASPEED_SDHCI_IRQ_STAT)] &= ~BIT(as->slot);

        qemu_irq_lower(sdhci->irq);
    }
}

static void aspeed_sdhci_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    AspeedSDHCIState *sdhci = ASPEED_SDHCI(dev);

    sysbus_init_irq(sbd, &sdhci->irq);
    memory_region_init_io(&sdhci->iomem, OBJECT(sdhci), &aspeed_sdhci_ops,
                          sdhci, TYPE_ASPEED_SDHCI, ASPEED_SDHCI_REG_SIZE);
    sysbus_init_mmio(sbd, &sdhci->iomem);

    sdhci->slots[0].slot = 0;
    sdhci->slots[0].sdhci.irq_notify = aspeed_sdhci_irq_notify;

    sdhci->slots[1].slot = 1;
    sdhci->slots[1].sdhci.irq_notify = aspeed_sdhci_irq_notify;
}

static void aspeed_sdhci_reset(DeviceState *dev)
{
    AspeedSDHCIState *sdhci = ASPEED_SDHCI(dev);

    memset(sdhci->regs, 0, ASPEED_SDHCI_REG_SIZE);
    sdhci->regs[TO_REG(ASPEED_SDHCI_INFO)] = ASPEED_SDHCI_INFO_RESET;
    sdhci->regs[TO_REG(ASPEED_SDHCI_DEBOUNCE)] = ASPEED_SDHCI_DEBOUNCE_RESET;
}

static const VMStateDescription vmstate_aspeed_sdhci = {
    .name = TYPE_ASPEED_SDHCI,
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedSDHCIState, ASPEED_SDHCI_NUM_REGS),
        VMSTATE_END_OF_LIST(),
    },
};

static void aspeed_sdhci_class_init(ObjectClass *classp, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(classp);

    dc->realize = aspeed_sdhci_realize;
    dc->reset = aspeed_sdhci_reset;
    dc->vmsd = &vmstate_aspeed_sdhci;
}

static TypeInfo aspeed_sdhci_info = {
    .name          = TYPE_ASPEED_SDHCI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedSDHCIState),
    .class_init    = aspeed_sdhci_class_init,
};

static void aspeed_sdhci_register_types(void)
{
    type_register_static(&aspeed_sdhci_info);
}

type_init(aspeed_sdhci_register_types)
