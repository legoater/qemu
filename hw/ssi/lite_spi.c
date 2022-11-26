/*
 * QEMU PowerPC Lite SPI model
 *
 * Copyright (c) 2021, IBM Corporation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "sysemu/reset.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/units.h"

#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/ssi/lite_spi.h"

#include "trace.h"

#define LSPI_DATA               0x00
#define LSPI_DATA_DUAL          0x01
#define LSPI_DATA_QUAD          0x02
#define LSPI_CTRL               0x04
#define   LSPI_CTRL_RESET         0x01  /* reset all registers */
#define   LSPI_CTRL_MANUAL_CS     0x02  /* assert CS, enable manual mode */
#define LSPI_CFG                0x08    /* Automatic map configuration */
FIELD(REG_CFG, CMD,      0, 7)
FIELD(REG_CFG, DUMMIES,  8, 3)
FIELD(REG_CFG, MODE,    11, 2)
#define LSPI_CFG_MODE_SINGLE      0x0
#define LSPI_CFG_MODE_DUAL        0x2
#define LSPI_CFG_MODE_QUAD        0x3
FIELD(REG_CFG, ADDR4,   13, 1)
FIELD(REG_CFG, CLK_DIV, 16, 8)
FIELD(REG_CFG, CSTOUT,  24, 6)

#define LSPI_REG(reg) ((reg) >> 2)

static void lite_spi_flash_select(LiteSPIState *s, bool select)
{
    qemu_set_irq(s->cs_lines[0], !select);
}

static void lite_spi_flash_setup(LiteSPIState *s, uint32_t addr)
{
    uint32_t cfg = s->regs[LSPI_REG(LSPI_CFG)];
    uint8_t cmd = FIELD_EX32(cfg, REG_CFG, CMD);
    int i = FIELD_EX32(cfg, REG_CFG, ADDR4) ? 4 : 3;
    uint8_t dummies = FIELD_EX32(cfg, REG_CFG, DUMMIES) ? 8 : 0;

    trace_lite_spi_flash_setup(addr, i, cmd, dummies);

    ssi_transfer(s->spi, cmd);
    while (i--) {
        ssi_transfer(s->spi, (addr >> (i * 8)) & 0xff);
    }

    for (i = 0; i < dummies; i++) {
        ssi_transfer(s->spi, 0xff);
    }
}

static uint64_t lite_spi_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    LiteSPIState *s = LITESPI(opaque);
    uint64_t ret = 0;
    int i;

    lite_spi_flash_select(s, true);
    lite_spi_flash_setup(s, addr);

    for (i = 0; i < size; i++) {
        ret |= ssi_transfer(s->spi, 0x0) << (8 * i);
    }

    lite_spi_flash_select(s, false);

    trace_lite_spi_flash_read(addr, size, ret);
    return ret;
}

static void lite_spi_flash_write(void *opaque, hwaddr addr, uint64_t data,
                                 unsigned size)
{
    LiteSPIState *s = LITESPI(opaque);
    int i;

    trace_lite_spi_flash_write(addr, size, data);

    lite_spi_flash_select(s, true);
    lite_spi_flash_setup(s, addr);

    for (i = 0; i < size; i++) {
        ssi_transfer(s->spi, (data >> (8 * i)) & 0xff);
    }

    lite_spi_flash_select(s, false);
}

static const MemoryRegionOps lite_spi_flash_ops = {
    .read = lite_spi_flash_read,
    .write = lite_spi_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static uint64_t lite_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    LiteSPIState *s = LITESPI(opaque);
    uint64_t val = -1;

    switch (addr) {
    case LSPI_DATA_QUAD: /* TODO dummies */
    case LSPI_DATA_DUAL: /* TODO dummies */
    case LSPI_DATA:
        val = ssi_transfer(s->spi, 0x0);
        break;
    case LSPI_CTRL:
    case LSPI_CFG:
        val = s->regs[LSPI_REG(addr)];
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_lite_spi_read(addr, size, val);
    return val;
}

static void lite_spi_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned size)
{
     LiteSPIState *s = LITESPI(opaque);

     switch (addr) {
     case LSPI_DATA:
         ssi_transfer(s->spi, data & 0xff);
         break;
     case LSPI_CTRL:
         if (data & LSPI_CTRL_RESET) {
             device_cold_reset(DEVICE(s));
         } else {
             lite_spi_flash_select(s, !!(data & LSPI_CTRL_MANUAL_CS));
         }
         break;
     case LSPI_CFG:
         s->regs[LSPI_REG(addr)] = data;
         break;

     default:
         qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                       __func__, addr);
         break;
     }

     trace_lite_spi_write(addr, size, data);
}

static const MemoryRegionOps lite_spi_ops = {
    .read = lite_spi_read,
    .write = lite_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void lite_spi_reset(DeviceState *dev)
{
    LiteSPIState *s = LITESPI(dev);

    memset(s->regs, 0, sizeof s->regs);

    /* set automatic config with normal reads */
    s->regs[LSPI_REG(LSPI_CFG)] =
        FIELD_DP32(0, REG_CFG, CMD,     0x3) |
        FIELD_DP32(0, REG_CFG, DUMMIES, 0x0) |
        FIELD_DP32(0, REG_CFG, MODE,    LSPI_CFG_MODE_SINGLE) |
        FIELD_DP32(0, REG_CFG, CLK_DIV, 1); /* SCK = CLK / ((CLK_DIV + 1) * 2) */
}

static void lite_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    LiteSPIState *s = LITESPI(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(s->cs_lines); ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &lite_spi_ops, s,
                          TYPE_LITESPI, sizeof(s->regs) * 4);
    sysbus_init_mmio(sbd, &s->mmio);

    memory_region_init_io(&s->mmio_flash, OBJECT(s), &lite_spi_flash_ops,
                          s, TYPE_LITESPI "-flash", 16 * MiB /* TODO */ );
    sysbus_init_mmio(sbd, &s->mmio_flash);

    s->spi = ssi_create_bus(dev, "spi");
}

static Property lite_spi_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void lite_spi_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = lite_spi_realize;
    dc->reset = lite_spi_reset;
    device_class_set_props(dc, lite_spi_properties);
    dc->user_creatable = false;
}

static const TypeInfo lite_spi_infos[] = {
    {
        .name           = TYPE_LITESPI,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(LiteSPIState),
        .class_init     = lite_spi_class_init,
    },
};

DEFINE_TYPES(lite_spi_infos)
