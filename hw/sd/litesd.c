/*
 * LiteSD Host Controller
 *
 * Copyright (C) 2022 Code Construct
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sd/litesd.h"
#include "hw/qdev-properties.h"
#include "trace.h"

#define LITESD_PHY_CARDDETECT   0
#define LITESD_PHY_CARDDETECT_ABSENT (1 << 0)
#define LITESD_PHY_CLOCKERDIV   1
#define LITESD_PHY_INITIALIZE   2

#define LITESD_CORE_CMDARG      0
#define LITESD_CORE_CMDCMD      1
#define LITESD_CORE_CMDCMD_XFER_READ    (1 << 0)
#define LITESD_CORE_CMDCMD_XFER_WRITE   (1 << 1)
#define LITESD_CORE_CMDCMD_RLEN_NONE       0
#define LITESD_CORE_CMDCMD_RLEN_SHORT      1
#define LITESD_CORE_CMDCMD_RLEN_LONG       2
#define LITESD_CORE_CMDCMD_RLEN_SHORT_BUSY 3
#define LITESD_CORE_CMDSND      2
#define LITESD_CORE_CMDRSP0     3
#define LITESD_CORE_CMDRSP1     4
#define LITESD_CORE_CMDRSP2     5
#define LITESD_CORE_CMDRSP3     6
#define LITESD_CORE_CMDEVT      7
#define LITESD_CORE_CMDEVT_DONE         (1 << 0)
#define LITESD_CORE_CMDEVT_WR_ERR       (1 << 1)
#define LITESD_CORE_DATAEVT     8
#define LITESD_CORE_BLKLEN      9
#define LITESD_CORE_BLKCNT      10

#define LITESD_READER_BASE0     0
#define LITESD_READER_BASE1     1
#define LITESD_READER_LEN       2
#define LITESD_READER_ENA       3
#define LITESD_READER_DONE      4

#define LITESD_WRITER_BASE0     0
#define LITESD_WRITER_BASE1     1
#define LITESD_WRITER_LEN       2
#define LITESD_WRITER_ENA       3
#define LITESD_WRITER_DONE      4

#define LITESD_IRQ_STATUS       0
#define LITESD_IRQ_PENDING      1
#define LITESD_IRQ_ENABLE       2
#define LITESD_IRQ_IRQ_CARD_DETECT      (1 << 0)
#define LITESD_IRQ_IRQ_SD_TO_MEM_DONE   (1 << 1)
#define LITESD_IRQ_IRQ_MEM_TO_SD_DONE   (1 << 2)
#define LITESD_IRQ_IRQ_CMD_DONE         (1 << 3)

#define TYPE_LITESD_BUS "litesd-bus"
DECLARE_INSTANCE_CHECKER(SDBus, LITESD_BUS, TYPE_LITESD_BUS)

static void litesd_irq_update(LiteSDState *s)
{
    uint32_t pending, enabled;

    pending = s->irq_regs[LITESD_IRQ_PENDING];
    enabled = s->irq_regs[LITESD_IRQ_ENABLE];

    qemu_set_irq(s->irq, !!(pending & enabled));
}

static void litesd_reader_dma(LiteSDState *s)
{
    size_t blklen, len;
    dma_addr_t addr;
    MemTxResult tx;
    int i, blkcnt;

    if (!s->reader_regs[LITESD_READER_ENA]) {
        return;
    }

    addr = (uint64_t)s->reader_regs[LITESD_READER_BASE0] << 32
        | (uint64_t)s->reader_regs[LITESD_READER_BASE1];
    len = s->reader_regs[LITESD_READER_LEN];
    blklen = s->core_regs[LITESD_CORE_BLKLEN];
    blkcnt = s->core_regs[LITESD_CORE_BLKCNT];

    if (blklen > s->dma_buf_size) {
        if (blklen > 16 * MiB) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: DMA block size 0x%zx is too large!",
                          __func__, blklen);
            return;
        }
        s->dma_buf_size = blklen;
        s->dma_buf = g_realloc(s->dma_buf, s->dma_buf_size);
    }

    trace_litesd_dma_from_sd(addr, len, blklen, blkcnt);

    for (i = 0; i < blkcnt; i++) {
        size_t this_blklen = MIN(blklen, len - (i * blklen));
        sdbus_read_data(&s->sdbus, s->dma_buf, this_blklen);
        tx = dma_memory_write(&s->dma_as, addr + i * blklen, s->dma_buf,
                              this_blklen, MEMTXATTRS_UNSPECIFIED);
        if (tx != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "Error DMAing to memory "
                          "(0x%" PRIx64 " len 0x%" PRIx64 ")",
                          addr + i * blklen, this_blklen);
            return;
        }
     }

    s->reader_regs[LITESD_READER_DONE] |= 1;
    s->core_regs[LITESD_CORE_DATAEVT] |= 1;
}

static void litesd_writer_dma(LiteSDState *s)
{
    size_t blklen, len;
    dma_addr_t addr;
    MemTxResult tx;
    int i, blkcnt;

    if (!s->writer_regs[LITESD_WRITER_ENA]) {
        return;
    }

    addr = (uint64_t)s->writer_regs[LITESD_WRITER_BASE0] << 32
        | (uint64_t)s->writer_regs[LITESD_WRITER_BASE1];
    len = s->writer_regs[LITESD_WRITER_LEN];
    blklen = s->core_regs[LITESD_CORE_BLKLEN];
    blkcnt = s->core_regs[LITESD_CORE_BLKCNT];

    if (blklen > s->dma_buf_size) {
        if (blklen > 16 * MiB) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: DMA block size 0x%zx is too large!",
                          __func__, blklen);
            return;
        }
        s->dma_buf_size = blklen;
        s->dma_buf = g_realloc(s->dma_buf, s->dma_buf_size);
    }

    trace_litesd_dma_to_sd(addr, len, blklen, blkcnt);

    for (i = 0; i < blkcnt; i++) {
        size_t this_blklen = MIN(blklen, len - (i * blklen));
        tx = dma_memory_read(&s->dma_as, addr + i * blklen, s->dma_buf,
                             this_blklen, MEMTXATTRS_UNSPECIFIED);
        if (tx != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "Error DMAing from memory "
                          "(0x%" PRIx64 " len 0x%" PRIx64 ")",
                          addr + i * blklen, this_blklen);
            return;
        }
        sdbus_write_data(&s->sdbus, s->dma_buf, this_blklen);
     }

    s->writer_regs[LITESD_WRITER_DONE] |= 1;
    s->core_regs[LITESD_CORE_DATAEVT] |= 1;
}

static void litesd_do_cmd(LiteSDState *s)
{
    uint8_t rlen, xfer, resp[16];
    SDRequest req;
    uint32_t cmd;
    int rc;

    cmd = s->core_regs[LITESD_CORE_CMDCMD];
    xfer = cmd >> 5 & 0x3;
    rlen = cmd & 0x3;
    req.cmd = (cmd >> 8) & 0xff;
    req.arg = s->core_regs[LITESD_CORE_CMDARG];

    trace_litesd_command(cmd, req.arg, xfer, rlen);

    rc = sdbus_do_command(&s->sdbus, &req, resp);
    if (rc < 0) {
        s->core_regs[LITESD_CORE_CMDEVT] =
            LITESD_CORE_CMDEVT_DONE | LITESD_CORE_CMDEVT_WR_ERR;
        goto out;
    }

    /*
     * To borrow a comment from Kamil Rakoczy: It looks strange I know, but
     * it's as it should be
     */
    switch (rlen) {
    case LITESD_CORE_CMDCMD_RLEN_LONG:
        s->core_regs[LITESD_CORE_CMDRSP0] = ldl_be_p(&resp[0]);
        s->core_regs[LITESD_CORE_CMDRSP1] = ldl_be_p(&resp[4]);
        s->core_regs[LITESD_CORE_CMDRSP2] = ldl_be_p(&resp[8]);
        s->core_regs[LITESD_CORE_CMDRSP3] = ldl_be_p(&resp[12]);
        break;
    case LITESD_CORE_CMDCMD_RLEN_SHORT:
    case LITESD_CORE_CMDCMD_RLEN_SHORT_BUSY:
        s->core_regs[LITESD_CORE_CMDRSP0] = 0;
        s->core_regs[LITESD_CORE_CMDRSP1] = 0;
        s->core_regs[LITESD_CORE_CMDRSP2] = ldl_be_p(&resp[4]);
        s->core_regs[LITESD_CORE_CMDRSP3] = ldl_be_p(&resp[0]);
        break;
    }
    s->core_regs[LITESD_CORE_CMDEVT] = LITESD_CORE_CMDEVT_DONE;

    switch (xfer) {
    case 0:
        break;
    case LITESD_CORE_CMDCMD_XFER_READ:
        litesd_reader_dma(s);
        break;
    case LITESD_CORE_CMDCMD_XFER_WRITE:
        litesd_writer_dma(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid xfer 0x%x\n",
                      __func__, xfer);
    }

out:
    s->irq_regs[LITESD_IRQ_PENDING] |= LITESD_IRQ_IRQ_CMD_DONE;
    litesd_irq_update(s);
}

static uint64_t litesd_phy_read(void *data, hwaddr addr, unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;
    uint32_t reg;

    switch (idx) {
    case LITESD_PHY_CARDDETECT:
    case LITESD_PHY_CLOCKERDIV:
        reg = s->phy_regs[idx];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
        reg = 0;
    }
    trace_litesd_phy_read(addr, size, reg);
    return reg;
}

static void litesd_phy_write(void *data, hwaddr addr, uint64_t val,
                             unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;

    trace_litesd_phy_write(addr, size, val);

    switch (idx) {
    case LITESD_PHY_CARDDETECT:
        /* read-only */
        break;
    case LITESD_PHY_CLOCKERDIV:
        s->phy_regs[idx] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
    }
}

static uint64_t litesd_core_read(void *data, hwaddr addr, unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;
    uint32_t reg;

    switch (idx) {
    case LITESD_CORE_CMDARG ... LITESD_CORE_BLKCNT:
        reg = s->core_regs[idx];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
        return 0;
    }

    trace_litesd_core_read(addr, size, reg);

    return reg;
}

static void litesd_core_write(void *data, hwaddr addr, uint64_t val,
                              unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;

    trace_litesd_core_write(addr, size, val);

    switch (idx) {
    case LITESD_CORE_CMDARG:
    case LITESD_CORE_CMDCMD:
    case LITESD_CORE_CMDSND:
    case LITESD_CORE_BLKLEN:
    case LITESD_CORE_BLKCNT:
        s->core_regs[idx] = val;
        if (idx == LITESD_CORE_CMDSND && val) {
            litesd_do_cmd(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
    }
}

static uint64_t litesd_reader_read(void *data, hwaddr addr, unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;
    uint64_t reg;

    switch (idx) {
    case LITESD_READER_BASE0 ... LITESD_READER_DONE:
        reg = s->reader_regs[idx];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
        return 0;
    }

    trace_litesd_reader_read(addr, size, reg);

    return reg;
}

static void litesd_reader_write(void *data, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;

    trace_litesd_reader_read(addr, size, val);

    switch (idx) {
    case LITESD_READER_BASE0 ... LITESD_READER_DONE:
        s->reader_regs[idx] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
    }
}

static uint64_t litesd_writer_read(void *data, hwaddr addr, unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;
    uint64_t reg;

    switch (idx) {
    case LITESD_WRITER_BASE0 ... LITESD_WRITER_DONE:
        reg = s->writer_regs[idx];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
        return 0;
    }

    trace_litesd_writer_read(addr, size, reg);

    return reg;
}

static void litesd_writer_write(void *data, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;

    trace_litesd_writer_read(addr, size, val);

    switch (idx) {
    case LITESD_WRITER_BASE0 ... LITESD_WRITER_DONE:
        s->writer_regs[idx] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
    }
}

static uint64_t litesd_irq_read(void *data, hwaddr addr, unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;
    uint64_t reg;

    switch (idx) {
    case LITESD_IRQ_STATUS ... LITESD_IRQ_ENABLE:
        reg = s->irq_regs[idx];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
        return 0;
    }

    trace_litesd_irq_read(addr, size, reg);

    return reg;
}

static void litesd_irq_write(void *data, hwaddr addr, uint64_t val,
                             unsigned int size)
{
    LiteSDState *s = LITESD(data);
    int idx = addr >> 2;

    trace_litesd_irq_read(addr, size, val);

    switch (idx) {
    case LITESD_IRQ_PENDING:
        s->irq_regs[LITESD_IRQ_PENDING] &= ~val;
        break;
    case LITESD_IRQ_ENABLE:
        s->irq_regs[LITESD_IRQ_ENABLE] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown reg %04lx\n", __func__, addr);
    }

    litesd_irq_update(s);
}

static const MemoryRegionOps litesd_phy_ops = {
    .read = litesd_phy_read,
    .write = litesd_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps litesd_core_ops = {
    .read = litesd_core_read,
    .write = litesd_core_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps litesd_reader_ops = {
    .read = litesd_reader_read,
    .write = litesd_reader_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps litesd_writer_ops = {
    .read = litesd_writer_read,
    .write = litesd_writer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps litesd_irq_ops = {
    .read = litesd_irq_read,
    .write = litesd_irq_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

struct LiteSDRegion {
    MemoryRegion *region;
    const MemoryRegionOps *ops;
    hwaddr offset;
    const char *type;
};

static void litesd_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    LiteSDState *s = LITESD(dev);
    struct LiteSDRegion regions[] = {
        { &s->iomem_core, &litesd_core_ops, 0x1000, TYPE_LITESD "-core" },
        { &s->iomem_reader, &litesd_reader_ops, 0x0800, TYPE_LITESD "-reader" },
        { &s->iomem_writer, &litesd_writer_ops, 0x2000, TYPE_LITESD "-writer" },
        { &s->iomem_phy, &litesd_phy_ops, 0x2800, TYPE_LITESD "-phy" },
        { &s->iomem_irq, &litesd_irq_ops, 0x1800, TYPE_LITESD "-irq" },
    };
    unsigned int i;

    /* base region */
    memory_region_init(&s->iomem, OBJECT(s), TYPE_LITESD, 0x3000);
    sysbus_init_mmio(sbd, &s->iomem);

    /* subregions */
    for (i = 0; i < ARRAY_SIZE(regions); i++) {
        struct LiteSDRegion *r = &regions[i];

        memory_region_init_io(r->region, OBJECT(s), r->ops, s, r->type, 0x800);
        memory_region_add_subregion(&s->iomem, r->offset, r->region);
    }

    if (!s->dma_region) {
        error_setg(errp, TYPE_LITESD " 'dma' link property not set");
        return;
    }

    address_space_init(&s->dma_as, s->dma_region, TYPE_LITESD "-dma");

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_LITESD_BUS,
              DEVICE(s), "sd-bus");

    sysbus_init_irq(sbd, &s->irq);

    s->dma_buf_size = 4096;
    s->dma_buf = g_malloc0(s->dma_buf_size);
}

static void litesd_unrealize(DeviceState *dev)
{
    LiteSDState *s = LITESD(dev);
    g_free(s->dma_buf);
}

static void litesd_reset(DeviceState *dev)
{
    LiteSDState *s = LITESD(dev);

    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    memset(s->core_regs, 0, sizeof(s->core_regs));
    memset(s->reader_regs, 0, sizeof(s->reader_regs));
    memset(s->writer_regs, 0, sizeof(s->writer_regs));
    memset(s->irq_regs, 0, sizeof(s->irq_regs));

    if (sdbus_get_inserted(&s->sdbus)) {
        s->phy_regs[LITESD_PHY_CARDDETECT] = 0;
    } else {
        s->phy_regs[LITESD_PHY_CARDDETECT] = LITESD_PHY_CARDDETECT_ABSENT;
    }

    qemu_set_irq(s->irq, 0);
}

static Property litesd_props[] = {
    DEFINE_PROP_LINK("dma", LiteSDState, dma_region,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST(),
};

static void litesd_class_init(ObjectClass *classp, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(classp);

    dc->realize = litesd_realize;
    dc->unrealize = litesd_unrealize;
    dc->reset = litesd_reset;
    device_class_set_props(dc, litesd_props);
}

static const TypeInfo litesd_info = {
    .name          = TYPE_LITESD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LiteSDState),
    .class_init    = litesd_class_init,
};

/* bus */
static void litesd_bus_set_inserted(DeviceState *dev, bool inserted)
{
    LiteSDState *s = LITESD(dev);
    uint32_t cur;

    cur = s->phy_regs[LITESD_PHY_CARDDETECT] & LITESD_PHY_CARDDETECT_ABSENT;
    if (!cur == inserted) {
        return;
    }

    s->phy_regs[LITESD_PHY_CARDDETECT] =
        inserted ? 0 : LITESD_PHY_CARDDETECT_ABSENT;

    /* edge irq; only set if enabled */
    if (s->irq_regs[LITESD_IRQ_ENABLE] & LITESD_IRQ_IRQ_CARD_DETECT) {
        s->irq_regs[LITESD_IRQ_PENDING] |= LITESD_IRQ_IRQ_CARD_DETECT;
    }

    litesd_irq_update(s);
}

static void litesd_bus_class_init(ObjectClass *klass, void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_inserted = litesd_bus_set_inserted;
}

static const TypeInfo litesd_bus_info = {
    .name = TYPE_LITESD_BUS,
    .parent = TYPE_SD_BUS,
    .instance_size = sizeof(SDBus),
    .class_init = litesd_bus_class_init,
};

static void litesd_register_types(void)
{
    type_register_static(&litesd_info);
    type_register_static(&litesd_bus_info);
}

type_init(litesd_register_types)
