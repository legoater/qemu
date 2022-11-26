/*
 * QEMU PowerPC Microwatt
 *
 * Copyright (c) 2021, IBM Corporation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/datadir.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "elf.h"

#include "sysemu/block-backend.h"
#include "sysemu/device_tree.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"

#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/loader.h"
#include "hw/intc/intc.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/ppc/ppc.h"
#include "hw/char/serial.h"
#include "hw/ppc/microwatt.h"

#include "trace.h"

#include <libfdt.h>

/*
 * Microwatt Sys config
 */

#define SYS_REG_SIGNATURE		0x00
#define SYS_REG_INFO			0x08
#define   SYS_REG_INFO_HAS_UART 		(1ull << 0)
#define   SYS_REG_INFO_HAS_DRAM 		(1ull << 1)
#define   SYS_REG_INFO_HAS_BRAM 		(1ull << 2)
#define   SYS_REG_INFO_HAS_SPI_FLASH 		(1ull << 3)
#define   SYS_REG_INFO_HAS_LITEETH 		(1ull << 4)
#define   SYS_REG_INFO_HAS_LARGE_SYSCON	        (1ull << 5)
#define   SYS_REG_INFO_HAS_UART1 		(1ull << 6)
#define   SYS_REG_INFO_HAS_ARTB                 (1ull << 7)
#define   SYS_REG_INFO_HAS_LITESDCARD 		(1ull << 8)
#define SYS_REG_BRAMINFO		0x10
#define   SYS_REG_BRAMINFO_SIZE_MASK		0xfffffffffffffull
#define SYS_REG_DRAMINFO		0x18
#define   SYS_REG_DRAMINFO_SIZE_MASK		0xfffffffffffffull
#define SYS_REG_CLKINFO			0x20
#define   SYS_REG_CLKINFO_FREQ_MASK		0xffffffffffull
#define SYS_REG_CTRL			0x28
#define   SYS_REG_CTRL_DRAM_AT_0		(1ull << 0)
#define   SYS_REG_CTRL_CORE_RESET		(1ull << 1)
#define   SYS_REG_CTRL_SOC_RESET		(1ull << 2)
#define SYS_REG_DRAMINITINFO		0x30
#define SYS_REG_SPI_INFO		0x38
#define   SYS_REG_SPI_INFO_FLASH_OFF_MASK	0xffffffff
#define SYS_REG_UART0_INFO		0x40
#define SYS_REG_UART1_INFO		0x48
#define   SYS_REG_UART_IS_16550			(1ull << 32)

static uint64_t mw_syscon_read(void *opaque, hwaddr addr, unsigned width)
{
    MwSysConState *s = MW_SYSCON(opaque);
    uint64_t val = -1;

    switch (addr) {
    case SYS_REG_SIGNATURE:
    case SYS_REG_INFO:
    case SYS_REG_BRAMINFO:
    case SYS_REG_DRAMINFO:
    case SYS_REG_CLKINFO:
    case SYS_REG_CTRL:
    case SYS_REG_DRAMINITINFO:
    case SYS_REG_SPI_INFO:
    case SYS_REG_UART0_INFO:
    case SYS_REG_UART1_INFO:
        val = s->regs[MW_SYSCON_REG(addr)];
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_mw_syscon_read(addr, width, val);

    return val;
}

static void mw_syscon_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned width)
{
    trace_mw_syscon_write(addr, width, val);

    switch (addr) {
    case SYS_REG_CTRL:
        if (val & SYS_REG_CTRL_SOC_RESET) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps mw_syscon_ops = {
    .read = mw_syscon_read,
    .write = mw_syscon_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
   .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

#define MW_SIGNATURE     0xf00daa5500010001
#define MW_FLASH_OFFSET  0x400000

static void mw_syscon_reset(DeviceState *dev)
{
    MwSysConState *s = MW_SYSCON(dev);

    memset(s->regs, 0, sizeof s->regs);

    s->regs[MW_SYSCON_REG(SYS_REG_SIGNATURE)] = MW_SIGNATURE;
    s->regs[MW_SYSCON_REG(SYS_REG_INFO)] = s->info;
    s->regs[MW_SYSCON_REG(SYS_REG_DRAMINFO)] =
        s->ram_size & SYS_REG_DRAMINFO_SIZE_MASK;
    s->regs[MW_SYSCON_REG(SYS_REG_CLKINFO)] =
        MW_TIMEBASE_FREQ & SYS_REG_CLKINFO_FREQ_MASK;
    s->regs[MW_SYSCON_REG(SYS_REG_SPI_INFO)] =
        MW_FLASH_OFFSET & SYS_REG_SPI_INFO_FLASH_OFF_MASK;
    s->regs[MW_SYSCON_REG(SYS_REG_UART0_INFO)] = SYS_REG_UART_IS_16550;

    if (serial_hd(1)) {
        s->regs[MW_SYSCON_REG(SYS_REG_INFO)] |= SYS_REG_INFO_HAS_UART1;
        s->regs[MW_SYSCON_REG(SYS_REG_UART1_INFO)] = SYS_REG_UART_IS_16550;
    }
}

static void mw_syscon_realize(DeviceState *dev, Error **errp)
{
    MwSysConState *s = MW_SYSCON(dev);

    if (!s->soc) {
        error_setg(errp, TYPE_MW_SYSCON ": 'soc' link not set");
        return;
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &mw_syscon_ops, s,
                          TYPE_MW_SYSCON, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static Property mw_syscon_properties[] = {
    DEFINE_PROP_LINK("soc", MwSysConState, soc, TYPE_MW_SOC, MwSoCState *),
    DEFINE_PROP_UINT64("info", MwSysConState, info,
                       SYS_REG_INFO_HAS_UART |
                       SYS_REG_INFO_HAS_DRAM |
                       SYS_REG_INFO_HAS_SPI_FLASH |
                       SYS_REG_INFO_HAS_LITEETH |
                       SYS_REG_INFO_HAS_LITESDCARD |
                       SYS_REG_INFO_HAS_LARGE_SYSCON),
    DEFINE_PROP_UINT64("ram-size", MwSysConState, ram_size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void mw_syscon_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc    = "MicroWatt Sys Config";
    dc->reset   = mw_syscon_reset;
    dc->realize = mw_syscon_realize;
    dc->user_creatable = false;
    device_class_set_props(dc, mw_syscon_properties);
}

/*
 * Microwatt ICS sources
 */

#define MW_ICS_PRIO_BITS (1 << 3)

static uint64_t mw_ics_reg_read(void *opaque, hwaddr addr, unsigned width)
{
    MwICSState *s = MW_ICS(opaque);
    ICSState *ics = &s->ics;
    uint64_t val;

    switch (addr >> 2) {
    case 0x0: /* Config */
        val = (MW_ICS_PRIO_BITS << 24) | ics->offset;
        break;
    case 0x1: /* Debug. Depends on the implementation */
        val = 0x0;
        break;
    default :
        g_assert_not_reached();
    }

    return val;
}

static void mw_ics_reg_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned width)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                  __func__, addr);
}

static const MemoryRegionOps mw_ics_reg_ops = {
    .read = mw_ics_reg_read,
    .write = mw_ics_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* Top bits hold more info on the source state */
FIELD(MW_ICS_XIVE, PRIO,   0, 7)
FIELD(MW_ICS_XIVE, SERVER, 8, 19)

static uint64_t mw_ics_xive_read(void *opaque, hwaddr addr, unsigned width)
{
    MwICSState *s = MW_ICS(opaque);
    ICSState *ics = &s->ics;
    uint16_t srcno = addr >> 2;
    uint64_t val = 0;

    val = FIELD_DP32(val, MW_ICS_XIVE, PRIO, ics->irqs[srcno].priority);
    val = FIELD_DP32(val, MW_ICS_XIVE, SERVER, ics->irqs[srcno].server);

    trace_mw_ics_read(addr, val);
    return val;
}

static void mw_ics_xive_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned width)
{
    MwICSState *s = MW_ICS(opaque);
    ICSState *ics = &s->ics;
    uint16_t srcno = addr >> 2;
    uint16_t server;
    uint8_t prio;

    trace_mw_ics_write(addr, val);

    prio   = FIELD_EX32(val, MW_ICS_XIVE, PRIO);
    server = FIELD_EX32(val, MW_ICS_XIVE, SERVER);

    /* Update the underlying ICSState caching the configuration */
    ics_write_xive(ics, srcno, server, prio, prio);
}

static const MemoryRegionOps mw_ics_xive_ops = {
    .read = mw_ics_xive_read,
    .write = mw_ics_xive_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mw_ics_set_irq(void *opaque, int srcno, int level)
{
    MwICSState *s = MW_ICS(opaque);

    ics_set_irq(&s->ics, srcno, level);
}

#define MW_ICS_IRQ_BASE    0x10
#define MW_ICS_NR_IRQS     0x10           /* HW allows a max of 0x100 */
#define MW_ICS_XIVE_OFFSET 0x800

static void mw_ics_realize(DeviceState *dev, Error **errp)
{
    MwICSState *s = MW_ICS(dev);
    ICSState *ics = &s->ics;
    int i;

    if (!qdev_realize(DEVICE(ics), NULL, errp)) {
        return;
    }

    for (i = 0; i < ics->nr_irqs; i++) {
        ics_set_irq_type(ics, i, false);
    }

    qdev_init_gpio_in(dev, mw_ics_set_irq, ics->nr_irqs);

    /* Global window. Size it with the number of irqs  */
    memory_region_init(&s->mmio, OBJECT(s), TYPE_MW_ICS,
                       MW_ICS_XIVE_OFFSET + ics->nr_irqs * 4);

    /* Regs at 0x0 */
    memory_region_init_io(&s->reg_mmio, OBJECT(s), &mw_ics_reg_ops, s,
                          TYPE_MW_ICS "-regs", 0x8);
    memory_region_add_subregion(&s->mmio, 0x0, &s->reg_mmio);

    /* XIVE entries at 2K offset */
    memory_region_init_io(&s->xive_mmio, OBJECT(s), &mw_ics_xive_ops, s,
                          TYPE_MW_ICS "-xive", ics->nr_irqs * 4);
    memory_region_add_subregion(&s->mmio, MW_ICS_XIVE_OFFSET,
                                &s->xive_mmio);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static void mw_ics_instance_init(Object *obj)
{
    MwICSState *s = MW_ICS(obj);
    ICSState *ics = &s->ics;

    object_initialize_child(obj, "ics", &s->ics, TYPE_ICS);
    object_property_add_alias(obj, "nr-irqs", OBJECT(&s->ics), "nr-irqs");
    object_property_add_alias(obj, "xics", OBJECT(&s->ics), "xics");

    /* HW defines the IRQ base number */
    ics->offset = MW_ICS_IRQ_BASE;
}

static void mw_ics_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc    = "MicroWatt ICS";
    dc->realize = mw_ics_realize;
    dc->user_creatable = false;
}

/*
 * Microwatt CPU Presenters
 */
static void mw_icp_realize(DeviceState *dev, Error **errp)
{
    MwICPState *s = MW_ICP(dev);
    MwSoCState *soc = s->soc;
    MwSoCClass *msc;
    int i, j;

    if (!s->soc) {
        error_setg(errp, TYPE_MW_ICP ": 'soc' link not set");
        return;
    }

    msc = MW_SOC_GET_CLASS(soc);

    memory_region_init(&s->mmio, OBJECT(s), TYPE_MW_ICP, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    for (i = 0; i < msc->num_cpus; i++) {
        MwCore *mw_core = &soc->cores[i];
        int core_hwid = CPU_CORE(mw_core)->core_id;

        for (j = 0; j < CPU_CORE(mw_core)->nr_threads; j++) {
            uint32_t pir = (core_hwid << 2) | j; /* P9 style */
            PnvICPState *icp = PNV_ICP(xics_icp_get(soc->xics, pir));

            memory_region_add_subregion(&s->mmio, pir << 12, &icp->mmio);
        }
    }
}

static Property mw_icp_properties[] = {
    DEFINE_PROP_LINK("soc", MwICPState, soc, TYPE_MW_SOC, MwSoCState *),
    DEFINE_PROP_END_OF_LIST(),
};

static void mw_icp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc    = "MicroWatt ICP";
    dc->realize = mw_icp_realize;
    dc->user_creatable = false;
    device_class_set_props(dc, mw_icp_properties);
}

/*
 * Microwatt SoC
 */

static const hwaddr mw_soc_memmap[] = {
    [MW_DEV_IOMEM]   = MW_SOC_IOMEM_BASE,
    [MW_DEV_SYSCON]  = MW_SOC_IOMEM_BASE  + 0x0,
    [MW_DEV_UART0]   = MW_SOC_IOMEM_BASE  + 0x2000,
    [MW_DEV_UART1]   = MW_SOC_IOMEM_BASE  + 0x3000, /* TODO */
    [MW_DEV_ICP]     = MW_SOC_IOMEM_BASE  + 0x4000,
    [MW_DEV_ICS]     = MW_SOC_IOMEM_BASE  + 0x5000,
    [MW_DEV_SPI]     = MW_SOC_IOMEM_BASE  + 0x6000,
    [MW_DEV_GPIO]    = MW_SOC_IOMEM_BASE  + 0x7000,

    [MW_DEV_DRAM]    = MW_SOC_EXT_IO_BASE + 0x0,
    [MW_DEV_ETH]     = MW_SOC_EXT_IO_BASE + 0x21000, /* TODO fix value */
    [MW_DEV_ETH_BUF] = MW_SOC_EXT_IO_BASE + 0x30000,
    [MW_DEV_SD]      = MW_SOC_EXT_IO_BASE + 0x40000,
};

static const int mw_soc_irqmap[] = {
    [MW_DEV_UART0]  = MW_ICS_IRQ_BASE + 0x0,
    [MW_DEV_ETH]    = MW_ICS_IRQ_BASE + 0x1,
    [MW_DEV_UART1]  = MW_ICS_IRQ_BASE + 0x2,
    [MW_DEV_SD]     = MW_ICS_IRQ_BASE + 0x3,
    [MW_DEV_GPIO]   = MW_ICS_IRQ_BASE + 0x4,
};

static qemu_irq mw_soc_get_irq(MwSoCState *s, int ctrl)
{
    int hwirq = mw_soc_irqmap[ctrl];
    ICSState *ics = &s->ics.ics;

    assert(ics_valid_irq(ics, hwirq));
    return qdev_get_gpio_in(DEVICE(&s->ics), hwirq - ics->offset);
}

static void mw_soc_realize(DeviceState *dev, Error **errp)
{
    MwSoCState *s = MW_SOC(dev);
    MwSoCClass *msc = MW_SOC_GET_CLASS(s);
    int i;

    /* XICS fabric points to the machine. Move to the SoC ? */
    if (!s->xics) {
        error_setg(errp, TYPE_MW_SOC ": 'xics' link not set");
        return;
    }

    /* IO space */
    create_unimplemented_device(TYPE_MW_SOC "-io", mw_soc_memmap[MW_DEV_IOMEM],
                                MW_SOC_IOMEM_SIZE);

    /* CPU */
    for (i = 0; i < msc->num_cpus; i++) {
        object_property_set_link(OBJECT(&s->cores[i]), "soc", OBJECT(s),
                                 &error_abort);
        if (!qdev_realize(DEVICE(&s->cores[i]), NULL, errp)) {
            return;
        }
    }

    /* Sys config */
    object_property_set_link(OBJECT(&s->syscon), "soc", OBJECT(s), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->syscon), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->syscon), 0, mw_soc_memmap[MW_DEV_SYSCON]);

    /* CPU IRQ Presenters */
    object_property_set_link(OBJECT(&s->icp), "soc", OBJECT(s), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->icp), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->icp), 0, mw_soc_memmap[MW_DEV_ICP]);

    /* HW IRQ Sources */
    if (!object_property_set_int(OBJECT(&s->ics), "nr-irqs", MW_ICS_NR_IRQS,
                                 errp)) {
        return;
    }
    object_property_set_link(OBJECT(&s->ics), "xics", OBJECT(s->xics),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ics), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ics), 0, mw_soc_memmap[MW_DEV_ICS]);

    /* UART */
    serial_mm_init(get_system_memory(), mw_soc_memmap[MW_DEV_UART0], 2,
                   mw_soc_get_irq(s, MW_DEV_UART0), 115200,
                   serial_hd(0), DEVICE_LITTLE_ENDIAN);

    if (serial_hd(1)) {
        serial_mm_init(get_system_memory(), mw_soc_memmap[MW_DEV_UART1], 2,
                       mw_soc_get_irq(s, MW_DEV_UART1), 115200,
                       serial_hd(1), DEVICE_LITTLE_ENDIAN);
    }

    /* Network */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->eth), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->eth), 0, mw_soc_memmap[MW_DEV_ETH]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->eth), 1, mw_soc_memmap[MW_DEV_ETH_BUF]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->eth), 0,
                       mw_soc_get_irq(s, MW_DEV_ETH));

    /* SPI */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi), 0, mw_soc_memmap[MW_DEV_SPI]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi), 1, MW_SOC_FLASH_BASE);

    /* SD/MMC */
    object_property_set_link(OBJECT(&s->sd), "dma",
                             OBJECT(get_system_memory()), &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sd), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sd), 0, mw_soc_memmap[MW_DEV_SD]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sd), 0,
                       mw_soc_get_irq(s, MW_DEV_SD));

    /* GPIO */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, mw_soc_memmap[MW_DEV_GPIO]);

    /* DRAM init firmware (is not a ROM !) */
    memory_region_init_ram(&s->dram_init, OBJECT(s), "dram-init", 16 * MiB,
                           &error_abort);
    memory_region_add_subregion(get_system_memory(), MW_SOC_DRAM_INIT,
                                &s->dram_init);
}

static void mw_soc_instance_init(Object *obj)
{
    MwSoCState *s = MW_SOC(obj);
    MwSoCClass *msc = MW_SOC_GET_CLASS(s);
    int i;

    for (i = 0; i < msc->num_cpus; i++) {
        object_initialize_child(obj, "cpu[*]", &s->cores[i], msc->cpu_type);
    }

    object_initialize_child(obj, "mw-syscon", &s->syscon, TYPE_MW_SYSCON);
    object_property_add_alias(obj, "ram-size", OBJECT(&s->syscon), "ram-size");
    object_initialize_child(obj, "mw-ics", &s->ics, TYPE_MW_ICS);
    object_initialize_child(obj, "mw-icp", &s->icp, TYPE_MW_ICP);
    object_initialize_child(obj, "eth", &s->eth, TYPE_LITEETH);
    object_initialize_child(obj, "spi", &s->spi, TYPE_LITESPI);
    object_initialize_child(obj, "sd", &s->sd, TYPE_LITESD);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_UNIMPLEMENTED_DEVICE);
    qdev_prop_set_uint64(DEVICE(&s->gpio), "size", 0x1000);
    qdev_prop_set_string(DEVICE(&s->gpio), "name", "gpio");
}

static void mw_soc_power9_intc_create(MwSoCState *soc, PowerPCCPU *cpu,
                                      Error **errp)
{
    Error *local_err = NULL;
    Object *obj;
    MwCPUState *mw_cpu = mw_cpu_state(cpu);

    /*
     * TODO: replace/rename PNV_ICP -> XICS_NATIVE_ICP ? it's not
     * machine dependent.
     */
    obj = icp_create(OBJECT(cpu), TYPE_PNV_ICP, soc->xics, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    mw_cpu->intc = obj;
}

static void mw_soc_power9_intc_reset(MwSoCState *soc, PowerPCCPU *cpu)
{
    MwCPUState *mw_cpu = mw_cpu_state(cpu);

    icp_reset(ICP(mw_cpu->intc));
}

static void mw_soc_power9_intc_destroy(MwSoCState *soc, PowerPCCPU *cpu)
{
    MwCPUState *mw_cpu = mw_cpu_state(cpu);

    icp_destroy(ICP(mw_cpu->intc));
    mw_cpu->intc = NULL;
}

static void mw_soc_power9_intc_print_info(MwSoCState *soc, PowerPCCPU *cpu,
                                          Monitor *mon)
{
    icp_pic_print_info(ICP(mw_cpu_state(cpu)->intc), mon);
}

static void mw_soc_power9_class_init(ObjectClass *oc, void *data)
{
    MwSoCClass *sc = MW_SOC_CLASS(oc);

    sc->cpu_type = MW_CORE_TYPE_NAME("power9mw_v1.0");
    sc->num_cpus = 1;
    sc->intc_create = mw_soc_power9_intc_create;
    sc->intc_reset = mw_soc_power9_intc_reset;
    sc->intc_destroy = mw_soc_power9_intc_destroy;
    sc->intc_print_info = mw_soc_power9_intc_print_info;
}

static Property mw_soc_properties[] = {
    DEFINE_PROP_LINK("xics", MwSoCState, xics, TYPE_XICS_FABRIC, XICSFabric *),
    DEFINE_PROP_END_OF_LIST(),
};

static void mw_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mw_soc_realize;
    dc->user_creatable = false;
    device_class_set_props(dc, mw_soc_properties);
}

/*
 * Microwatt machines
 */

#define TYPE_MW_MACHINE       MACHINE_TYPE_NAME("microwatt")
typedef struct MwMachineClass MwMachineClass;
typedef struct MwMachineState MwMachineState;
DECLARE_OBJ_CHECKERS(MwMachineState, MwMachineClass,
                     MW_MACHINE, TYPE_MW_MACHINE)

struct MwMachineState {
    /*< private >*/
    MachineState parent_obj;

    MemoryRegion ram_alias;

    MwSoCState   soc;
};

struct MwMachineClass {
    /*< private >*/
    MachineClass parent_class;

    const char   *soc_name;
};

static void mw_machine_reset(MachineState *machine, ShutdownCause reason)
{
    /* TODO: reload or generate the DTB instead */
    qemu_devices_reset(reason);
}

static int mw_dtb_update(void *fdt, const char *cmdline,
                         hwaddr initrd_base, int initrd_size)
{
    int ret;

    ret = qemu_fdt_setprop_string(fdt, "/chosen", "bootargs", cmdline);
    if (ret < 0) {
        goto out;
    }

    if (initrd_size) {
        ret = qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-start",
                                    initrd_base);
        if (ret < 0) {
            goto out;
        }

        ret = qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                                    (initrd_base + initrd_size));
    }
out:
    return ret;
}

static hwaddr mw_dtb_load(MachineState *machine,
                          hwaddr kernel_base, int kernel_size,
                          hwaddr initrd_base, int initrd_size)
{
    void *fdt = NULL;
    hwaddr dt_base = 0;
    g_autofree char *filename;
    int size;
    int ret;

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->dtb);
    if (!filename) {
        error_report("Couldn't find dtb file '%s'", machine->dtb);
        exit(1);
    }

    fdt = load_device_tree(filename, &size);
    if (!fdt) {
        error_report("Couldn't load dtb file '%s'", filename);
        exit(1);
    }

    ret = mw_dtb_update(fdt, machine->kernel_cmdline, initrd_base, initrd_size);
    if (ret < 0) {
        error_report("Couldn't update dtb file '%s'", filename);
        exit(1);
    }

    qemu_fdt_dumpdtb(fdt, fdt_totalsize(fdt));

    if (initrd_size) {
            dt_base = QEMU_ALIGN_UP(initrd_base + initrd_size, 0x10000);
    } else {
            dt_base = QEMU_ALIGN_UP(kernel_base + kernel_size, 0x10000);
    }

    cpu_physical_memory_write(dt_base, fdt, fdt_totalsize(fdt));

    g_free(fdt);
    return dt_base;
}

static void mw_attach_flash(LiteSPIState *s, const char *flashtype)
{
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    qemu_irq cs_line;
    DeviceState *dev;

    dev = qdev_new(flashtype);
    if (dinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
    }
    qdev_realize_and_unref(dev, BUS(s->spi), &error_fatal);

    cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(s), 0, cs_line);
}

static void write_boot_rom(DriveInfo *dinfo, hwaddr addr, size_t rom_size,
                           Error **errp)
{
    BlockBackend *blk = blk_by_legacy_dinfo(dinfo);
    uint8_t *storage;
    int64_t size;

    /* The block backend size should have already been 'validated' by
     * the creation of the m25p80 object.
     */
    size = blk_getlength(blk);
    if (size <= 0) {
        error_setg(errp, "failed to get flash size");
        return;
    }

    if (rom_size > size) {
        rom_size = size;
    }

    storage = g_new0(uint8_t, rom_size);
    if (blk_pread(blk, 0, rom_size, storage, 0) < 0) {
        error_setg(errp, "failed to read the initial flash content");
        return;
    }

    rom_add_blob_fixed("mw.boot_rom", storage, rom_size, addr);
    g_free(storage);
}

static void mw_machine_init(MachineState *machine)
{
    MwMachineState *mw = MW_MACHINE(machine);
    MwMachineClass *mwc = MW_MACHINE_GET_CLASS(machine);
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    hwaddr dt_base = 0;
    hwaddr boot_entry = 0;
    NICInfo *nd = &nd_table[0];
    DriveInfo *drive0 = drive_get(IF_MTD, 0, 0);

    if (machine->ram_size < mc->default_ram_size) {
        g_autofree char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be bigger than %s", sz);
        exit(EXIT_FAILURE);
    }

    qemu_check_nic_model(nd, TYPE_LITEETH);

    /* RAM mapping and alias */
    memory_region_add_subregion(get_system_memory(), MW_SOC_DRAM_BASE,
                                machine->ram);

    /* for -kernel boot */
    memory_region_init_alias(&mw->ram_alias, NULL, "mw-ram_alias",
                             machine->ram, 0x0, machine->ram_size);
    memory_region_add_subregion(get_system_memory(), MW_SOC_MEMORY_BASE,
                                &mw->ram_alias);

    /* SoC */
    object_initialize_child(OBJECT(machine), "soc", &mw->soc, mwc->soc_name);
    object_property_set_uint(OBJECT(&mw->soc), "ram-size", machine->ram_size,
                             &error_abort);
    object_property_set_link(OBJECT(&mw->soc), "xics", OBJECT(mw),
                             &error_abort);

    qdev_set_nic_properties(DEVICE(&mw->soc.eth), nd);

    sysbus_realize(SYS_BUS_DEVICE(&mw->soc), &error_abort);

    mw_attach_flash(&mw->soc.spi, "n25q128a13");

    /* load kernel and initrd */
    if (machine->kernel_filename) {
        hwaddr kernel_base = 0;
        int kernel_size = 0;
        hwaddr initrd_base = 0;
        int initrd_size = 0;

        kernel_size = load_elf(machine->kernel_filename, NULL, NULL, NULL,
                               &boot_entry, &kernel_base, NULL, NULL,
                               0 /* LE */, PPC_ELF_MACHINE, 0, 0);
        if (kernel_size < 0) {
            error_report("Could not load kernel '%s' : %s",
                         machine->kernel_filename, load_elf_strerror(kernel_size));
            exit(1);
        }

        if (machine->initrd_filename) {
            initrd_base = QEMU_ALIGN_UP(kernel_base + kernel_size, 0x10000);
            initrd_size = load_image_targphys(machine->initrd_filename,
                                              initrd_base, 16 * MiB /* Some value */);
            if (initrd_size < 0) {
                error_report("Could not load initial ram disk '%s'",
                             machine->initrd_filename);
                exit(1);
            }
        }

        if (machine->dtb) {
            dt_base = mw_dtb_load(machine, kernel_base, kernel_size, initrd_base,
                                  initrd_size);
        }
    } else if (machine->firmware) {
        g_autofree char *filename;
        int ret;

        /* load some blob if no ELF */
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(1);
        }

        ret = load_image_mr(filename, &mw->soc.dram_init);
        if (ret < 0) {
            error_report("Could not load firmware '%s'", filename);
            exit(1);
        }

        boot_entry = MW_SOC_DRAM_INIT;

    } else if (drive0) {
        uint64_t rom_size = memory_region_size(&mw->soc.dram_init);

        write_boot_rom(drive0, MW_SOC_DRAM_INIT, rom_size, &error_abort);

        boot_entry = MW_SOC_DRAM_INIT;
    }

    mw->soc.boot_info.entry = boot_entry;
    mw->soc.boot_info.dt_base = dt_base;
}

static ICSState *mw_ics_get(XICSFabric *xi, int irq)
{
    MwMachineState *mw = MW_MACHINE(xi);
    ICSState *ics = &mw->soc.ics.ics;

    return ics_valid_irq(ics, irq) ? ics : NULL;
}

static void mw_ics_resend(XICSFabric *xi)
{
    MwMachineState *mw = MW_MACHINE(xi);

    ics_resend(&mw->soc.ics.ics);
}

static ICPState *mw_icp_get(XICSFabric *xi, int pir)
{
    PowerPCCPU *cpu = ppc_get_vcpu_by_pir(pir);

    return cpu ? ICP(mw_cpu_state(cpu)->intc) : NULL;
}

static void mw_pic_print_info(InterruptStatsProvider *obj,
                               Monitor *mon)
{
    MwMachineState *mw = MW_MACHINE(obj);
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);

        MW_SOC_GET_CLASS(&mw->soc)->intc_print_info(&mw->soc, cpu, mon);
    }
    ics_pic_print_info(&mw->soc.ics.ics, mon);
}

static void mw_machine_power9_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    XICSFabricClass *xic = XICS_FABRIC_CLASS(oc);
    MwMachineClass *mwc = MW_MACHINE_CLASS(oc);

    mc->desc = "Microwatt POWER9";
    mc->alias = "microwatt";

    mwc->soc_name = TYPE_MW_SOC "-power9";

    xic->icp_get = mw_icp_get;
    xic->ics_get = mw_ics_get;
    xic->ics_resend = mw_ics_resend;
}

static void mw_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    InterruptStatsProviderClass *ispc = INTERRUPT_STATS_PROVIDER_CLASS(oc);

    mc->desc = "Microwatt Generic";
    mc->init = mw_machine_init;
    mc->reset = mw_machine_reset;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->max_cpus = 1;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "mw-ram";

    ispc->print_info = mw_pic_print_info;
}

static const TypeInfo types[] = {
    {
        .name          = MACHINE_TYPE_NAME("microwatt9"),
        .parent        = TYPE_MW_MACHINE,
        .class_init    = mw_machine_power9_class_init,
        .interfaces    = (InterfaceInfo[]) {
            { TYPE_XICS_FABRIC },
            { },
        },
    },
    {
        .name          = TYPE_MW_MACHINE,
        .parent        = TYPE_MACHINE,
        .abstract      = true,
        .instance_size = sizeof(MwMachineState),
        .class_init    = mw_machine_class_init,
        .class_size    = sizeof(MwMachineClass),
        .interfaces    = (InterfaceInfo[]) {
            { TYPE_INTERRUPT_STATS_PROVIDER },
            { },
        },
    },

    {
        .name          = TYPE_MW_SOC "-power9",
        .parent        = TYPE_MW_SOC,
        .class_init    = mw_soc_power9_class_init,
    },
    {
        .name          = TYPE_MW_SOC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_init = mw_soc_instance_init,
        .instance_size = sizeof(MwSoCState),
        .class_init    = mw_soc_class_init,
        .class_size    = sizeof(MwSoCClass),
        .abstract       = true,
    },

    {
        .name          = TYPE_MW_ICP,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MwICPState),
        .class_init    = mw_icp_class_init,
    },
    {
        .name          = TYPE_MW_ICS,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_init = mw_ics_instance_init,
        .instance_size = sizeof(MwICSState),
        .class_init    = mw_ics_class_init,
    },
    {
        .name          = TYPE_MW_SYSCON,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MwSysConState),
        .class_init    = mw_syscon_class_init,
    },
};

DEFINE_TYPES(types)
