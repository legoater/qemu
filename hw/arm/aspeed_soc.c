/*
 * ASPEED SoC family
 *
 * Andrew Jeffery <andrew@aj.id.au>
 * Jeremy Kerr <jk@ozlabs.org>
 *
 * Copyright 2016 IBM Corp.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "hw/arm/aspeed_soc.h"
#include "hw/char/serial.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/aspeed_i2c.h"
#include "net/net.h"
#include "sysemu/blockdev.h"

#define ASPEED_SOC_IOMEM_SIZE       0x00200000

static const hwaddr aspeed_soc_ast2400_memmap[] = {
    [ASPEED_IOMEM]  = 0x1E600000,
    [ASPEED_FMC]    = 0x1E620000,
    [ASPEED_SPI1]   = 0x1E630000,
    [ASPEED_VIC]    = 0x1E6C0000,
    [ASPEED_SDMC]   = 0x1E6E0000,
    [ASPEED_SCU]    = 0x1E6E2000,
    [ASPEED_XDMA]   = 0x1E6E7000,
    [ASPEED_ADC]    = 0x1E6E9000,
    [ASPEED_SRAM]   = 0x1E720000,
    [ASPEED_GPIO]   = 0x1E780000,
    [ASPEED_RTC]    = 0x1E781000,
    [ASPEED_TIMER1] = 0x1E782000,
    [ASPEED_WDT]    = 0x1E785000,
    [ASPEED_PWM]    = 0x1E786000,
    [ASPEED_LPC]    = 0x1E789000,
    [ASPEED_IBT]    = 0x1E789140,
    [ASPEED_I2C]    = 0x1E78A000,
    [ASPEED_ETH1]   = 0x1E660000,
    [ASPEED_ETH2]   = 0x1E680000,
    [ASPEED_UART1]  = 0x1E783000,
    [ASPEED_UART5]  = 0x1E784000,
    [ASPEED_VUART]  = 0x1E787000,
    [ASPEED_SDRAM]  = 0x40000000,
};

static const hwaddr aspeed_soc_ast2500_memmap[] = {
    [ASPEED_IOMEM]  = 0x1E600000,
    [ASPEED_FMC]    = 0x1E620000,
    [ASPEED_SPI1]   = 0x1E630000,
    [ASPEED_SPI2]   = 0x1E631000,
    [ASPEED_VIC]    = 0x1E6C0000,
    [ASPEED_SDMC]   = 0x1E6E0000,
    [ASPEED_SCU]    = 0x1E6E2000,
    [ASPEED_XDMA]   = 0x1E6E7000,
    [ASPEED_ADC]    = 0x1E6E9000,
    [ASPEED_SRAM]   = 0x1E720000,
    [ASPEED_SDHCI]  = 0x1E740000,
    [ASPEED_GPIO]   = 0x1E780000,
    [ASPEED_RTC]    = 0x1E781000,
    [ASPEED_TIMER1] = 0x1E782000,
    [ASPEED_WDT]    = 0x1E785000,
    [ASPEED_PWM]    = 0x1E786000,
    [ASPEED_LPC]    = 0x1E789000,
    [ASPEED_IBT]    = 0x1E789140,
    [ASPEED_I2C]    = 0x1E78A000,
    [ASPEED_ETH1]   = 0x1E660000,
    [ASPEED_ETH2]   = 0x1E680000,
    [ASPEED_UART1]  = 0x1E783000,
    [ASPEED_UART5]  = 0x1E784000,
    [ASPEED_VUART]  = 0x1E787000,
    [ASPEED_SDRAM]  = 0x80000000,
};

static const hwaddr aspeed_soc_ast2600_memmap[] = {
    [ASPEED_SRAM]   = 0x10000000,
    /* 0x16000000     0x17FFFFFF : AHB BUS do LPC Bus bridge */
    [ASPEED_IOMEM]  = 0x1E600000,
    [ASPEED_PWM]    = 0x1E610000,
    [ASPEED_FMC]    = 0x1E620000,
    [ASPEED_SPI1]   = 0x1E630000,
    [ASPEED_SPI2]   = 0x1E641000,
    [ASPEED_MII1]   = 0x1E650000,
    [ASPEED_MII2]   = 0x1E650008,
    [ASPEED_MII3]   = 0x1E650010,
    [ASPEED_MII4]   = 0x1E650018,
    [ASPEED_ETH1]   = 0x1E660000,
    [ASPEED_ETH3]   = 0x1E670000,
    [ASPEED_ETH2]   = 0x1E680000,
    [ASPEED_ETH4]   = 0x1E690000,
    [ASPEED_VIC]    = 0x1E6C0000,
    [ASPEED_SDMC]   = 0x1E6E0000,
    [ASPEED_SCU]    = 0x1E6E2000,
    [ASPEED_XDMA]   = 0x1E6E7000,
    [ASPEED_ADC]    = 0x1E6E9000,
    [ASPEED_VIDEO]  = 0x1E700000,
    [ASPEED_SDHCI]  = 0x1E740000,
    [ASPEED_GPIO]   = 0x1E780000,
    [ASPEED_RTC]    = 0x1E781000,
    [ASPEED_TIMER1] = 0x1E782000,
    [ASPEED_WDT]    = 0x1E785000,
    [ASPEED_LPC]    = 0x1E789000,
    [ASPEED_IBT]    = 0x1E789140,
    [ASPEED_I2C]    = 0x1E78A000,
    [ASPEED_UART1]  = 0x1E783000,
    [ASPEED_UART5]  = 0x1E784000,
    [ASPEED_VUART]  = 0x1E787000,
    [ASPEED_FSI1]   = 0x1E79B000,
    [ASPEED_FSI2]   = 0x1E79B100,
    [ASPEED_SDRAM]  = 0x80000000,
};

#define ASPEED_A7MPCORE_ADDR 0x40460000

static const int aspeed_soc_ast2400_irqmap[] = {
    [ASPEED_UART1]  = 9,
    [ASPEED_UART2]  = 32,
    [ASPEED_UART3]  = 33,
    [ASPEED_UART4]  = 34,
    [ASPEED_UART5]  = 10,
    [ASPEED_VUART]  = 8,
    [ASPEED_FMC]    = 19,
    [ASPEED_SDMC]   = 0,
    [ASPEED_SCU]    = 21,
    [ASPEED_ADC]    = 31,
    [ASPEED_GPIO]   = 20,
    [ASPEED_RTC]    = 22,
    [ASPEED_TIMER1] = 16,
    [ASPEED_TIMER2] = 17,
    [ASPEED_TIMER3] = 18,
    [ASPEED_TIMER4] = 35,
    [ASPEED_TIMER5] = 36,
    [ASPEED_TIMER6] = 37,
    [ASPEED_TIMER7] = 38,
    [ASPEED_TIMER8] = 39,
    [ASPEED_WDT]    = 27,
    [ASPEED_PWM]    = 28,
    [ASPEED_LPC]    = 8,
    [ASPEED_IBT]    = 8, /* LPC */
    [ASPEED_I2C]    = 12,
    [ASPEED_ETH1]   = 2,
    [ASPEED_ETH2]   = 3,
    [ASPEED_XDMA]   = 6,
    [ASPEED_SDHCI]  = 26,
};

#define aspeed_soc_ast2500_irqmap aspeed_soc_ast2400_irqmap

#define ASPEED_SOC_AST2600_MAX_IRQ 128

static const int aspeed_soc_ast2600_irqmap[] = {
    [ASPEED_UART1]  = 47,
    [ASPEED_UART2]  = 48,
    [ASPEED_UART3]  = 49,
    [ASPEED_UART4]  = 50,
    [ASPEED_UART5]  = 8,
    [ASPEED_VUART]  = 8,
    [ASPEED_FMC]    = 39,
    [ASPEED_SDMC]   = 0,
    [ASPEED_SCU]    = 12,
    [ASPEED_XDMA]   = 6,
    [ASPEED_SDHCI]  = 43,
    [ASPEED_ADC]    = 46,
    [ASPEED_GPIO]   = 40,
    [ASPEED_RTC]    = 13,
    [ASPEED_TIMER1] = 16,
    [ASPEED_TIMER2] = 17,
    [ASPEED_TIMER3] = 18,
    [ASPEED_TIMER4] = 19,
    [ASPEED_TIMER5] = 20,
    [ASPEED_TIMER6] = 21,
    [ASPEED_TIMER7] = 22,
    [ASPEED_TIMER8] = 23,
    [ASPEED_WDT]    = 24,
    [ASPEED_PWM]    = 44,
    [ASPEED_LPC]    = 35,
    [ASPEED_IBT]    = 35, /* LPC */
    [ASPEED_I2C]    = 110, /* 110 -> 125 */
    [ASPEED_ETH1]   = 2,
    [ASPEED_ETH2]   = 3,
    [ASPEED_FSI1]   = 100,
    [ASPEED_FSI2]   = 101,
};

static const AspeedSoCInfo aspeed_socs[] = {
    {
        .name         = "ast2400-a1",
        .cpu_type     = ARM_CPU_TYPE_NAME("arm926"),
        .silicon_rev  = AST2400_A1_SILICON_REV,
        .sram_size    = 0x8000,
        .spis_num     = 1,
        .wdts_num     = 2,
        .macs_num     = 2,
        .irqmap       = aspeed_soc_ast2400_irqmap,
        .memmap       = aspeed_soc_ast2400_memmap,
        .num_cpus     = 1,
    }, {
        .name         = "ast2500-a1",
        .cpu_type     = ARM_CPU_TYPE_NAME("arm1176"),
        .silicon_rev  = AST2500_A1_SILICON_REV,
        .sram_size    = 0x9000,
        .spis_num     = 2,
        .wdts_num     = 3,
        .macs_num     = 2,
        .irqmap       = aspeed_soc_ast2500_irqmap,
        .memmap       = aspeed_soc_ast2500_memmap,
        .num_cpus     = 1,
    }, {
        .name         = "ast2600-a0",
        .cpu_type     = ARM_CPU_TYPE_NAME("cortex-a7"),
        .silicon_rev  = AST2600_A0_SILICON_REV,
        .sram_size    = 0x10000,
        .spis_num     = 2,
        .wdts_num     = 4,
        .macs_num     = 4,
        .irqmap       = aspeed_soc_ast2600_irqmap,
        .memmap       = aspeed_soc_ast2600_memmap,
        .num_cpus     = 2,
    },
};

static qemu_irq aspeed_soc_get_irq(AspeedSoCState *s, int ctrl)
{
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    DeviceState *intc = ASPEED_IS_AST2600(sc->info->silicon_rev) ?
        DEVICE(&s->a7mpcore) : DEVICE(&s->vic);

    return qdev_get_gpio_in(intc, sc->info->irqmap[ctrl]);
}

static void aspeed_soc_init(Object *obj)
{
    AspeedSoCState *s = ASPEED_SOC(obj);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    int i;
    char socname[8];
    char typename[64];

    if (sscanf(sc->info->name, "%7s", socname) != 1) {
        g_assert_not_reached();
    }

    for (i = 0; i < sc->info->num_cpus; i++) {
        object_initialize_child(obj, "cpu[*]", OBJECT(&s->cpu[i]),
                                sizeof(s->cpu[i]), sc->info->cpu_type,
                                &error_abort, NULL);
    }

    snprintf(typename, sizeof(typename), "aspeed.scu-%s", socname);
    sysbus_init_child_obj(obj, "scu", OBJECT(&s->scu), sizeof(s->scu),
                          typename);
    qdev_prop_set_uint32(DEVICE(&s->scu), "silicon-rev",
                         sc->info->silicon_rev);
    object_property_add_alias(obj, "hw-strap1", OBJECT(&s->scu),
                              "hw-strap1", &error_abort);
    object_property_add_alias(obj, "hw-strap2", OBJECT(&s->scu),
                              "hw-strap2", &error_abort);
    object_property_add_alias(obj, "hw-prot-key", OBJECT(&s->scu),
                              "hw-prot-key", &error_abort);

    if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
        sysbus_init_child_obj(obj, "a7mpcore", &s->a7mpcore,
                              sizeof(s->a7mpcore), TYPE_A15MPCORE_PRIV);
    } else {
        sysbus_init_child_obj(obj, "vic", OBJECT(&s->vic), sizeof(s->vic),
                              TYPE_ASPEED_VIC);
    }

    sysbus_init_child_obj(obj, "rtc", OBJECT(&s->rtc), sizeof(s->rtc),
                          TYPE_ASPEED_RTC);

    sysbus_init_child_obj(obj, "timerctrl", OBJECT(&s->timerctrl),
                          sizeof(s->timerctrl), TYPE_ASPEED_TIMER);
    object_property_add_const_link(OBJECT(&s->timerctrl), "scu",
                                   OBJECT(&s->scu), &error_abort);

    sysbus_init_child_obj(obj, "i2c", OBJECT(&s->i2c), sizeof(s->i2c),
                          TYPE_ASPEED_I2C);
    qdev_prop_set_uint32(DEVICE(&s->i2c), "silicon-rev",
                         sc->info->silicon_rev);

    sysbus_init_child_obj(obj, "adc", OBJECT(&s->adc), sizeof(s->adc),
                          TYPE_ASPEED_ADC);

    snprintf(typename, sizeof(typename), "aspeed.fmc-%s", socname);
    sysbus_init_child_obj(obj, "fmc", OBJECT(&s->fmc), sizeof(s->fmc),
                          typename);
    object_property_add_alias(obj, "num-cs", OBJECT(&s->fmc), "num-cs",
                              &error_abort);
    object_property_add_alias(obj, "dram", OBJECT(&s->fmc), "dram",
                              &error_abort);

    for (i = 0; i < sc->info->spis_num; i++) {
        snprintf(typename, sizeof(typename), "aspeed.spi%d-%s", i + 1, socname);
        sysbus_init_child_obj(obj, "spi[*]", OBJECT(&s->spi[i]),
                              sizeof(s->spi[i]), typename);
    }

    sysbus_init_child_obj(obj, "sdmc", OBJECT(&s->sdmc), sizeof(s->sdmc),
                          TYPE_ASPEED_SDMC);
    qdev_prop_set_uint32(DEVICE(&s->sdmc), "silicon-rev",
                         sc->info->silicon_rev);
    object_property_add_alias(obj, "ram-size", OBJECT(&s->sdmc),
                              "ram-size", &error_abort);
    object_property_add_alias(obj, "max-ram-size", OBJECT(&s->sdmc),
                              "max-ram-size", &error_abort);

    for (i = 0; i < sc->info->wdts_num; i++) {
        sysbus_init_child_obj(obj, "wdt[*]", OBJECT(&s->wdt[i]),
                              sizeof(s->wdt[i]), TYPE_ASPEED_WDT);
        qdev_prop_set_uint32(DEVICE(&s->wdt[i]), "silicon-rev",
                                    sc->info->silicon_rev);
        object_property_add_const_link(OBJECT(&s->wdt[i]), "scu",
                                       OBJECT(&s->scu), &error_abort);
    }

    for (i = 0; i < sc->info->macs_num; i++) {
        sysbus_init_child_obj(obj, "ftgmac100[*]", OBJECT(&s->ftgmac100[i]),
                              sizeof(s->ftgmac100[i]), TYPE_FTGMAC100);

        if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
            sysbus_init_child_obj(obj, "mii[*]", &s->mii[i], sizeof(s->mii[i]),
                                  TYPE_ASPEED_MII);
            object_property_add_const_link(OBJECT(&s->mii[i]), "nic",
                                           OBJECT(&s->ftgmac100[i]),
                                           &error_abort);
        }
    }

    sysbus_init_child_obj(obj, "xdma", OBJECT(&s->xdma), sizeof(s->xdma),
                          TYPE_ASPEED_XDMA);

    snprintf(typename, sizeof(typename), "aspeed.gpio-%s", socname);
    sysbus_init_child_obj(obj, "gpio", OBJECT(&s->gpio), sizeof(s->gpio),
                          typename);

    sysbus_init_child_obj(obj, "sdhci", OBJECT(&s->sdhci), sizeof(s->sdhci),
                          TYPE_ASPEED_SDHCI);

    for (i = 0; i < ASPEED_SDHCI_NUM_SLOTS; ++i) {
        sysbus_init_child_obj(obj, "sdhci_slot[*]",
                              OBJECT(&s->sdhci.slots[i].sdhci),
                              sizeof(s->sdhci.slots[i].sdhci),
                              TYPE_SYSBUS_SDHCI);
    }

    sysbus_init_child_obj(obj, "ibt", OBJECT(&s->ibt), sizeof(s->ibt),
                          TYPE_ASPEED_IBT);

    sysbus_init_child_obj(obj, "pwm", OBJECT(&s->pwm), sizeof(s->pwm),
                          TYPE_ASPEED_PWM);

    sysbus_init_child_obj(obj, "lpc", OBJECT(&s->lpc), sizeof(s->lpc),
                           TYPE_ASPEED_LPC);

    if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
        sysbus_init_child_obj(obj, "fsi[*]", OBJECT(&s->fsi[0]),
                              sizeof(s->fsi[0]), TYPE_ASPEED_FSI);
    }
}

/*
 * ASPEED ast2600 has 0xf as cluster ID
 *
 * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0388e/CIHEBGFG.html
 */
static uint64_t aspeed_calc_affinity(int cpu)
{
    return (0xf << ARM_AFF1_SHIFT) | cpu;
}

static void aspeed_soc_realize(DeviceState *dev, Error **errp)
{
    int i, offset;
    AspeedSoCState *s = ASPEED_SOC(dev);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    Error *err = NULL, *local_err = NULL;

    /* IO space */
    create_unimplemented_device("aspeed_soc.io", sc->info->memmap[ASPEED_IOMEM],
                                ASPEED_SOC_IOMEM_SIZE);

    /* Video engine stub */
    create_unimplemented_device("aspeed.video", sc->info->memmap[ASPEED_VIDEO],
                                0x1000);

    if (s->num_cpus > sc->info->num_cpus) {
        warn_report("%s: invalid number of CPUs %d, using default %d",
                    sc->info->name, s->num_cpus, sc->info->num_cpus);
        s->num_cpus = sc->info->num_cpus;
    }

    /* CPU */
    for (i = 0; i < s->num_cpus; i++) {
        if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
            object_property_set_int(OBJECT(&s->cpu[i]), QEMU_PSCI_CONDUIT_SMC,
                                    "psci-conduit", &error_abort);
            if (s->num_cpus > 1) {
                object_property_set_int(OBJECT(&s->cpu[i]), ASPEED_A7MPCORE_ADDR,
                                        "reset-cbar", &error_abort);
            }
            object_property_set_int(OBJECT(&s->cpu[i]), aspeed_calc_affinity(i),
                                    "mp-affinity", &error_abort);

            /*
             * TODO: the secondary CPUs are started and a boot helper
             * is needed when using -kernel
             */
        }

        object_property_set_bool(OBJECT(&s->cpu[i]), true, "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
    }

    /* A7MPCORE */
    if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
        qemu_irq irq;

        object_property_set_int(OBJECT(&s->a7mpcore), s->num_cpus, "num-cpu",
                                &error_abort);
        object_property_set_int(OBJECT(&s->a7mpcore),
                                ASPEED_SOC_AST2600_MAX_IRQ + GIC_INTERNAL,
                                "num-irq", &error_abort);

        object_property_set_bool(OBJECT(&s->a7mpcore), true, "realized",
                                 &error_abort);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->a7mpcore), 0, ASPEED_A7MPCORE_ADDR);

        for (i = 0; i < s->num_cpus; i++) {
            SysBusDevice *sbd = SYS_BUS_DEVICE(&s->a7mpcore);
            DeviceState  *d   = DEVICE(qemu_get_cpu(i));

            irq = qdev_get_gpio_in(d, ARM_CPU_IRQ);
            sysbus_connect_irq(sbd, i, irq);
            irq = qdev_get_gpio_in(d, ARM_CPU_FIQ);
            sysbus_connect_irq(sbd, i + s->num_cpus, irq);
            irq = qdev_get_gpio_in(d, ARM_CPU_VIRQ);
            sysbus_connect_irq(sbd, i + 2 * s->num_cpus, irq);
            irq = qdev_get_gpio_in(d, ARM_CPU_VFIQ);
            sysbus_connect_irq(sbd, i + 3 * s->num_cpus, irq);
        }
    }

    /* SRAM */
    memory_region_init_ram(&s->sram, OBJECT(dev), "aspeed.sram",
                           sc->info->sram_size, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(get_system_memory(),
                                sc->info->memmap[ASPEED_SRAM], &s->sram);

    /* SCU */
    object_property_set_bool(OBJECT(&s->scu), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->scu), 0, sc->info->memmap[ASPEED_SCU]);

    /* VIC */
    if (!ASPEED_IS_AST2600(sc->info->silicon_rev)) {
        object_property_set_bool(OBJECT(&s->vic), true, "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->vic), 0,
                        sc->info->memmap[ASPEED_VIC]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->vic), 0,
                           qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->vic), 1,
                           qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_FIQ));
    }

    /* RTC */
    object_property_set_bool(OBJECT(&s->rtc), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0, sc->info->memmap[ASPEED_RTC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0,
                       aspeed_soc_get_irq(s, ASPEED_RTC));

    /* Timer */
    object_property_set_bool(OBJECT(&s->timerctrl), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timerctrl), 0,
                    sc->info->memmap[ASPEED_TIMER1]);
    for (i = 0; i < ASPEED_TIMER_NR_TIMERS; i++) {
        qemu_irq irq = aspeed_soc_get_irq(s, ASPEED_TIMER1 + i);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timerctrl), i, irq);
    }

    /* ADC */
    object_property_set_bool(OBJECT(&s->adc), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adc), 0, sc->info->memmap[ASPEED_ADC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc), 0,
                       aspeed_soc_get_irq(s, ASPEED_ADC));

    /* UART - attach an 8250 to the IO space as our UART5 */
    if (serial_hd(0)) {
        qemu_irq uart5 = aspeed_soc_get_irq(s, ASPEED_UART5);
        serial_mm_init(get_system_memory(), sc->info->memmap[ASPEED_UART5], 2,
                       uart5, 38400, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    }

    /* VUART */
    if (serial_hd(1)) {
        qemu_irq vuart = qdev_get_gpio_in(DEVICE(&s->vic), 8);
        serial_mm_init(get_system_memory(), sc->info->memmap[ASPEED_VUART], 2,
                       vuart, 38400, serial_hd(1), DEVICE_LITTLE_ENDIAN);
    }

    /* UART1 */
    if (serial_hd(2)) {
        qemu_irq uart1 = aspeed_soc_get_irq(s, ASPEED_UART1);
        serial_mm_init(get_system_memory(), sc->info->memmap[ASPEED_UART1], 2,
                       uart1, 38400, serial_hd(2), DEVICE_LITTLE_ENDIAN);
    }

    /* I2C */
    object_property_set_bool(OBJECT(&s->i2c), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c), 0, sc->info->memmap[ASPEED_I2C]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c), 0,
                       aspeed_soc_get_irq(s, ASPEED_I2C));

    /* FMC, The number of CS is set at the board level */
    object_property_set_int(OBJECT(&s->fmc), sc->info->memmap[ASPEED_SDRAM],
                            "sdram-base", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    object_property_set_bool(OBJECT(&s->fmc), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->fmc), 0, sc->info->memmap[ASPEED_FMC]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->fmc), 1,
                    s->fmc.ctrl->flash_window_base);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->fmc), 0,
                       aspeed_soc_get_irq(s, ASPEED_FMC));

    /* SPI */
    for (i = 0; i < sc->info->spis_num; i++) {
        object_property_set_int(OBJECT(&s->spi[i]), 1, "num-cs", &err);
        object_property_set_bool(OBJECT(&s->spi[i]), true, "realized",
                                 &local_err);
        error_propagate(&err, local_err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi[i]), 0,
                        sc->info->memmap[ASPEED_SPI1 + i]);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi[i]), 1,
                        s->spi[i].ctrl->flash_window_base);
    }

    /* SDMC - SDRAM Memory Controller */
    object_property_set_bool(OBJECT(&s->sdmc), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdmc), 0, sc->info->memmap[ASPEED_SDMC]);

    /* Watch dog */
    offset = ASPEED_IS_AST2600(sc->info->silicon_rev) ? 0x40 : 0x20;
    for (i = 0; i < sc->info->wdts_num; i++) {
        object_property_set_bool(OBJECT(&s->wdt[i]), true, "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->wdt[i]), 0,
                        sc->info->memmap[ASPEED_WDT] + i * offset);
    }

    /* Net */
    for (i = 0; i < nb_nics; i++) {
        qdev_set_nic_properties(DEVICE(&s->ftgmac100[i]), &nd_table[i]);
        object_property_set_bool(OBJECT(&s->ftgmac100[i]), true, "aspeed",
                                 &err);
        object_property_set_bool(OBJECT(&s->ftgmac100[i]), true, "realized",
                                 &local_err);
        error_propagate(&err, local_err);
        if (err) {
            error_propagate(errp, err);
           return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->ftgmac100[i]), 0,
                        sc->info->memmap[ASPEED_ETH1 + i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ftgmac100[i]), 0,
                           aspeed_soc_get_irq(s, ASPEED_ETH1 + i));


        if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
            object_property_set_bool(OBJECT(&s->mii[i]), true, "realized", &err);
            if (err) {
                error_propagate(errp, err);
                return;
            }

            sysbus_mmio_map(SYS_BUS_DEVICE(&s->mii[i]), 0,
                            sc->info->memmap[ASPEED_MII1 + i]);
        }
    }

    /* XDMA */
    object_property_set_bool(OBJECT(&s->xdma), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->xdma), 0,
                    sc->info->memmap[ASPEED_XDMA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->xdma), 0,
                       aspeed_soc_get_irq(s, ASPEED_XDMA));

    /* GPIO */
    object_property_set_bool(OBJECT(&s->gpio), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, sc->info->memmap[ASPEED_GPIO]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), 0,
                       aspeed_soc_get_irq(s, ASPEED_GPIO));

    /* SD/SDIO */
    for (i = 0; i < ASPEED_SDHCI_NUM_SLOTS; i++) {
        hwaddr hci_addr = sc->info->memmap[ASPEED_SDHCI] + (0x100 * (i + 1));
        DriveInfo *di;
        BlockBackend *blk;
        DeviceState *card;

        /*
         * Compatible with:
         * - SD Host Controller Specification Version 2.0
         * - SDIO Specification Version 2.0
         * - MMC Specification Version 4.3
         */
        object_property_set_int(OBJECT(&s->sdhci.slots[i].sdhci), 2,
                                "sd-spec-version", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }

        object_property_set_uint(OBJECT(&s->sdhci.slots[i].sdhci),
                                 ASPEED_SDHCI_CAPABILITIES, "capareg", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }

        object_property_set_bool(OBJECT(&s->sdhci.slots[i].sdhci), true,
                                 "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci.slots[i].sdhci), 0, hci_addr);

        di = drive_get_next(IF_SD);
        blk = di ? blk_by_legacy_dinfo(di) : NULL;
        card = qdev_create(qdev_get_child_bus(DEVICE(&s->sdhci.slots[i].sdhci),
                                              "sd-bus"), TYPE_SD_CARD);
        qdev_prop_set_drive(card, "drive", blk, &error_fatal);
        object_property_set_bool(OBJECT(card), true, "realized", &error_fatal);
    }

    object_property_set_bool(OBJECT(&s->sdhci), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci), 0,
                    sc->info->memmap[ASPEED_SDHCI]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdhci), 0,
                       aspeed_soc_get_irq(s, ASPEED_SDHCI));

    /* iBT */
    object_property_set_bool(OBJECT(&s->ibt), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ibt), 0, sc->info->memmap[ASPEED_IBT]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->ibt), 0,
                       aspeed_soc_get_irq(s, ASPEED_LPC));

    /* PWM */
    object_property_set_bool(OBJECT(&s->pwm), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pwm), 0, sc->info->memmap[ASPEED_PWM]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pwm), 0,
                       aspeed_soc_get_irq(s, ASPEED_PWM));

    /* LPC */
    object_property_set_bool(OBJECT(&s->lpc), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->lpc), 0, sc->info->memmap[ASPEED_LPC]);
    /* LPC IRQ in use by the iBT sub controller */

    /* FSI */
    if (ASPEED_IS_AST2600(sc->info->silicon_rev)) {
        object_property_set_bool(OBJECT(&s->fsi[0]), true, "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->fsi[0]), 0,
                        sc->info->memmap[ASPEED_FSI1]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->fsi[0]), 0,
                           aspeed_soc_get_irq(s, ASPEED_FSI1));
    }
}
static Property aspeed_soc_properties[] = {
    DEFINE_PROP_UINT32("num-cpus", AspeedSoCState, num_cpus, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void aspeed_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    AspeedSoCClass *sc = ASPEED_SOC_CLASS(oc);

    sc->info = (AspeedSoCInfo *) data;
    dc->realize = aspeed_soc_realize;
    /* Reason: Uses serial_hds and nd_table in realize() directly */
    dc->user_creatable = false;
    dc->props = aspeed_soc_properties;
}

static const TypeInfo aspeed_soc_type_info = {
    .name           = TYPE_ASPEED_SOC,
    .parent         = TYPE_DEVICE,
    .instance_init  = aspeed_soc_init,
    .instance_size  = sizeof(AspeedSoCState),
    .class_size     = sizeof(AspeedSoCClass),
    .abstract       = true,
};

static void aspeed_soc_register_types(void)
{
    int i;

    type_register_static(&aspeed_soc_type_info);
    for (i = 0; i < ARRAY_SIZE(aspeed_socs); ++i) {
        TypeInfo ti = {
            .name       = aspeed_socs[i].name,
            .parent     = TYPE_ASPEED_SOC,
            .class_init = aspeed_soc_class_init,
            .class_data = (void *) &aspeed_socs[i],
        };
        type_register(&ti);
    }
}

type_init(aspeed_soc_register_types)
