/*
 * QEMU PowerPC Microwatt CPU Core model
 *
 * Copyright (c) 2021, IBM Corporation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "sysemu/reset.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "target/ppc/cpu.h"
#include "hw/ppc/ppc.h"
#include "hw/ppc/microwatt.h"
#include "hw/ppc/mw_core.h"
#include "hw/qdev-properties.h"
#include "helper_regs.h"

static const char *mw_core_cpu_typename(MwCore *mc)
{
    const char *core_type = object_class_get_name(object_get_class(OBJECT(mc)));
    int len = strlen(core_type) - strlen(MW_CORE_TYPE_SUFFIX);
    char *s = g_strdup_printf(POWERPC_CPU_TYPE_NAME("%.*s"), len, core_type);
    const char *cpu_type = object_class_get_name(object_class_by_name(s));
    g_free(s);
    return cpu_type;
}

static void mw_core_cpu_reset(MwCore *mc, PowerPCCPU *cpu)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    MwSoCState *soc = mc->soc;
    MwSoCClass *msc = MW_SOC_GET_CLASS(soc);

    cpu_reset(cs);

    /* Tune our boot state */
    env->gpr[3] = soc->boot_info.dt_base;
    env->nip = soc->boot_info.entry;

    env->msr |= 1ull << MSR_SF | 1ull << MSR_LE;

    /* HV mode is still required for Radix */
    env->msr |= 1ull << MSR_HV;

    /* and Little endian interrupts when under HV */
    env->spr[SPR_HID0] |= HID0_POWER9_HILE| HID0_POWER9_ATTN;

    /* Minimum LPCR for QEMU : Host Radix and Large Decrementer */
    env->spr[SPR_LPCR] = LPCR_HR | LPCR_LD;

    hreg_compute_hflags(env);

    msc->intc_reset(soc, cpu);
}

static void mw_core_cpu_realize(MwCore *mc, PowerPCCPU *cpu, Error **errp)
{
    CPUPPCState *env = &cpu->env;
    int core_pir;
    int thread_index = 0; /* TODO: TCG supports only one thread */
    ppc_spr_t *pir = &env->spr_cb[SPR_PIR];
    Error *local_err = NULL;
    MwSoCClass *msc = MW_SOC_GET_CLASS(mc->soc);

    if (!qdev_realize(DEVICE(cpu), NULL, errp)) {
        return;
    }

    msc->intc_create(mc->soc, cpu, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    core_pir = object_property_get_uint(OBJECT(mc), "pir", &error_abort);

    pir->default_value = core_pir + thread_index;

    cpu_ppc_tb_init(env, MW_TIMEBASE_FREQ);
}

static void mw_core_reset(void *dev)
{
    CPUCore *cc = CPU_CORE(dev);
    MwCore *mc = MW_CORE(dev);
    int i;

    for (i = 0; i < cc->nr_threads; i++) {
        mw_core_cpu_reset(mc, mc->threads[i]);
    }
}

static void mw_core_realize(DeviceState *dev, Error **errp)
{
    MwCore *mc = MW_CORE(OBJECT(dev));
    CPUCore *cc = CPU_CORE(OBJECT(dev));
    const char *typename = mw_core_cpu_typename(mc);
    Error *local_err = NULL;
    void *obj;
    int i, j;

    if (!mc->soc) {
        error_setg(errp, TYPE_MW_ICP ": 'soc' link not set");
        return;
    }

    mc->threads = g_new(PowerPCCPU *, cc->nr_threads);
    for (i = 0; i < cc->nr_threads; i++) {
        PowerPCCPU *cpu;
        g_autofree char *name = g_strdup_printf("thread[%d]", i);

        obj = object_new(typename);
        cpu = POWERPC_CPU(obj);

        mc->threads[i] = POWERPC_CPU(obj);

        object_property_add_child(OBJECT(mc), name, obj);

        cpu->machine_data = g_new0(MwCPUState, 1);

        object_unref(obj);
    }

    for (j = 0; j < cc->nr_threads; j++) {
        mw_core_cpu_realize(mc, mc->threads[j], &local_err);
        if (local_err) {
            goto err;
        }
    }

    qemu_register_reset(mw_core_reset, mc);
    return;

err:
    while (--i >= 0) {
        obj = OBJECT(mc->threads[i]);
        object_unparent(obj);
    }
    g_free(mc->threads);
    error_propagate(errp, local_err);
}

static void mw_core_cpu_unrealize(MwCore *mc, PowerPCCPU *cpu)
{
    MwCPUState *mw_cpu = mw_cpu_state(cpu);
    MwSoCClass *msc = MW_SOC_GET_CLASS(mc->soc);

    msc->intc_destroy(mc->soc, cpu);
    cpu_remove_sync(CPU(cpu));
    cpu->machine_data = NULL;
    g_free(mw_cpu);
    object_unparent(OBJECT(cpu));
}

static void mw_core_unrealize(DeviceState *dev)
{
    MwCore *mc = MW_CORE(dev);
    CPUCore *cc = CPU_CORE(dev);
    int i;

    qemu_unregister_reset(mw_core_reset, mc);

    for (i = 0; i < cc->nr_threads; i++) {
        mw_core_cpu_unrealize(mc, mc->threads[i]);
    }
    g_free(mc->threads);
}

static Property mw_core_properties[] = {
    DEFINE_PROP_UINT32("pir", MwCore, pir, 0),
    DEFINE_PROP_LINK("soc", MwCore, soc, TYPE_MW_SOC, MwSoCState *),
    DEFINE_PROP_END_OF_LIST(),
};

static void mw_core_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mw_core_realize;
    dc->unrealize = mw_core_unrealize;
    device_class_set_props(dc, mw_core_properties);
    dc->user_creatable = false;
}

static const TypeInfo mw_core_infos[] = {
    {
        .name = MW_CORE_TYPE_NAME("power9mw_v1.0"),
        .parent = TYPE_MW_CORE,
    },
    {
        .name           = TYPE_MW_CORE,
        .parent         = TYPE_CPU_CORE,
        .instance_size  = sizeof(MwCore),
        .class_init     = mw_core_class_init,
        .abstract       = true,
    },
};

DEFINE_TYPES(mw_core_infos)
