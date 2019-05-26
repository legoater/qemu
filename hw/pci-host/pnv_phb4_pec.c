/*
 * QEMU PowerPC PowerNV PHB4 model
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "target/ppc/cpu.h"
#include "hw/ppc/fdt.h"
#include "hw/pci-host/pnv_phb4_regs.h"
#include "hw/pci-host/pnv_phb4.h"
#include "hw/ppc/pnv_xscom.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "hw/ppc/pnv.h"

#include <libfdt.h>

static uint64_t pnv_pec_nest_xscom_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    PnvPhb4PecState *pec = PNV_PHB4_PEC(opaque);
    uint32_t offset = addr >> 3;

    /* TODO: add list of allowed registers and error out if not */
    return pec->nest_regs[offset];
}

static uint64_t pnv_pec_pci_xscom_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    PnvPhb4PecState *pec = PNV_PHB4_PEC(opaque);
    uint32_t offset = addr >> 3;

    /* TODO: add list of allowed registers and error out if not */

    return pec->pci_regs[offset];
}


static uint64_t pnv_pec_stk_nest_xscom_read(void *opaque, hwaddr addr,
                                            unsigned size)
{
    PnvPhb4PecStack *stack = PNV_PHB4_PEC_STACK(opaque);
    uint32_t offset = addr >> 3;

    /* TODO: add list of allowed registers and error out if not */
    return stack->nest_regs[offset];
}

static uint64_t pnv_pec_stk_pci_xscom_read(void *opaque, hwaddr addr,
                                           unsigned size)
{
    PnvPhb4PecStack *stack = PNV_PHB4_PEC_STACK(opaque);
    uint32_t offset = addr >> 3;

    /* TODO: add list of allowed registers and error out if not */

    return stack->pci_regs[offset];
}

static void pnv_pec_stk_update_map(PnvPhb4PecStack *stack)
{
    uint64_t bar_en = stack->nest_regs[PEC_NEST_STK_BAR_EN];
    uint64_t bar, mask, size;
    char name[64];

    /*
     * NOTE: This will really not work well if those are remapped
     * after the PHB has created its sub regions. We could do better
     * if we had a way to resize regions but we don't really care
     * that much in practice as the stuff below really only happens
     * once early during boot
     */

    /* Handle unmaps */
    if (stack->mmio0_mapped && !(bar_en & PEC_NEST_STK_BAR_EN_MMIO0)) {
        memory_region_del_subregion(get_system_memory(), &stack->mmbar0);
        stack->mmio0_mapped = false;
    }
    if (stack->mmio1_mapped && !(bar_en & PEC_NEST_STK_BAR_EN_MMIO1)) {
        memory_region_del_subregion(get_system_memory(), &stack->mmbar1);
        stack->mmio1_mapped = false;
    }
    if (stack->phb_mapped && !(bar_en & PEC_NEST_STK_BAR_EN_PHB)) {
        memory_region_del_subregion(get_system_memory(), &stack->phbbar);
        stack->phb_mapped = false;
    }
    if (stack->int_mapped && !(bar_en & PEC_NEST_STK_BAR_EN_INT)) {
        memory_region_del_subregion(get_system_memory(), &stack->intbar);
        stack->int_mapped = false;
    }

    /* Update PHB */
    pnv_phb4_update_regions(stack->phb);

    /* Handle maps */
    if (!stack->mmio0_mapped && (bar_en & PEC_NEST_STK_BAR_EN_MMIO0)) {
        bar = stack->nest_regs[PEC_NEST_STK_MMIO_BAR0] >> 8;
        mask = stack->nest_regs[PEC_NEST_STK_MMIO_BAR0_MASK];
        size = ((~mask) >> 8) + 1;
        snprintf(name, sizeof(name), "pec-%d.%d-stack-%d-mmio0",
                 stack->pec->chip_id, stack->pec->index, stack->stack_no);
        memory_region_init(&stack->mmbar0, OBJECT(stack), name, size);
        memory_region_add_subregion(get_system_memory(), bar, &stack->mmbar0);
        stack->mmio0_mapped = true;
        stack->mmio0_base = bar;
        stack->mmio0_size = size;
    }
    if (!stack->mmio1_mapped && (bar_en & PEC_NEST_STK_BAR_EN_MMIO1)) {
        bar = stack->nest_regs[PEC_NEST_STK_MMIO_BAR1] >> 8;
        mask = stack->nest_regs[PEC_NEST_STK_MMIO_BAR1_MASK];
        size = ((~mask) >> 8) + 1;
        snprintf(name, sizeof(name), "pec-%d.%d-stack-%d-mmio1",
                 stack->pec->chip_id, stack->pec->index, stack->stack_no);
        memory_region_init(&stack->mmbar1, OBJECT(stack), name, size);
        memory_region_add_subregion(get_system_memory(), bar, &stack->mmbar1);
        stack->mmio1_mapped = true;
        stack->mmio1_base = bar;
        stack->mmio1_size = size;
    }
    if (!stack->phb_mapped && (bar_en & PEC_NEST_STK_BAR_EN_PHB)) {
        bar = stack->nest_regs[PEC_NEST_STK_PHB_REGS_BAR] >> 8;
        size = PNV_PHB4_NUM_REGS << 3;
        snprintf(name, sizeof(name), "pec-%d.%d-stack-%d-phb",
                 stack->pec->chip_id, stack->pec->index, stack->stack_no);
        memory_region_init(&stack->phbbar, OBJECT(stack), name, size);
        memory_region_add_subregion(get_system_memory(), bar, &stack->phbbar);
        stack->phb_mapped = true;
    }
    if (!stack->int_mapped && (bar_en & PEC_NEST_STK_BAR_EN_INT)) {
        bar = stack->nest_regs[PEC_NEST_STK_INT_BAR] >> 8;
        size = PNV_PHB4_MAX_INTs << 16;
        snprintf(name, sizeof(name), "pec-%d.%d-stack-%d-int",
                 stack->pec->chip_id, stack->pec->index, stack->stack_no);
        memory_region_init(&stack->intbar, OBJECT(stack), name, size);
        memory_region_add_subregion(get_system_memory(), bar, &stack->intbar);
        stack->int_mapped = true;
    }

    /* Update PHB */
    pnv_phb4_update_regions(stack->phb);
}

static void pnv_pec_nest_xscom_write(void *opaque, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    PnvPhb4PecState *pec = PNV_PHB4_PEC(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PEC_NEST_PBCQ_HW_CONFIG:
    case PEC_NEST_DROP_PRIO_CTRL:
    case PEC_NEST_PBCQ_ERR_INJECT:
    case PEC_NEST_PCI_NEST_CLK_TRACE_CTL:
    case PEC_NEST_PBCQ_PMON_CTRL:
    case PEC_NEST_PBCQ_PBUS_ADDR_EXT:
    case PEC_NEST_PBCQ_PRED_VEC_TIMEOUT:
    case PEC_NEST_CAPP_CTRL:
    case PEC_NEST_PBCQ_READ_STK_OVR:
    case PEC_NEST_PBCQ_WRITE_STK_OVR:
    case PEC_NEST_PBCQ_STORE_STK_OVR:
    case PEC_NEST_PBCQ_RETRY_BKOFF_CTRL:
        pec->nest_regs[reg] = val;
        break;
    }

    /* XXX TODO: Set error flags on other regs */
}

static void pnv_pec_pci_xscom_write(void *opaque, hwaddr addr,
                                    uint64_t val, unsigned size)
{
    PnvPhb4PecState *pec = PNV_PHB4_PEC(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PEC_PCI_PBAIB_HW_CONFIG:
    case PEC_PCI_PBAIB_READ_STK_OVR:
        pec->pci_regs[reg] = val;
        break;
    }

    /* XXX TODO: Set error flags on other regs */
}

static void pnv_pec_stk_nest_xscom_write(void *opaque, hwaddr addr,
                                         uint64_t val, unsigned size)
{
    PnvPhb4PecStack *stack = PNV_PHB4_PEC_STACK(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PEC_NEST_STK_PCI_NEST_FIR:
        stack->nest_regs[PEC_NEST_STK_PCI_NEST_FIR] = val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_CLR:
        stack->nest_regs[PEC_NEST_STK_PCI_NEST_FIR] &= val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_SET:
        stack->nest_regs[PEC_NEST_STK_PCI_NEST_FIR] |= val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_MSK:
        stack->nest_regs[PEC_NEST_STK_PCI_NEST_FIR_MSK] = val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_MSKC:
        stack->nest_regs[PEC_NEST_STK_PCI_NEST_FIR_MSK] &= val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_MSKS:
        stack->nest_regs[PEC_NEST_STK_PCI_NEST_FIR_MSK] |= val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_ACT0:
    case PEC_NEST_STK_PCI_NEST_FIR_ACT1:
        stack->nest_regs[reg] = val;
        break;
    case PEC_NEST_STK_PCI_NEST_FIR_WOF:
        stack->nest_regs[reg] = 0;
        break;
    case PEC_NEST_STK_ERR_REPORT_0:
    case PEC_NEST_STK_ERR_REPORT_1:
    case PEC_NEST_STK_PBCQ_GNRL_STATUS:
        /* Flag error ? */
        break;
    case PEC_NEST_STK_PBCQ_MODE:
        stack->nest_regs[reg] = val & 0xff00000000000000ull;
        break;
    case PEC_NEST_STK_MMIO_BAR0:
    case PEC_NEST_STK_MMIO_BAR0_MASK:
    case PEC_NEST_STK_MMIO_BAR1:
    case PEC_NEST_STK_MMIO_BAR1_MASK:
        if (stack->nest_regs[PEC_NEST_STK_BAR_EN] &
            (PEC_NEST_STK_BAR_EN_MMIO0 |
             PEC_NEST_STK_BAR_EN_MMIO1)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                           "PHB4-PEC: Changing enabled BAR unsupported\n");
        }
        stack->nest_regs[reg] = val & 0xffffffffff000000ull;
        break;
    case PEC_NEST_STK_PHB_REGS_BAR:
        if (stack->nest_regs[PEC_NEST_STK_BAR_EN] & PEC_NEST_STK_BAR_EN_PHB) {
            qemu_log_mask(LOG_GUEST_ERROR,
                           "PHB4-PEC: Changing enabled BAR unsupported\n");
        }
        stack->nest_regs[reg] = val & 0xffffffffffc00000ull;
        break;
    case PEC_NEST_STK_INT_BAR:
        if (stack->nest_regs[PEC_NEST_STK_BAR_EN] & PEC_NEST_STK_BAR_EN_INT) {
            qemu_log_mask(LOG_GUEST_ERROR,
                           "PHB4-PEC: Changing enabled BAR unsupported\n");
        }
        stack->nest_regs[reg] = val & 0xfffffff000000000ull;
        break;
    case PEC_NEST_STK_BAR_EN:
        stack->nest_regs[reg] = val & 0xf000000000000000ull;
        pnv_pec_stk_update_map(stack);
        break;
    case PEC_NEST_STK_DATA_FRZ_TYPE:
    case PEC_NEST_STK_PBCQ_TUN_BAR:
        /* Not used for now */
        stack->nest_regs[reg] = val;
        break;
    }

    /* TODO: Error out on other regs for now ... */
}

static void pnv_pec_stk_pci_xscom_write(void *opaque, hwaddr addr,
                                        uint64_t val, unsigned size)
{
    PnvPhb4PecStack *stack = PNV_PHB4_PEC_STACK(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PEC_PCI_STK_PCI_FIR:
        stack->nest_regs[reg] = val;
        break;
    case PEC_PCI_STK_PCI_FIR_CLR:
        stack->nest_regs[PEC_PCI_STK_PCI_FIR] &= val;
        break;
    case PEC_PCI_STK_PCI_FIR_SET:
        stack->nest_regs[PEC_PCI_STK_PCI_FIR] |= val;
        break;
    case PEC_PCI_STK_PCI_FIR_MSK:
        stack->nest_regs[reg] = val;
        break;
    case PEC_PCI_STK_PCI_FIR_MSKC:
        stack->nest_regs[PEC_PCI_STK_PCI_FIR_MSK] &= val;
        break;
    case PEC_PCI_STK_PCI_FIR_MSKS:
        stack->nest_regs[PEC_PCI_STK_PCI_FIR_MSK] |= val;
        break;
    case PEC_PCI_STK_PCI_FIR_ACT0:
    case PEC_PCI_STK_PCI_FIR_ACT1:
        stack->nest_regs[reg] = val;
        break;
    case PEC_PCI_STK_PCI_FIR_WOF:
        stack->nest_regs[reg] = 0;
        break;
    case PEC_PCI_STK_ETU_RESET:
        stack->nest_regs[reg] = val & 0x8000000000000000ull;
        // TODO: Implement reset
        break;
    case PEC_PCI_STK_PBAIB_ERR_REPORT:
        break;
    case PEC_PCI_STK_PBAIB_TX_CMD_CRED:
    case PEC_PCI_STK_PBAIB_TX_DAT_CRED:
        stack->nest_regs[reg] = val;
        break;
    }

    /* XXX Don't error out on other regs for now ... */
}


static const MemoryRegionOps pnv_pec_nest_xscom_ops = {
    .read = pnv_pec_nest_xscom_read,
    .write = pnv_pec_nest_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const MemoryRegionOps pnv_pec_pci_xscom_ops = {
    .read = pnv_pec_pci_xscom_read,
    .write = pnv_pec_pci_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const MemoryRegionOps pnv_pec_stk_nest_xscom_ops = {
    .read = pnv_pec_stk_nest_xscom_read,
    .write = pnv_pec_stk_nest_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const MemoryRegionOps pnv_pec_stk_pci_xscom_ops = {
    .read = pnv_pec_stk_pci_xscom_read,
    .write = pnv_pec_stk_pci_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void pnv_pec_realize(DeviceState *dev, Error **errp)
{
    PnvPhb4PecState *pec = PNV_PHB4_PEC(dev);
    PnvMachineState *pnv = PNV_MACHINE(qdev_get_machine());
    uint32_t nbase = PNV9_XSCOM_PEC_NEST_BASE + 0x400 * pec->index;
    uint32_t pbase = PNV9_XSCOM_PEC_PCI_BASE + 0x1000000 * pec->index;
    char name[64];
    int i;

    switch(pec->index) {
    case 0:
        /* PEC 0 supports a single stack only */
        pec->num_stacks = 1;
        break;
    case 1:
        pec->num_stacks = 2;
        break;
    case 2:
        pec->num_stacks = 3;
        break;
    default:
        error_setg(errp, "invalid PEC index: %d", pec->index);
        return;
    }

    /* Can that be done using "link" properties ? */
    pec->chip = pnv_get_chip(pnv, pec->chip_id);
    if (!pec->chip) {
        error_setg(errp, "invalid chip id: %d", pec->chip_id);
        return;
    }

    /* Create stacks */
    for (i = 0; i < pec->num_stacks; i++) {
        Object *stk_obj = object_new(TYPE_PNV_PHB4_PEC_STACK);
        PnvPhb4PecStack *stack = PNV_PHB4_PEC_STACK(stk_obj);
        char stk_name[32];

        pec->stacks[i] = stack;
        snprintf(stk_name, sizeof(stk_name), "stack%d", i);
        object_property_add_child(OBJECT(pec), stk_name, stk_obj, errp);

        // XX Use properties ? */
        stack->stack_no = i;
        stack->pec = pec;

        object_property_set_bool(stk_obj, true, "realized", errp);

        /* XXX Move the stuff below inside the stack itself  ??? */

        /* Initialize the XSCOM regions for the stack registers */
        snprintf(name, sizeof(name), "xscom-pec-nest-%d.%d-stack-%d",
                 pec->chip_id, pec->index, i);
        pnv_xscom_region_init(&stack->nest_regs_mr, stk_obj,
                              &pnv_pec_stk_nest_xscom_ops, stack, name,
                              PHB4_PEC_NEST_STK_REGS_COUNT);
        pnv_xscom_add_subregion(pec->chip, nbase + 0x40 * (i + 1),
                                &stack->nest_regs_mr);

        snprintf(name, sizeof(name), "xscom-pec-pci-%d.%d-stack-%d",
                 pec->chip_id, pec->index, i);
        pnv_xscom_region_init(&stack->pci_regs_mr, stk_obj,
                              &pnv_pec_stk_pci_xscom_ops, stack, name,
                              PHB4_PEC_PCI_STK_REGS_COUNT);
        pnv_xscom_add_subregion(pec->chip, pbase + 0x40 * (i + 1),
                                &stack->pci_regs_mr);

        /* XXX Add the PHB pass-through region */
    }

    /* Initialize the XSCOM regions for the PEC registers */
    snprintf(name, sizeof(name), "xscom-pec-nest-%d.%d",
             pec->chip_id, pec->index);
    pnv_xscom_region_init(&pec->nest_regs_mr, OBJECT(dev),
                          &pnv_pec_nest_xscom_ops, pec, name,
                          PHB4_PEC_NEST_REGS_COUNT);
    pnv_xscom_add_subregion(pec->chip, nbase, &pec->nest_regs_mr);

    snprintf(name, sizeof(name), "xscom-pec-pci-%d.%d",
             pec->chip_id, pec->index);
    pnv_xscom_region_init(&pec->pci_regs_mr, OBJECT(dev),
                          &pnv_pec_pci_xscom_ops, pec, name,
                          PHB4_PEC_PCI_REGS_COUNT);
    pnv_xscom_add_subregion(pec->chip, pbase, &pec->pci_regs_mr);
}

static int pnv_pec_dt_xscom(PnvXScomInterface *dev, void *fdt,
                            int xscom_offset)
{
    const char compat[] = "ibm,power9-pbcq";
    PnvPhb4PecState *pec = PNV_PHB4_PEC(dev);
    uint32_t nbase = PNV9_XSCOM_PEC_NEST_BASE + 0x400 * pec->index;
    uint32_t pbase = PNV9_XSCOM_PEC_PCI_BASE + 0x1000000 * pec->index;
    int offset, i;
    char *name;
    uint32_t reg[] = {
        cpu_to_be32(nbase),
        cpu_to_be32(PNV9_XSCOM_PEC_NEST_SIZE),
        cpu_to_be32(pbase),
        cpu_to_be32(PNV9_XSCOM_PEC_PCI_SIZE),
    };

    name = g_strdup_printf("pbcq@%x", nbase);
    offset = fdt_add_subnode(fdt, xscom_offset, name);
    _FDT(offset);
    g_free(name);

    _FDT((fdt_setprop(fdt, offset, "reg", reg, sizeof(reg))));

    _FDT((fdt_setprop_cell(fdt, offset, "ibm,pec-index", pec->index)));
    _FDT((fdt_setprop_cell(fdt, offset, "#address-cells", 1)));
    _FDT((fdt_setprop_cell(fdt, offset, "#size-cells", 0)));
    _FDT((fdt_setprop(fdt, offset, "compatible", compat, sizeof(compat))));

    for (i = 0; i < pec->num_stacks; i++) {
        const char stk_compat[] = "ibm,power9-phb-stack";
        uint32_t phb_idx;
        int stk_offset;

        switch(pec->index) {
        case 0:
            phb_idx = i;
            break;
        case 1:
            phb_idx = 1 + i;
            break;
        case 2:
            phb_idx = 3 + i;
            break;
        default:
            /* Shouldn't be possible due to check in pnv_pec_realize() */
            return 0;
        }

        name = g_strdup_printf("stack@%x", i);
        stk_offset = fdt_add_subnode(fdt, offset, name);
        _FDT(stk_offset);
        g_free(name);
        _FDT((fdt_setprop(fdt, stk_offset, "compatible", stk_compat, sizeof(stk_compat))));
        _FDT((fdt_setprop_cell(fdt, stk_offset, "reg", i)));
        _FDT((fdt_setprop_cell(fdt, stk_offset, "ibm,phb-index", phb_idx)));
    }

    return 0;
}

static void pnv_phb4_parent_fixup(PnvPHB4 *phb, Object *parent, Error **errp)
{
    Object *obj = OBJECT(phb);
    char default_id[16];

    if (obj->parent == parent) {
        return;
    }

    object_ref(obj);
    if (obj->parent) {
        snprintf(default_id, sizeof(default_id), "phb[%d]", phb->phb_id);

        object_unparent(obj);
    }
    object_property_add_child(parent,
                              DEVICE(obj)->id ? DEVICE(obj)->id : default_id,
                              obj, errp);
    object_unref(obj);
}

void pnv_phb4_pec_attach(struct PnvPHB4 *phb, const MemoryRegionOps *xscom_ops, Error **errp)
{
    PnvMachineState *pnv = PNV_MACHINE(qdev_get_machine());
    PnvPhb4PecStack *stack = NULL;
    PnvPhb4PecState *pec;
    Error *local_err = NULL;
    int i, pec_id, stack_id;
    PnvChip *chip;
    Pnv9Chip *p9;
    uint32_t pbase;
    char name[64];

    /* Decode PHB ID */
    if (phb->phb_id == 0) {
        pec_id = 0;
        stack_id = 0;
    } else if (phb->phb_id < 3) {
        pec_id = 1;
        stack_id = phb->phb_id - 1;
    } else if (phb->phb_id < 6) {
        pec_id = 2;
        stack_id = phb->phb_id - 3;
    } else {
        error_setg(errp, "invalid PHB id: %d", phb->phb_id);
        return;
    }

    /* Can that be done using "link" properties ? */
    chip = pnv_get_chip(pnv, phb->chip_id);
    if (!chip) {
        error_setg(errp, "invalid chip id: %d", phb->chip_id);
        return;
    }
    p9 = PNV9_CHIP(chip);

    for (i = 0; i < PNV9_CHIP_MAX_PEC; i++) {
        pec = &p9->pecs[i];

        if (pec->index != pec_id) {
            continue;
        }

        /* Shouldn't be possible unless we have a bug above or in PEC creation */
        assert(stack_id < pec->num_stacks);

        stack = pec->stacks[stack_id];
        if (stack->phb != NULL) {
            error_setg(errp, "Duplicate PHB chip %d PHB %d",
                       phb->chip_id, phb->phb_id);
            return;
        }
        stack->phb = phb;
        phb->stack = stack;
        break;
    }

    /* Shouldn't be possible to not find one */
    if (!stack) {
        error_setg(errp, "can't find strack for phb id: %d", phb->phb_id);
        return;
    }

    /* Set the "big_phb" flag */
    phb->big_phb = phb->phb_id == 0 || phb->phb_id == 3;

    pnv_phb4_parent_fixup(phb, OBJECT(stack), &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
    }

    pbase = PNV9_XSCOM_PEC_PCI_BASE + 0x1000000 * pec->index + PEC_PCI_SCOM_STK0;
    snprintf(name, sizeof(name), "xscom-phb4-%d.%d",
             pec->chip_id, phb->phb_id);
    pnv_xscom_region_init(&stack->phb_regs_mr, OBJECT(phb),
                          xscom_ops, phb, name, 0x40);
    pnv_xscom_add_subregion(pec->chip, pbase + 0x40 * stack_id,
                            &stack->phb_regs_mr);
}

static Property pnv_pec_properties[] = {
        DEFINE_PROP_UINT32("index", PnvPhb4PecState, index, 0),
        DEFINE_PROP_UINT32("chip-id", PnvPhb4PecState, chip_id, 0),
        DEFINE_PROP_END_OF_LIST(),
};

static void pnv_pec_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PnvXScomInterfaceClass *xdc = PNV_XSCOM_INTERFACE_CLASS(klass);

    xdc->dt_xscom = pnv_pec_dt_xscom;

    dc->realize = pnv_pec_realize;
    dc->props = pnv_pec_properties;
}

static const TypeInfo pnv_pec_type_info = {
    .name          = TYPE_PNV_PHB4_PEC,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(PnvPhb4PecState),
    .class_init    = pnv_pec_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_PNV_XSCOM_INTERFACE },
        { }
    }
};

static const TypeInfo pnv_pec_stk_type_info = {
    .name          = TYPE_PNV_PHB4_PEC_STACK,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(PnvPhb4PecStack),
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_PNV_XSCOM_INTERFACE },
        { }
    }
};

static void pnv_pec_register_types(void)
{
    type_register_static(&pnv_pec_type_info);
    type_register_static(&pnv_pec_stk_type_info);
}

type_init(pnv_pec_register_types)
