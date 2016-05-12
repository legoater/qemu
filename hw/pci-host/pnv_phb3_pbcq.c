/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright IBM Corp. 2014
 */
#include "qemu/osdep.h"
#include "hw/pci-host/pnv_phb3.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"

#include <libfdt.h>

static bool pnv_pbcq_xscom_read(XScomDevice *dev, uint32_t range,
                                uint32_t offset, uint64_t *out_val)
{
    PnvPBCQState *pbcq = PNV_PBCQ(dev);

    switch(range) {
    case 0:
        *out_val = pbcq->nest_regs[offset];
        break;
    case 1:
        *out_val = pbcq->pci_regs[offset];
        break;
    case 2:
        if (offset == PBCQ_SPCI_ASB_DATA) {
            if (!pbcq->phb) {
                *out_val = ~0ull;
                break;
            }
            *out_val = pnv_phb3_reg_read(pbcq->phb,
                                     pbcq->spci_regs[PBCQ_SPCI_ASB_ADDR], 8);
            break;
        }
        *out_val = pbcq->spci_regs[offset];
        break;
    default:
            return false;
    }
    return true;
}

static void pnv_pbcq_update_map(PnvPBCQState *pbcq)
{
    uint64_t bar_en = pbcq->nest_regs[PBCQ_NEST_BAR_EN];
    uint64_t bar, mask, size;

    /*
     * NOTE: This will really not work well if those are remapped
     * after the PHB has created its sub regions. We could do better
     * if we had a way to resize regions but we don't really care
     * that much in practice as the stuff below really only happens
     * once early during boot
     */

    /* Handle unmaps */
    if (pbcq->mmio0_mapped && !(bar_en & PBCQ_NEST_BAR_EN_MMIO0)) {
        memory_region_del_subregion(get_system_memory(), &pbcq->mmbar0);
        pbcq->mmio0_mapped = false;
    }
    if (pbcq->mmio1_mapped && !(bar_en & PBCQ_NEST_BAR_EN_MMIO1)) {
        memory_region_del_subregion(get_system_memory(), &pbcq->mmbar1);
        pbcq->mmio1_mapped = false;
    }
    if (pbcq->phb_mapped && !(bar_en & PBCQ_NEST_BAR_EN_PHB)) {
        memory_region_del_subregion(get_system_memory(), &pbcq->phbbar);
        pbcq->phb_mapped = false;
    }

    /* Update PHB if it exists */
    if (pbcq->phb) {
        pnv_phb3_update_regions(pbcq->phb);
    }

    /* Handle maps */
    if (!pbcq->mmio0_mapped && (bar_en & PBCQ_NEST_BAR_EN_MMIO0)) {
        bar = pbcq->nest_regs[PBCQ_NEST_MMIO_BAR0] >> 14;
        mask = pbcq->nest_regs[PBCQ_NEST_MMIO_MASK0];
        size = ((~mask) >> 14) + 1;
        memory_region_init(&pbcq->mmbar0, OBJECT(pbcq), "pbcq-mmio0", size);
        memory_region_add_subregion(get_system_memory(), bar, &pbcq->mmbar0);
        pbcq->mmio0_mapped = true;
        pbcq->mmio0_base = bar;
        pbcq->mmio0_size = size;
    }
    if (!pbcq->mmio1_mapped && (bar_en & PBCQ_NEST_BAR_EN_MMIO1)) {
        bar = pbcq->nest_regs[PBCQ_NEST_MMIO_BAR1] >> 14;
        mask = pbcq->nest_regs[PBCQ_NEST_MMIO_MASK1];
        size = ((~mask) >> 14) + 1;
        memory_region_init(&pbcq->mmbar1, OBJECT(pbcq), "pbcq-mmio1", size);
        memory_region_add_subregion(get_system_memory(), bar, &pbcq->mmbar1);
        pbcq->mmio1_mapped = true;
        pbcq->mmio1_base = bar;
        pbcq->mmio1_size = size;
    }
    if (!pbcq->phb_mapped && (bar_en & PBCQ_NEST_BAR_EN_PHB)) {
        bar = pbcq->nest_regs[PBCQ_NEST_PHB_BAR] >> 14;
        size = 0x1000;
        memory_region_init(&pbcq->phbbar, OBJECT(pbcq), "pbcq-phb", size);
        memory_region_add_subregion(get_system_memory(), bar, &pbcq->phbbar);
        pbcq->phb_mapped = true;
    }

    /* Update PHB if it exists */
    if (pbcq->phb) {
        pnv_phb3_update_regions(pbcq->phb);
    }
}

static bool pnv_pbcq_xnest_write(PnvPBCQState *pbcq, uint32_t reg, uint64_t val)
{
    switch(reg) {
    case PBCQ_NEST_MMIO_BAR0:
    case PBCQ_NEST_MMIO_BAR1:
    case PBCQ_NEST_MMIO_MASK0:
    case PBCQ_NEST_MMIO_MASK1:
        if (pbcq->nest_regs[PBCQ_NEST_BAR_EN] &
            (PBCQ_NEST_BAR_EN_MMIO0 |
             PBCQ_NEST_BAR_EN_MMIO1)) {
            printf("WARNING: PH3: Changing enabled BAR unsupported\n");
        }
        pbcq->nest_regs[reg] = val & 0xffffffffc0000000ull;
        return true;
    case PBCQ_NEST_PHB_BAR:
        if (pbcq->nest_regs[PBCQ_NEST_BAR_EN] & PBCQ_NEST_BAR_EN_PHB) {
            printf("WARNING: PH3: Changing enabled BAR unsupported\n");
        }
        pbcq->nest_regs[reg] = val & 0xfffffffffc000000ull;
        return true;
    case PBCQ_NEST_BAR_EN:
        pbcq->nest_regs[reg] = val & 0xf800000000000000ull;
        pnv_pbcq_update_map(pbcq);
        pnv_phb3_remap_irqs(pbcq->phb);
        return true;
    case PBCQ_NEST_IRSN_COMPARE:
    case PBCQ_NEST_IRSN_MASK:
        pbcq->nest_regs[reg] = val & PBCQ_NEST_IRSN_COMP_MASK;
        pnv_phb3_remap_irqs(pbcq->phb);
        return true;
    case PBCQ_NEST_LSI_SRC_ID:
        pbcq->nest_regs[reg] = val & PBCQ_NEST_LSI_SRC_MASK;
        pnv_phb3_remap_irqs(pbcq->phb);
        return true;
    }

    /* XXX Don't error out on other regs for now ... */
    return true;
}

static bool pnv_pbcq_xpci_write(PnvPBCQState *pbcq, uint32_t reg, uint64_t val)
{
    switch(reg) {
    case PBCQ_PCI_BAR2:
        pbcq->pci_regs[reg] = val & 0xfffffffffc000000ull;
        pnv_pbcq_update_map(pbcq);
        break;
    }

    /* XXX Don't error out on other regs for now ... */
    return true;
}

static bool pnv_pbcq_xspci_write(PnvPBCQState *pbcq, uint32_t reg, uint64_t val)
{
    switch(reg) {
    case PBCQ_SPCI_ASB_ADDR:
        pbcq->spci_regs[reg] = val & 0xfff;
        return true;
    case PBCQ_SPCI_ASB_STATUS:
        pbcq->spci_regs[reg] &= ~val;
        return true;
    case PBCQ_SPCI_ASB_DATA:
        if (!pbcq->phb) {
            return true;
        }
        pnv_phb3_reg_write(pbcq->phb, pbcq->spci_regs[PBCQ_SPCI_ASB_ADDR], val, 8);
        return true;
        //   case PBCQ_SPCI_AIB_CAPP_EN:
        //   case PBCQ_SPCI_CAPP_SEC_TMR:
    }

    /* XXX Don't error out on other regs for now ... */
    return true;
}

static bool pnv_pbcq_xscom_write(XScomDevice *dev, uint32_t range,
                                 uint32_t offset, uint64_t val)
{
    PnvPBCQState *pbcq = PNV_PBCQ(dev);

    switch(range) {
    case 0:
            return pnv_pbcq_xnest_write(pbcq, offset, val);
    case 1:
            return pnv_pbcq_xpci_write(pbcq, offset, val);
    case 2:
            return pnv_pbcq_xspci_write(pbcq, offset, val);
    default:
            return false;
    }
}

static void pnv_pbcq_default_bars(PnvPBCQState *pbcq)
{
    uint64_t mm0, mm1, reg;

    mm0 = 0x3d00000000000ull +
            0x4000000000ull * pbcq->chip_id +
            0x1000000000ull * pbcq->phb_id;
    mm1 = 0x3ff8000000000ull +
            0x0200000000ull * pbcq->chip_id +
            0x0080000000ull * pbcq->phb_id;
    reg = 0x3fffe40000000ull +
            0x0000400000ull * pbcq->chip_id +
            0x0000100000ull * pbcq->phb_id;

    pbcq->nest_regs[PBCQ_NEST_MMIO_BAR0] = mm0 << 14;
    pbcq->nest_regs[PBCQ_NEST_MMIO_BAR1] = mm1 << 14;
    pbcq->nest_regs[PBCQ_NEST_PHB_BAR] = reg << 14;
    pbcq->nest_regs[PBCQ_NEST_MMIO_MASK0] = 0x3fff000000000ull << 14;
    pbcq->nest_regs[PBCQ_NEST_MMIO_MASK1] = 0x3ffff80000000ull << 14;
    pbcq->pci_regs[PBCQ_PCI_BAR2] = reg << 14;
}

static void pnv_pbcq_realize(DeviceState *dev, Error **errp)
{
    PnvPBCQState *pbcq = PNV_PBCQ(dev);
    XScomBus *xb = XSCOM_BUS(dev->parent_bus);
    XScomDevice *xd = XSCOM_DEVICE(dev);

    assert(pbcq->phb_id < 4);

    /* Copy chip ID over for ease of access */
    pbcq->chip_id = xb->chip_id;

    /* Calculate XSCOM bases */
    pbcq->nest_xbase = 0x02012000 + 0x400 * pbcq->phb_id;
    pbcq->pci_xbase  = 0x09012000 + 0x400 * pbcq->phb_id;
    pbcq->spci_xbase = 0x09013c00 + 0x040 * pbcq->phb_id;
    xd->ranges[0].addr = pbcq->nest_xbase;
    xd->ranges[0].size = PBCQ_NEST_REGS_COUNT;
    xd->ranges[1].addr = pbcq->pci_xbase;
    xd->ranges[1].size = PBCQ_PCI_REGS_COUNT;
    xd->ranges[2].addr = pbcq->spci_xbase;
    xd->ranges[2].size = PBCQ_SPCI_REGS_COUNT;

    /* XXX Fix OPAL to do that: establish default BAR values */
    pnv_pbcq_default_bars(pbcq);
}

#define _FDT(exp) \
    do { \
        int ret = (exp);                                           \
        if (ret < 0) {                                             \
            fprintf(stderr, "qemu: error creating device tree: %s: %s\n", \
                    #exp, fdt_strerror(ret));                      \
            exit(1);                                               \
        }                                                          \
    } while (0)


static int pnv_pbcq_devnode(XScomDevice *dev, void *fdt)
{
    PnvPBCQState *pbcq = PNV_PBCQ(dev);

    _FDT((fdt_property_cell(fdt, "ibm,phb-index", pbcq->phb_id)));

    return 0;
}

static Property pnv_pbcq_properties[] = {
        DEFINE_PROP_UINT32("phb_id", PnvPBCQState, phb_id, 0),
        DEFINE_PROP_END_OF_LIST(),

};

static void pnv_pbcq_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XScomDeviceClass *k = XSCOM_DEVICE_CLASS(klass);
    static const char *compat[] = { "ibm,power8-pbcq", NULL };

    k->devnode = pnv_pbcq_devnode;
    k->read = pnv_pbcq_xscom_read;
    k->write = pnv_pbcq_xscom_write;
    k->dt_name = "pbcq";
    k->dt_compatible = compat;

    dc->realize = pnv_pbcq_realize;
    dc->props = pnv_pbcq_properties;
}

static const TypeInfo pnv_pbcq_type_info = {
    .name          = TYPE_PNV_PBCQ,
    .parent        = TYPE_XSCOM_DEVICE,
    .instance_size = sizeof(PnvPBCQState),
    .class_init    = pnv_pbcq_class_init,
};

static void pnv_pbcq_register_types(void)
{
    type_register_static(&pnv_pbcq_type_info);
}

type_init(pnv_pbcq_register_types)
