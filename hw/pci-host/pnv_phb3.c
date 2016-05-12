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
#include "qapi/error.h"
#include "hw/pci-host/pnv_phb3.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "hw/ppc/xics.h"

#define DBG_ERR(p, fmt, ...) do { \
    if (1) fprintf(stderr, "PHB3(%s): " fmt "\n", __func__, ## __VA_ARGS__); \
    } while(0)

#define DBG_DMA(p, fmt, ...) do { \
    if (0) fprintf(stderr, "PHB3(%s): " fmt "\n", __func__, ## __VA_ARGS__); \
    } while(0)

#define DBG_MAP(p, fmt, ...) do { \
    if (0) fprintf(stderr, "PHB3(%s): " fmt "\n", __func__, ## __VA_ARGS__); \
    } while(0)

//#define DISPLAY_UNIMPLENTED_REG

static PCIDevice *pnb_phb3_find_cfg_dev(PnvPhb3State *phb)
{
    PCIHostState *pci = PCI_HOST_BRIDGE(phb);
    uint64_t addr = phb->regs[PHB_CONFIG_ADDRESS >> 3];
    uint8_t bus, devfn;

    if (!(addr >> 63)) {
        return NULL;
    }
    bus = (addr >> 52) & 0xff;
    devfn = (addr >> 44) & 0xff;

    return pci_find_device(pci->bus, bus, devfn);
}

static void pnv_phb3_config_write(PnvPhb3State *phb, unsigned off,
                                  unsigned size, uint64_t val)
{
    uint32_t cfg_addr, limit;
    PCIDevice *pdev;

    pdev = pnb_phb3_find_cfg_dev(phb);
    if (!pdev) {
        return;
    }
    cfg_addr = (phb->regs[PHB_CONFIG_ADDRESS >> 3] >> 32) & 0xfff;
    cfg_addr |= off;
    limit = pci_config_size(pdev);
    if (limit <= cfg_addr) {
        /* conventional pci device can be behind pcie-to-pci bridge.
           256 <= addr < 4K has no effects. */
        return;
    }
    switch (size) {
    case 1:
        break;
    case 2:
        val = bswap16(val);
        break;
    case 4:
        val = bswap32(val);
        break;
    default:
        DBG_ERR(phb, "Unsupported config access size %d !", size);
        return;
    }
    pci_host_config_write_common(pdev, cfg_addr, limit, val, size);
}

static uint64_t pnv_phb3_config_read(PnvPhb3State *phb, unsigned off,
                                     unsigned size)
{
    uint32_t cfg_addr, limit;
    PCIDevice *pdev;
    uint64_t val;

    pdev = pnb_phb3_find_cfg_dev(phb);
    if (!pdev) {
        return ~0ull;
    }
    cfg_addr = (phb->regs[PHB_CONFIG_ADDRESS >> 3] >> 32) & 0xffc;
    cfg_addr |= off;
    limit = pci_config_size(pdev);
    if (limit <= cfg_addr) {
        /* conventional pci device can be behind pcie-to-pci bridge.
           256 <= addr < 4K has no effects. */
        return ~0ull;
    }
    val = pci_host_config_read_common(pdev, cfg_addr, limit, size);
    switch (size) {
    case 1:
        return val;
    case 2:
        return bswap16(val);
    case 4:
        return bswap32(val);
    default:
        DBG_ERR(phb, "Unsupported config access size %d !", size);
        return ~0ull;
    }
}

static void pnv_phb3_check_m32(PnvPhb3State *phb)
{
    uint64_t base, start, size;
    MemoryRegion *parent;

    if (phb->m32_mapped) {
        // Should we destroy it in RCU friendly way... ?
        memory_region_del_subregion(phb->mr_m32.container, &phb->mr_m32);
        phb->m32_mapped = false;
    }

    /* Disabled ? move on with life ... */
    if (!(phb->regs[PHB_PHB3_CONFIG >> 3] & PHB_PHB3C_M32_EN)) {
        return;
    }

    /* Grab geometry from registers */
    base = phb->regs[PHB_M32_BASE_ADDR >> 3];
    start = phb->regs[PHB_M32_START_ADDR >> 3];
    size = ~(phb->regs[PHB_M32_BASE_MASK >> 3] | 0xfffc000000000000ull) + 1;

    DBG_MAP(phb, "PHB3: M32 enabled, base=%016llx/%016llx",
           (unsigned long long)base, (unsigned long long)size);
    DBG_MAP(phb, "PHB3: MMIO0            =%016llx/%016llx [%d]",
           (unsigned long long)phb->pbcq->mmio0_base,
           (unsigned long long)phb->pbcq->mmio0_size, phb->pbcq->mmio0_mapped);
    DBG_MAP(phb, "PHB3: MMIO1            =%016llx/%016llx [%d]",
           (unsigned long long)phb->pbcq->mmio1_base,
           (unsigned long long)phb->pbcq->mmio1_size, phb->pbcq->mmio1_mapped);
    /* Check if it matches an enabled MMIO region in the PBCQ */
    if (phb->pbcq->mmio0_mapped && base >= phb->pbcq->mmio0_base &&
        (base + size) <= (phb->pbcq->mmio0_base + phb->pbcq->mmio0_size)) {
        parent = &phb->pbcq->mmbar0;
        base -= phb->pbcq->mmio0_base;
        DBG_MAP(phb, "M32: Mapping under MMIO0");
    } else if (phb->pbcq->mmio1_mapped && base >= phb->pbcq->mmio1_base &&
        (base + size) <= (phb->pbcq->mmio1_base + phb->pbcq->mmio1_size)) {
        parent = &phb->pbcq->mmbar1;
        base -= phb->pbcq->mmio1_base;
        DBG_MAP(phb, "M32: Mapping under MMIO1");
    } else {
        DBG_MAP(phb, "M32 not matching either PBCQ MMIO region !");
        return;
    }

    /* Create alias */
    memory_region_init_alias(&phb->mr_m32, OBJECT(phb), "phb3-m32",
                             &phb->pci_mmio, start, size);
    memory_region_add_subregion(parent, base, &phb->mr_m32);
    phb->m32_mapped = true;
}

static void pnv_phb3_check_m64(PnvPhb3State *phb, uint32_t index)
{
    uint64_t base, start, size, m64;
    MemoryRegion *parent;

    if (phb->m64_mapped[index]) {
        // Should we destroy it in RCU friendly way... ?
        memory_region_del_subregion(phb->mr_m64[index].container,
                                    &phb->mr_m64[index]);
        phb->m64_mapped[index] = false;
    }

    /* Get table entry */
    m64 = phb->ioda_M64BT[index];

    /* Disabled ? move on with life ... */
    if (!(m64 & IODA2_M64BT_ENABLE)) {
        return;
    }

    /* Grab geometry from registers */
    base = GETFIELD(IODA2_M64BT_BASE, m64) << 20;
    if (m64 & IODA2_M64BT_SINGLE_PE) {
        base &= ~0x1ffffffull;
    }
    size = GETFIELD(IODA2_M64BT_MASK, m64) << 20;
    size |= 0xfffc000000000000ull;
    size = ~size + 1;
    start = base | (phb->regs[PHB_M64_UPPER_BITS >> 3]);

    DBG_MAP(phb, "PHB3: M64[%d] enabled, base=%016llx/%016llx", index,
           (unsigned long long)base, (unsigned long long)size);
    DBG_MAP(phb, "PHB3: MMIO0            =%016llx/%016llx [%d]",
           (unsigned long long)phb->pbcq->mmio0_base,
           (unsigned long long)phb->pbcq->mmio0_size, phb->pbcq->mmio0_mapped);
    DBG_MAP(phb, "PHB3: MMIO1            =%016llx/%016llx [%d]",
           (unsigned long long)phb->pbcq->mmio1_base,
           (unsigned long long)phb->pbcq->mmio1_size, phb->pbcq->mmio1_mapped);
    /* Check if it matches an enabled MMIO region in the PBCQ */
    if (phb->pbcq->mmio0_mapped && base >= phb->pbcq->mmio0_base &&
        (base + size) <= (phb->pbcq->mmio0_base + phb->pbcq->mmio0_size)) {
        parent = &phb->pbcq->mmbar0;
        base -= phb->pbcq->mmio0_base;
        DBG_MAP(phb, "M64: Mapping under MMIO0");
    } else if (phb->pbcq->mmio1_mapped && base >= phb->pbcq->mmio1_base &&
        (base + size) <= (phb->pbcq->mmio1_base + phb->pbcq->mmio1_size)) {
        parent = &phb->pbcq->mmbar1;
        base -= phb->pbcq->mmio1_base;
        DBG_MAP(phb, "M64: Mapping under MMIO1");
    } else {
        DBG_MAP(phb, "M64 not matching either PBCQ MMIO region !");
        return;
    }

    /* Create alias */
    memory_region_init_alias(&phb->mr_m64[index], OBJECT(phb), "phb3-m64",
                             &phb->pci_mmio, start, size);
    memory_region_add_subregion(parent, base, &phb->mr_m64[index]);
    phb->m64_mapped[index] = true;
}

static void pnv_phb3_check_all_m64s(PnvPhb3State *phb)
{
    uint64_t i;

    for (i=0; i<PHB_NUM_M64; i++) {
        pnv_phb3_check_m64(phb, i);
    }
}

static void pnv_phb3_lxivt_write(PnvPhb3State *phb, unsigned idx, uint64_t val)
{
    uint8_t server, prio;

    phb->ioda_LXIVT[idx] = val & (IODA2_LXIVT_SERVER_MASK |
                                  IODA2_LXIVT_PRIORITY_MASK |
                                  IODA2_LXIVT_NODE_ID_MASK);
    server = GETFIELD(IODA2_LXIVT_SERVER, val);
    prio = GETFIELD(IODA2_LXIVT_PRIORITY, val);

    ics_simple_write_xive(phb->lsi_ics, idx, server, prio, prio);
}

static uint64_t *pnv_phb3_ioda_access(PnvPhb3State *phb,
                                      unsigned *out_table, unsigned *out_idx)
{
    uint64_t adreg = phb->regs[PHB_IODA_ADDR >> 3];
    unsigned int index = GETFIELD(PHB_IODA_AD_TADR, adreg);
    unsigned int table = GETFIELD(PHB_IODA_AD_TSEL, adreg);
    unsigned int mask;
    uint64_t *tptr = NULL;

    switch(table) {
    case IODA2_TBL_LIST:
        tptr = phb->ioda_LIST;
        mask = 7;
        break;
    case IODA2_TBL_LXIVT:
        tptr = phb->ioda_LXIVT;
        mask = 7;
        break;
    case IODA2_TBL_IVC_CAM:
    case IODA2_TBL_RBA:
        mask = 31;
        break;
    case IODA2_TBL_RCAM:
        mask = 63;
        break;
    case IODA2_TBL_MRT:
        mask = 7;
        break;
    case IODA2_TBL_PESTA:
    case IODA2_TBL_PESTB:
        mask = 255;
        break;
    case IODA2_TBL_TVT:
        tptr = phb->ioda_TVT;
        mask = 255;
        break;
    case IODA2_TBL_TCAM:
    case IODA2_TBL_TDR:
        mask = 63;
        break;
    case IODA2_TBL_M64BT:
        tptr = phb->ioda_M64BT;
        mask = 15;
        break;
    case IODA2_TBL_M32DT:
        tptr = phb->ioda_MDT;
        mask = 255;
        break;
    case IODA2_TBL_PEEV:
        tptr = phb->ioda_PEEV;
        mask = 3;
        break;
    default:
        DBG_ERR(phb, "Unknown IODA table idx %d", table);
        return NULL;
    }
    index &= mask;
    if (out_idx) {
        *out_idx = index;
    }
    if (out_table) {
        *out_table = table;
    }
    if (adreg & PHB_IODA_AD_AUTOINC) {
        index = (index + 1) & mask;
        adreg = SETFIELD(PHB_IODA_AD_TADR, adreg, index);
    }
    if (tptr) {
        tptr += index;
    }
    phb->regs[PHB_IODA_ADDR >> 3] = adreg;
    return tptr;
}

static uint64_t pnv_phb3_ioda_read(PnvPhb3State *phb)
{
        unsigned table;
        uint64_t *tptr;

        tptr = pnv_phb3_ioda_access(phb, &table, NULL);
        if (!tptr) {
            /* Return 0 on unsupported tables, not ff's */
            return 0;
        }
        return *tptr;
}

static void pnv_phb3_ioda_write(PnvPhb3State *phb, uint64_t val)
{
        unsigned table, idx;
        uint64_t *tptr;

        tptr = pnv_phb3_ioda_access(phb, &table, &idx);
        if (!tptr) {
            return;
        }

        /* Handle side effects */
        switch(table) {
        case IODA2_TBL_LXIVT:
            pnv_phb3_lxivt_write(phb, idx, val);
            break;
        case IODA2_TBL_M64BT:
            *tptr = val;
            pnv_phb3_check_m64(phb, idx);
            break;
        default:
            *tptr = val;
        }
}

/* This is called whenever the PHB LSI, MSI source ID register or
 * the PBCQ irq filters are written.
 */
void pnv_phb3_remap_irqs(PnvPhb3State *phb)
{
    uint32_t local, global, count, mask, comp;
    uint64_t baren;

    /* First check if we are enabled. Unlike real HW we don't separate TX and RX
     * so we enable if both are set
     */
    baren = phb->pbcq->nest_regs[PBCQ_NEST_BAR_EN];
    if (!(baren & PBCQ_NEST_BAR_EN_IRSN_RX) ||
        !(baren & PBCQ_NEST_BAR_EN_IRSN_TX)) {
        phb->lsi_ics->offset = 0;
        return;
    }

    /* Grab local LSI source ID */
    local = GETFIELD(PHB_LSI_SRC_ID, phb->regs[PHB_LSI_SOURCE_ID >> 3]) << 3;

    /* Grab global one and compare */
    global = GETFIELD(PBCQ_NEST_LSI_SRC,
                      phb->pbcq->nest_regs[PBCQ_NEST_LSI_SRC_ID]) << 3;
    if (global != local) {
        /* This happens during initialization, let's come back when we
         * are properly configured
         */
        phb->lsi_ics->offset = 0;
        return;
    }

    /* Get the base on the powerbus */
    comp = GETFIELD(PBCQ_NEST_IRSN_COMP,
                    phb->pbcq->nest_regs[PBCQ_NEST_IRSN_COMPARE]);
    mask = GETFIELD(PBCQ_NEST_IRSN_COMP,
                    phb->pbcq->nest_regs[PBCQ_NEST_IRSN_MASK]);
    count = ((~mask) + 1) & 0x7ffff;
    phb->total_irq = count;

    /* Sanity checks */
    if ((global + 8) > count) {
        DBG_MAP(phb, "LSIs out of reach: LSI base=%d total irq=%d",
                global, count);
    }
    if (count > 2048) {
        DBG_MAP(phb, "More interrupts than supported: %d", count);
    }
    if ((comp & mask) != comp) {
        DBG_MAP(phb, "IRQ compare bits not in mask: comp=0x%x mask=0x%x",
                comp, mask);
        comp &= mask;
    }
    /* Setup LSI offset */
    phb->lsi_ics->offset = comp + global;

    /* Setup MSI offset */
    pnv_phb3_msi_update_config(phb->msis, comp, count);

    DBG_MAP(phb, "Initialized for %d interrupts @0x%x, LSI off=%d",
            count, comp, global);
}

static void pnv_phb3_lsi_src_id_write(PnvPhb3State *phb, uint64_t val)
{
    /* Sanitize content */
    val &= PHB_LSI_SRC_ID_MASK;
    phb->regs[PHB_LSI_SOURCE_ID >> 3] = val;
    pnv_phb3_remap_irqs(phb);
}

static void pnv_phb3_rtc_invalidate(PnvPhb3State *phb, uint64_t val)
{
    PnvPhb3DMASpace *ds;

    /* Always invalidate all for now ... */
    QLIST_FOREACH(ds, &phb->dma_spaces, list) {
        ds->pe_num = PHB_INVALID_PE;
    }
}


static void pnv_phb3_update_msi_regions(PnvPhb3DMASpace *ds)
{
    uint64_t cfg = ds->phb->regs[PHB_PHB3_CONFIG >> 3];

    if (cfg & PHB_PHB3C_32BIT_MSI_EN) {
        if (!ds->msi32_mapped) {
            memory_region_add_subregion(&ds->dma_mr, 0xffff0000, &ds->msi32_mr);
            ds->msi32_mapped = true;
        }
    } else {
        if (ds->msi32_mapped) {
            memory_region_del_subregion(&ds->dma_mr, &ds->msi32_mr);
            ds->msi32_mapped = false;
        }
    }

    if (cfg & PHB_PHB3C_64BIT_MSI_EN) {
        if (!ds->msi64_mapped) {
            memory_region_add_subregion(&ds->dma_mr,
                                        (1ull << 60), &ds->msi64_mr);
            ds->msi64_mapped = true;
        }
    } else {
        if (ds->msi64_mapped) {
            memory_region_del_subregion(&ds->dma_mr, &ds->msi64_mr);
            ds->msi64_mapped = false;
        }
    }
}

static void pnv_phb3_update_all_msi_regions(PnvPhb3State *phb)
{
    PnvPhb3DMASpace *ds;

    QLIST_FOREACH(ds, &phb->dma_spaces, list) {
        pnv_phb3_update_msi_regions(ds);
    }
}

void pnv_phb3_reg_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    PnvPhb3State *phb = opaque;
    bool changed;

    /* Special case configuration data */
    if ((off & 0xfffc) == PHB_CONFIG_DATA) {
        pnv_phb3_config_write(phb, off & 0x3, size, val);
        return;
    }

    /* Other registers are 64-bit only */
    if (size != 8 || off & 0x7) {
        DBG_ERR(phb, "Invalid register access, offset: 0x%x size: %d",
                (unsigned int)off, size);
        return;
    }

    /* Handle masking */
    switch(off) {
    case PHB_M64_UPPER_BITS:
        val &= 0xfffc000000000000ull;
        break;
    }

    /* Record whether it changed */
    changed = phb->regs[off >> 3] != val;

    /* Store in register cache first */
    phb->regs[off >> 3] = val;

    /* Handle side effects */
    switch(off) {
    case PHB_PHB3_CONFIG:
        if (changed) {
            pnv_phb3_update_all_msi_regions(phb);
        }
        /* fall through */
    case PHB_M32_BASE_ADDR:
    case PHB_M32_BASE_MASK:
    case PHB_M32_START_ADDR:
        if (changed) {
            pnv_phb3_check_m32(phb);
        }
        break;
    case PHB_M64_UPPER_BITS:
        if (changed) {
            pnv_phb3_check_all_m64s(phb);
        }
        break;
    case PHB_LSI_SOURCE_ID:
        if (changed) {
            pnv_phb3_lsi_src_id_write(phb, val);
        }
        break;

    /* IODA table accesses */
    case PHB_IODA_DATA0:
        pnv_phb3_ioda_write(phb, val);
        break;

    /* RTC invalidation */
    case PHB_RTC_INVALIDATE:
        pnv_phb3_rtc_invalidate(phb, val);
        break;

    /* FFI request */
    case PHB_FFI_REQUEST:
        pnv_phb3_msi_ffi(phb->msis, val);
        break;

    /* Silent simple writes */
    case PHB_CONFIG_ADDRESS:
    case PHB_IODA_ADDR:
    case PHB_TCE_KILL:
    case PHB_TCE_SPEC_CTL:
    case PHB_PEST_BAR:
    case PHB_PELTV_BAR:
    case PHB_RTT_BAR:
    case PHB_RBA_BAR:
    case PHB_IVT_BAR:
    case PHB_FFI_LOCK:
        break;

#ifdef DISPLAY_UNIMPLENTED_REG
    /* Noise on anything else */
    default:
        DBG_ERR(phb, "reg_write 0x%x=%016llx",
               (unsigned int)off, (unsigned long long)val);
#endif
    }
}

uint64_t pnv_phb3_reg_read(void *opaque, hwaddr off, unsigned size)
{
    PnvPhb3State *phb = opaque;
    uint64_t val;

    if ((off & 0xfffc) == PHB_CONFIG_DATA) {
        return pnv_phb3_config_read(phb, off & 0x3, size);
    }

    /* Other registers are 64-bit only */
    if (size != 8 || off & 0x7) {
        DBG_ERR(phb, "Invalid register access, offset: 0x%x size: %d",
                (unsigned int)off, size);
        return ~0ull;
    }

    /* Default read from cache */
    val = phb->regs[off >> 3];

    switch(off) {
    /* Simulate venice DD2.0 */
    case PHB_VERSION:
        return 0x000000a300000005ull;

    /* IODA table accesses */
    case PHB_IODA_DATA0:
        return pnv_phb3_ioda_read(phb);

    /* Link training always appears trained */
    case PHB_PCIE_DLP_TRAIN_CTL:
        return PHB_PCIE_DLP_INBAND_PRESENCE | PHB_PCIE_DLP_TC_DL_LINKACT;

    /* FFI Lock */
    case PHB_FFI_LOCK:
        /* Set lock and return previous value */
        phb->regs[off >> 3] |= PHB_FFI_LOCK_STATE;
        return val;

    /* Silent simple reads */
    case PHB_PHB3_CONFIG:
    case PHB_M32_BASE_ADDR:
    case PHB_M32_BASE_MASK:
    case PHB_M32_START_ADDR:
    case PHB_CONFIG_ADDRESS:
    case PHB_IODA_ADDR:
    case PHB_RTC_INVALIDATE:
    case PHB_TCE_KILL:
    case PHB_TCE_SPEC_CTL:
    case PHB_PEST_BAR:
    case PHB_PELTV_BAR:
    case PHB_RTT_BAR:
    case PHB_RBA_BAR:
    case PHB_IVT_BAR:
    case PHB_M64_UPPER_BITS:
        break;

#ifdef DISPLAY_UNIMPLENTED_REG
    /* Noise on anything else */
    default:
        DBG_ERR(phb, "reg_read 0x%x=%016llx",
                (unsigned int)off, (unsigned long long)val);
#endif
    }
    return val;
}

static const MemoryRegionOps pnv_phb3_reg_ops = {
    .read = pnv_phb3_reg_read,
    .write = pnv_phb3_reg_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static int pnv_phb3_map_irq(PCIDevice *pci_dev, int irq_num)
{
    /* Check that out properly ... */
    return irq_num & 3;
}

static void pnv_phb3_set_irq(void *opaque, int irq_num, int level)
{
    PnvPhb3State *phb = opaque;

    /* LSI only ... */
    if (irq_num > 3) {
        DBG_ERR(phb, "Unknown IRQ to set %d", irq_num);
    }
    qemu_set_irq(phb->lsi_ics->qirqs[irq_num], level);
}

static bool pnv_phb3_resolve_pe(PnvPhb3DMASpace *ds)
{
    uint64_t rtt, addr;
    uint16_t rte;
    int bus_num;

    /* Already resolved ? */
    if (ds->pe_num != PHB_INVALID_PE) {
        return true;
    }

    /* We need to lookup the RTT */
    rtt = ds->phb->regs[PHB_RTT_BAR >> 3];
    if (!(rtt & PHB_RBA_BAR_ENABLE)) {
        DBG_ERR(ds->phb, "DMA with RTT BAR disabled !");
        // Set error bits ? fence ? ...
        return false;
    }

    /* Read RTE */
    bus_num = pci_bus_num(ds->bus);
    addr = rtt & PHB_RTT_BASE_ADDRESS_MASK;
    addr += 2 * ((bus_num << 8) | ds->devfn);
    if (dma_memory_read(&address_space_memory, addr, &rte, sizeof(rte))) {
        DBG_ERR(ds->phb, "Failed to read RTT entry at %016llx",
                (unsigned long long)addr);
        // Set error bits ? fence ? ...
        return false;
    }
    rte = be16_to_cpu(rte);

    /* Fail upon reading of invalid PE# */
    if (rte >= PHB_NUM_PE) {
        DBG_ERR(ds->phb, "RTE for RID 0x%x invalid (%04x)", ds->devfn, rte);
        // Set error bits ? fence ? ...
        return false;
    }
    ds->pe_num = rte;
    return true;
}

static void pnv_phb3_translate_tve(PnvPhb3DMASpace *ds, hwaddr addr,
                                   bool is_write, uint64_t tve,
                                   IOMMUTLBEntry *tlb)
{
    uint64_t tta = GETFIELD(IODA2_TVT_TABLE_ADDR, tve);
    int32_t  lev = GETFIELD(IODA2_TVT_NUM_LEVELS, tve);
    uint32_t tts = GETFIELD(IODA2_TVT_TCE_TABLE_SIZE, tve);
    uint32_t tps = GETFIELD(IODA2_TVT_IO_PSIZE, tve);

    DBG_DMA(ds->phb, "xlate %016llx:%c TVE=%016llx",
            (unsigned long long)addr, is_write ? 'W' : 'R',
            (unsigned long long)tve);

    DBG_DMA(ds->phb, " tta=%016llx lev=%d tts=%d tps=%d",
            (unsigned long long)tta, lev, tts, tps);

    /* Invalid levels */
    if (lev > 4) {
        DBG_ERR(ds->phb, "Invalid #levels in TVE %d", lev);
        return;
    }

    /* IO Page Size of 0 means untranslated, else use TCEs */
    if (tps == 0) {
        /* We only support non-translate in top window
         * XXX FIX THAT, Venice/Murano support it on bottom window
         * above 4G and Naples suports it on everything
         */
        if (!(tve & PPC_BIT(51))) {
            DBG_ERR(ds->phb, "xlate for invalid non-translate TVE");
            return;
        }
        // XXX Handle boundaries */

        /* XXX Use 4k pages like q35 ... for now */
        DBG_DMA(ds->phb, " non-translate ok");
        tlb->iova = addr & 0xfffffffffffff000ull;
        tlb->translated_addr = addr & 0x0003fffffffff000ull;
        tlb->addr_mask = 0xfffull;
        tlb->perm = IOMMU_RW;
    } else {
        uint32_t tce_shift, tbl_shift, sh;
        uint64_t base, taddr, tce, tce_mask;

        /* TVE disabled ? */
        if (tts == 0) {
            DBG_ERR(ds->phb, "xlate for invalid translated TVE");
            return;
        }

        /* Address bits per bottom level TCE entry */
        tce_shift = tps + 11;

        /* Address bits per table level */
        tbl_shift = tts + 8;

        /* Top level table base address */
        base = tta << 12;

        /* Total shift to first level */
        sh = tbl_shift * lev + tce_shift;

        DBG_DMA(ds->phb, " tce_shift %d tbl_shift %d", tce_shift, tbl_shift);

        // XXX Multi-level untested */
        while ((lev--) >= 0) {
            /* Grab the TCE address */
            taddr = base | (((addr >> sh) & ((1ul << tbl_shift) - 1)) << 3);
            if (dma_memory_read(&address_space_memory, taddr, &tce, sizeof(tce))) {
                DBG_ERR(ds->phb, "Failed to read TCE at 0x%016llx",
                        (unsigned long long)taddr);
                return;
            }
            tce = be64_to_cpu(tce);
            DBG_DMA(ds->phb, " lev %d taddr 0x%016llx tce 0x%016llx", lev + 1,
                    (unsigned long long)taddr, (unsigned long long)tce);

            /* Check permission for indirect TCE */
            if ((lev >= 0) && !(tce & 3)) {
                DBG_ERR(ds->phb, "Invalid indirect TCE at 0x%016llx",
                        (unsigned long long)taddr);
                DBG_ERR(ds->phb, " xlate %016llx:%c TVE=%016llx",
                        (unsigned long long)addr, is_write ? 'W' : 'R',
                        (unsigned long long)tve);
                DBG_ERR(ds->phb, " tta=%016llx lev=%d tts=%d tps=%d",
                        (unsigned long long)tta, lev, tts, tps);
                return;
            }
            sh -= tbl_shift;
            base = tce & ~0xfffull;
        }

        /* We exit the loop with TCE being the final TCE */
        tce_mask = ~((1ull << tce_shift) - 1);
        tlb->iova = addr & tce_mask;
        tlb->translated_addr = tce & tce_mask;
        tlb->addr_mask = ~tce_mask;
        tlb->perm = tce & 3;
        if ((is_write & !(tce & 2)) || ((!is_write) && !(tce & 1))) {
            DBG_ERR(ds->phb, "TCE access fault at 0x%016llx",
                    (unsigned long long)taddr);
            DBG_ERR(ds->phb, " xlate %016llx:%c TVE=%016llx",
                    (unsigned long long)addr, is_write ? 'W' : 'R',
                    (unsigned long long)tve);
            DBG_ERR(ds->phb, " tta=%016llx lev=%d tts=%d tps=%d",
                    (unsigned long long)tta, lev, tts, tps);
        }
    }
}

static IOMMUTLBEntry pnv_phb3_translate_iommu(MemoryRegion *iommu, hwaddr addr,
                                              bool is_write)
{
    PnvPhb3DMASpace *ds = container_of(iommu, PnvPhb3DMASpace, dma_mr);
    int tve_sel;
    uint64_t tve, cfg;
    IOMMUTLBEntry ret = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = 0,
        .addr_mask = ~(hwaddr)0,
        .perm = IOMMU_NONE,
    };

    /* Resolve PE# */
    if (!pnv_phb3_resolve_pe(ds)) {
        DBG_ERR(ds->phb, "Failed to resolve PE# for bus @%p (%d) devfn 0x%x",
                ds->bus, pci_bus_num(ds->bus), ds->devfn);
        return ret;
    }

    DBG_DMA(ds->phb, "xlate bus @%p (%d) devfn 0x%x PE %d ds @%p",
            ds->bus, pci_bus_num(ds->bus),ds->devfn, ds->pe_num, ds);

    /* Check top bits */
    switch (addr >> 60) {
    case 00:
        /* DMA or 32-bit MSI ? */
        cfg = ds->phb->regs[PHB_PHB3_CONFIG >> 3];
        if ((cfg & PHB_PHB3C_32BIT_MSI_EN) &&
            ((addr & 0xffffffffffff0000ull) == 0xffff0000ull)) {
            DBG_ERR(ds->phb, "xlate on 32-bit MSI region");
            return ret;
        }
        /* Choose TVE XXX Use PHB3 Control Register */
        tve_sel = (addr >> 59) & 1;
        tve = ds->phb->ioda_TVT[ds->pe_num * 2 + tve_sel];
        DBG_DMA(ds->phb, " TVE_SEL=%d -> %d",
                tve_sel, ds->pe_num * 2 + tve_sel);
        pnv_phb3_translate_tve(ds, addr, is_write, tve, &ret);
        break;
    case 01:
        DBG_ERR(ds->phb, "xlate on 64-bit MSI region");
        break;
    default:
        DBG_ERR(ds->phb, "xlate on unsupported address 0x%016llx",
                (unsigned long long)addr);
    }
    return ret;
}

static const MemoryRegionIOMMUOps pnv_phb3_iommu_ops = {
    .translate = pnv_phb3_translate_iommu,
};

/*
 * MSI/MSIX memory region implementation.
 * The handler handles both MSI and MSIX.
 */
static void pnv_phb3_msi_write(void *opaque, hwaddr addr,
                               uint64_t data, unsigned size)
{
    PnvPhb3DMASpace *ds = opaque;

    /* Resolve PE# */
    if (!pnv_phb3_resolve_pe(ds)) {
        DBG_ERR(ds->phb, "Failed to resolve PE# for bus @%p (%d) devfn 0x%x",
                ds->bus, pci_bus_num(ds->bus), ds->devfn);
        return;
    }

    pnv_phb3_msi_send(ds->phb->msis, addr, data, ds->pe_num);
}

static const MemoryRegionOps pnv_phb3_msi_ops = {
    /* There is no .read as the read result is undefined by PCI spec */
    .read = NULL,
    .write = pnv_phb3_msi_write,
    .endianness = DEVICE_LITTLE_ENDIAN
};

static AddressSpace *pnv_phb3_dma_iommu(PCIBus *bus, void *opaque, int devfn)
{
    PnvPhb3State *phb = opaque;
    PnvPhb3DMASpace *ds;

    DBG_DMA(phb, "get IOMMU for bus @%p devfn 0x%x", bus, devfn);

    QLIST_FOREACH(ds, &phb->dma_spaces, list) {
        if (ds->bus == bus && ds->devfn == devfn) {
            DBG_DMA(phb, " found @%p", ds);
            break;
        }
    }
    if (ds == NULL) {
        ds = g_malloc0(sizeof(PnvPhb3DMASpace));
        ds->bus = bus;
        ds->devfn = devfn;
        ds->pe_num = PHB_INVALID_PE;
        ds->phb = phb;
        memory_region_init_iommu(&ds->dma_mr, OBJECT(phb),
                                 &pnv_phb3_iommu_ops, "phb3_iommu", UINT64_MAX);
        address_space_init(&ds->dma_as, &ds->dma_mr, "phb3_iommu");
        memory_region_init_io(&ds->msi32_mr, OBJECT(phb), &pnv_phb3_msi_ops,
                              ds, "msi32", 0x10000);
        memory_region_init_io(&ds->msi64_mr, OBJECT(phb), &pnv_phb3_msi_ops,
                              ds, "msi64", 0x100000);
        pnv_phb3_update_msi_regions(ds);

        QLIST_INSERT_HEAD(&phb->dma_spaces, ds, list);
        DBG_DMA(phb, " created @%p", ds);
    }
    return &ds->dma_as;
}

static void pnv_phb3_root_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->max_dev = 1;
}

#define TYPE_PNV_PHB3_ROOT_BUS "pnv-phb3-root-bus"

static const TypeInfo pnv_phb3_root_bus_info = {
    .name = TYPE_PNV_PHB3_ROOT_BUS,
    .parent = TYPE_PCIE_BUS,
    .class_init = pnv_phb3_root_bus_class_init,
};

static void pnv_phb3_initfn(Object *obj)
{
    PnvPhb3State *phb = PNV_PHB3(obj);

    phb->lsi_ics = ICS(object_new(TYPE_ICS_SIMPLE));
    object_property_add_child(OBJECT(phb), "ics", OBJECT(phb->lsi_ics), NULL);
    /* Default init ... will be fixed by HW inits */
    phb->lsi_ics->offset = 0;
    phb->lsi_ics->nr_irqs = PHB_NUM_LSI;
    QLIST_INIT(&phb->dma_spaces);
}

static void pnv_phb3_realize(DeviceState *dev, Error **errp)
{
    PnvPhb3State *phb = PNV_PHB3(dev);
    PCIHostState *pci = PCI_HOST_BRIDGE(dev);
    Error *error = NULL;
    int i;

    memory_region_init(&phb->pci_mmio, OBJECT(phb), "pci-mmio",
                       PCI_MMIO_TOTAL_SIZE);

    /* PHB3 doesn't support IO space. However, qemu gets very upset if
     * we don't have an IO region to anchor IO BARs onto so we just
     * initialize one which we never hook up to anything
     */
    memory_region_init(&phb->pci_io, OBJECT(phb), "pci-io", 0x10000);

    memory_region_init_io(&phb->mr_regs, OBJECT(phb), &pnv_phb3_reg_ops, phb,
                          "phb3-regs", 0x1000);

    /* Realize LSI ICS */
    xics_add_ics(phb->xics, phb->lsi_ics);
    object_property_set_bool(OBJECT(phb->lsi_ics), true, "realized", &error);
    if (error) {
        error_propagate(errp, error);
        return;
    }
    for (i = 0; i < PHB_NUM_LSI; i++)
        ics_simple_set_irq_type(phb->lsi_ics, i, true);

    pci->bus = pci_register_bus(dev, "phb3-root-bus",
                                pnv_phb3_set_irq, pnv_phb3_map_irq, phb,
                                &phb->pci_mmio, &phb->pci_io,
                                0, 4, TYPE_PNV_PHB3_ROOT_BUS);
    pci->bus->devfn_max = 1;
    pci_setup_iommu(pci->bus, pnv_phb3_dma_iommu, phb);
}

void pnv_phb3_update_regions(PnvPhb3State *phb)
{
    /* Unmap first always */
    if (phb->regs_mapped) {
        memory_region_del_subregion(&phb->pbcq->phbbar, &phb->mr_regs);
        phb->regs_mapped = false;
    }

    /* Map registers if enabled */
    if (phb->pbcq->phb_mapped) {
        /* XXX We should use the PHB BAR 2 register but we don't ... */
        memory_region_add_subregion(&phb->pbcq->phbbar, 0, &phb->mr_regs);
        phb->regs_mapped = true;
    }

    /* Check/update m32 */
    if (phb->m32_mapped) {
        pnv_phb3_check_m32(phb);
    }
}

static void pnv_phb3_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pnv_phb3_realize;
}

static const TypeInfo pnv_phb3_type_info = {
    .name = TYPE_PNV_PHB3,
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PnvPhb3State),
    .class_init = pnv_phb3_class_init,
    .instance_init = pnv_phb3_initfn,
};

static void pnv_phb3_register_types(void)
{
    type_register_static(&pnv_phb3_type_info);
    type_register_static(&pnv_phb3_root_bus_info);
}

type_init(pnv_phb3_register_types)

void pnv_phb3_create(PnvChip *chip, XICSState *xics, uint32_t idx)
{
    struct DeviceState *dev;
    PnvPhb3State *phb;
    PnvPBCQState *pbcq;
    PCIHostState *pcih;
    PCIDevice *pdev;
    PCIBridge *bdev;
    uint8_t chassis;

    chassis = chip->chip_id * 4 + idx;

    /* Create PBCQ */
    dev = qdev_create(&chip->xscom->bus, TYPE_PNV_PBCQ);
    qdev_prop_set_uint32(dev, "phb_id", idx);
    qdev_init_nofail(dev);
    pbcq = PNV_PBCQ(dev);

    /* Create PHB3 */
    dev = qdev_create(NULL, TYPE_PNV_PHB3);
    phb = PNV_PHB3(dev);
    phb->pbcq = pbcq;
    pbcq->phb = phb;
    phb->xics = xics;
    qdev_init_nofail(dev);
    pcih = PCI_HOST_BRIDGE(phb);

    /* Create MSI source */
    phb->msis = pnv_phb3_msi_create(phb);

    /* Add root complex */
    pdev = pci_create_multifunction(pcih->bus, 0, false, TYPE_PNV_PHB3_RC);
    qdev_prop_set_uint8(&pdev->qdev, "chassis", chassis);
    qdev_prop_set_uint16(&pdev->qdev, "slot", 1);
    qdev_init_nofail(&pdev->qdev);
    bdev = PCI_BRIDGE(pdev);

    /* Setup bus for that chip */
    chip->phb[idx] = pci_bridge_get_sec_bus(bdev);
}
