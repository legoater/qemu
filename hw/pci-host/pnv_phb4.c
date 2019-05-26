/*
 * QEMU PowerPC PowerNV PHB4 model
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "monitor/monitor.h"
#include "target/ppc/cpu.h"
#include "hw/pci-host/pnv_phb4_regs.h"
#include "hw/pci-host/pnv_phb4.h"
#include "hw/pci/pcie_host.h"
#include "hw/pci/pcie_port.h"
#include "hw/ppc/pnv.h"

#define phb4_error(fmt, ...)                                            \
    qemu_log_mask(LOG_GUEST_ERROR, "phb4: " fmt "\n",  ## __VA_ARGS__)

/* TODO: fix GETFIELD/SETFIELD macro definitions */
#if HOST_LONG_BITS == 32
# define MASK_TO_LSH(m)          (__builtin_ffsll(m) - 1)
#elif HOST_LONG_BITS == 64
# define MASK_TO_LSH(m)          (__builtin_ffsl(m) - 1)
#else
# error Unknown sizeof long
#endif

#define GETFIELD(m, v)          (((v) & (m)) >> MASK_TO_LSH(m))
#define SETFIELD(m, v, val)                             \
        (((v) & ~(m)) | ((((typeof(v))(val)) << MASK_TO_LSH(m)) & (m)))

static PCIDevice *pnv_phb4_find_cfg_dev(PnvPHB4 *phb)
{
    PCIHostState *pci = PCI_HOST_BRIDGE(phb);
    uint64_t addr = phb->regs[PHB_CONFIG_ADDRESS >> 3];
    uint8_t bus, devfn;

    if (!(addr >> 63)) {
        return NULL;
    }
    bus = (addr >> 52) & 0xff;
    devfn = (addr >> 44) & 0xff;

    /* We don't access the root complex this way */
    if (bus == 0 && devfn == 0) {
        return NULL;
    }
    return pci_find_device(pci->bus, bus, devfn);
}

/*
 * The CONFIG_DATA register expects little endian accesses, but as the
 * region is big endian, we have to swap the value.
 */
static void pnv_phb4_config_write(PnvPHB4 *phb, unsigned off,
                                  unsigned size, uint64_t val)
{
    uint32_t cfg_addr, limit;
    PCIDevice *pdev;

    pdev = pnv_phb4_find_cfg_dev(phb);
    if (!pdev) {
        return;
    }
    cfg_addr = (phb->regs[PHB_CONFIG_ADDRESS >> 3] >> 32) & 0xffc;
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
        g_assert_not_reached();
    }
    pci_host_config_write_common(pdev, cfg_addr, limit, val, size);
}

static uint64_t pnv_phb4_config_read(PnvPHB4 *phb, unsigned off,
                                     unsigned size)
{
    uint32_t cfg_addr, limit;
    PCIDevice *pdev;
    uint64_t val;

    pdev = pnv_phb4_find_cfg_dev(phb);
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
        g_assert_not_reached();
    }
}

/*
 * Root complex register accesses are memory mapped.
 */
static void pnv_phb4_rc_config_write(PnvPHB4 *phb, unsigned off,
                                     unsigned size, uint64_t val)
{
    PCIHostState *pci = PCI_HOST_BRIDGE(phb);
    PCIDevice *pdev;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "phb4: rc_config_write invalid size %d\n", size);
        return;
    }

    pdev = pci_find_device(pci->bus, 0, 0);
    if (!pdev) {
        // XXX No RC ?
        assert(false);
        return;
    }
    pci_host_config_write_common(pdev, off, PHB_RC_CONFIG_SIZE, bswap32(val), 4);
}

static uint64_t pnv_phb4_rc_config_read(PnvPHB4 *phb, unsigned off,
                                        unsigned size)
{
    PCIHostState *pci = PCI_HOST_BRIDGE(phb);
    PCIDevice *pdev;
    uint64_t val;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "phb4: rc_config_read invalid size %d\n", size);
        return ~0ull;
    }

    pdev = pci_find_device(pci->bus, 0, 0);
    if (!pdev) {
        // XXX
        assert(false);
        return ~0ull;
    }
    val = pci_host_config_read_common(pdev, off, PHB_RC_CONFIG_SIZE, 4);
    return bswap32(val);
}

static void pnv_phb4_check_mbt(PnvPHB4 *phb, uint32_t index)
{
    uint64_t base, start, size, mbe0, mbe1;
    MemoryRegion *parent;
    char name[64];

    /* Unmap first */
    if (phb->mmio_mapped[index]) {
        /* Should we destroy it in RCU friendly way... ? */
        memory_region_del_subregion(phb->mr_mmio[index].container,
                                    &phb->mr_mmio[index]);
        phb->mmio_mapped[index] = false;
    }

    /* Get table entry */
    mbe0 = phb->ioda_MBT[(index << 1)];
    mbe1 = phb->ioda_MBT[(index << 1) + 1];

    if (!(mbe0 & IODA3_MBT0_ENABLE)) {
        return;
    }

    /* Grab geometry from registers */
    base = GETFIELD(IODA3_MBT0_BASE_ADDR, mbe0) << 12;
    size = GETFIELD(IODA3_MBT1_MASK, mbe1) << 12;
    size |= 0xff00000000000000ull;
    size = ~size + 1;

    /* Calculate PCI side start address based on M32/M64 window type */
    if (mbe0 & IODA3_MBT0_TYPE_M32) {
        start = phb->regs[PHB_M32_START_ADDR >> 3];
        if ((start + size) > 0x100000000ull) {
            phb4_error("M32 set beyond 4GB boundary !");
            size = 0x100000000 - start;
        }
    } else {
        start = base | (phb->regs[PHB_M64_UPPER_BITS >> 3]);
    }

    /* XX TODO: Figure out how to implemet/decode AOMASK */

    /* Check if it matches an enabled MMIO region in the PEC stack */
    if (phb->stack->mmio0_mapped && base >= phb->stack->mmio0_base &&
        (base + size) <= (phb->stack->mmio0_base + phb->stack->mmio0_size)) {
        parent = &phb->stack->mmbar0;
        base -= phb->stack->mmio0_base;
    } else if (phb->stack->mmio1_mapped && base >= phb->stack->mmio1_base &&
        (base + size) <= (phb->stack->mmio1_base + phb->stack->mmio1_size)) {
        parent = &phb->stack->mmbar1;
        base -= phb->stack->mmio1_base;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "PHB MBAR %d out of parent bounds", index);
        return;
    }

    /* Create alias XXX better name ? */
    snprintf(name, sizeof(name), "phb4-mbar%d", index);
    memory_region_init_alias(&phb->mr_mmio[index], OBJECT(phb), name,
                             &phb->pci_mmio, start, size);
    memory_region_add_subregion(parent, base, &phb->mr_mmio[index]);
    phb->mmio_mapped[index] = true;
}

static void pnv_phb4_check_all_mbt(PnvPHB4 *phb)
{
    uint64_t i;
    uint32_t num_windows = phb->big_phb ? PNV_PHB4_MAX_MMIO_WINDOWS :
        PNV_PHB4_MIN_MMIO_WINDOWS;

    for (i = 0; i < num_windows; i++) {
        pnv_phb4_check_mbt(phb, i);
    }
}

static uint64_t *pnv_phb4_ioda_access(PnvPHB4 *phb,
                                      unsigned *out_table, unsigned *out_idx)
{
    uint64_t adreg = phb->regs[PHB_IODA_ADDR >> 3];
    unsigned int index = GETFIELD(PHB_IODA_AD_TADR, adreg);
    unsigned int table = GETFIELD(PHB_IODA_AD_TSEL, adreg);
    unsigned int mask;
    uint64_t *tptr = NULL;

    switch (table) {
    case IODA3_TBL_LIST:
        tptr = phb->ioda_LIST;
        mask = 7;
        break;
    case IODA3_TBL_MIST:
        tptr = phb->ioda_MIST;
        mask = phb->big_phb ? PNV_PHB4_MAX_MIST : (PNV_PHB4_MAX_MIST >> 1);
        mask -= 1;
        break;
    case IODA3_TBL_RCAM:
        mask = phb->big_phb ? 127 : 63;
        break;
    case IODA3_TBL_MRT:
        mask = phb->big_phb ? 15 : 7;
        break;
    case IODA3_TBL_PESTA:
    case IODA3_TBL_PESTB:
        mask = phb->big_phb ? PNV_PHB4_MAX_PEs : (PNV_PHB4_MAX_PEs >> 1);
        mask -= 1;
        break;
    case IODA3_TBL_TVT:
        tptr = phb->ioda_TVT;
        mask = phb->big_phb ? PNV_PHB4_MAX_TVEs : (PNV_PHB4_MAX_TVEs >> 1);
        mask -= 1;
        break;
    case IODA3_TBL_TCR:
    case IODA3_TBL_TDR:
        mask = phb->big_phb ? 1023 : 511;
        break;
    case IODA3_TBL_MBT:
        tptr = phb->ioda_MBT;
        mask = phb->big_phb ? PNV_PHB4_MAX_MBEs : (PNV_PHB4_MAX_MBEs >> 1);
        mask -= 1;
        break;
    case IODA3_TBL_MDT:
        tptr = phb->ioda_MDT;
        mask = phb->big_phb ? PNV_PHB4_MAX_PEs : (PNV_PHB4_MAX_PEs >> 1);
        mask -= 1;
        break;
    case IODA3_TBL_PEEV:
        tptr = phb->ioda_PEEV;
        mask = phb->big_phb ? PNV_PHB4_MAX_PEEVs : (PNV_PHB4_MAX_PEEVs >> 1);
        mask -= 1;
        break;
    default:
        phb4_error("invalid IODA table %d", table);
        return NULL;
    }
    index &= mask;
    if (out_idx) {
        *out_idx = index;
    }
    if (out_table) {
        *out_table = table;
    }
    if (tptr) {
        tptr += index;
    }
    if (adreg & PHB_IODA_AD_AUTOINC) {
        index = (index + 1) & mask;
        adreg = SETFIELD(PHB_IODA_AD_TADR, adreg, index);
    }

    phb->regs[PHB_IODA_ADDR >> 3] = adreg;
    return tptr;
}

static uint64_t pnv_phb4_ioda_read(PnvPHB4 *phb)
{
    unsigned table, idx;
    uint64_t *tptr;

    tptr = pnv_phb4_ioda_access(phb, &table, &idx);
    if (!tptr) {
        /* Special PESTA case */
        if (table == IODA3_TBL_PESTA) {
            return ((uint64_t)(phb->ioda_PEST_AB[idx] & 1)) << 63;
        } else if (table == IODA3_TBL_PESTB) {
            return ((uint64_t)(phb->ioda_PEST_AB[idx] & 2)) << 62;
        }
        /* Return 0 on unsupported tables, not ff's */
        return 0;
    }
    return *tptr;
}

static void pnv_phb4_ioda_write(PnvPHB4 *phb, uint64_t val)
{
    unsigned table, idx;
    uint64_t *tptr;

    tptr = pnv_phb4_ioda_access(phb, &table, &idx);
    if (!tptr) {
        /* Special PESTA case */
        if (table == IODA3_TBL_PESTA) {
            phb->ioda_PEST_AB[idx] &= ~1;
            phb->ioda_PEST_AB[idx] |= (val >> 63) & 1;
        } else if (table == IODA3_TBL_PESTB) {
            phb->ioda_PEST_AB[idx] &= ~2;
            phb->ioda_PEST_AB[idx] |= (val >> 62) & 2;
        }
        return;
    }

    /* Handle side effects */
    switch (table) {
    case IODA3_TBL_LIST:
        //pnv_phb4_lxivt_write(phb, idx, val);
        break;
    case IODA3_TBL_MIST: {
        /* Special mask for MIST partial write */
        uint64_t adreg = phb->regs[PHB_IODA_ADDR >> 3];
        uint32_t mmask = GETFIELD(PHB_IODA_AD_MIST_PWV, adreg);
        uint64_t v = *tptr;
        if (mmask == 0) {
            mmask = 0xf;
        }
        if (mmask & 8) {
            v &= 0x0000ffffffffffffull;
            v |= 0xcfff000000000000ull & val;
        }
        if (mmask & 4) {
            v &= 0xffff0000ffffffffull;
            v |= 0x0000cfff00000000ull & val;
        }
        if (mmask & 2) {
            v &= 0xffffffff0000ffffull;
            v |= 0x00000000cfff0000ull & val;
        }
        if (mmask & 1) {
            v &= 0xffffffffffff0000ull;
            v |= 0x000000000000cfffull & val;
        }
        *tptr = val;
        break;
    }
    case IODA3_TBL_MBT:
        *tptr = val;

        /* Copy accross the valid bit to the other half */
        phb->ioda_MBT[idx ^ 1] &= 0x7fffffffffffffffull;
        phb->ioda_MBT[idx ^ 1] |= 0x8000000000000000ull & val;

        /* Update mappings */
        pnv_phb4_check_mbt(phb, idx >> 1);
        break;
    default:
        *tptr = val;
    }
}

static void pnv_phb4_rtc_invalidate(PnvPHB4 *phb, uint64_t val)
{
    PnvPhb4DMASpace *ds;

    /* Always invalidate all for now ... */
    QLIST_FOREACH(ds, &phb->dma_spaces, list) {
        ds->pe_num = PHB_INVALID_PE;
    }
}

static void pnv_phb4_update_msi_regions(PnvPhb4DMASpace *ds)
{
    uint64_t cfg = ds->phb->regs[PHB_PHB4_CONFIG >> 3];

    if (cfg & PHB_PHB4C_32BIT_MSI_EN) {
        if (!ds->msi32_mapped) {
            memory_region_add_subregion(MEMORY_REGION(&ds->dma_mr),
                                        0xffff0000, &ds->msi32_mr);
            ds->msi32_mapped = true;
        }
    } else {
        if (ds->msi32_mapped) {
            memory_region_del_subregion(MEMORY_REGION(&ds->dma_mr),
                                        &ds->msi32_mr);
            ds->msi32_mapped = false;
        }
    }

    if (cfg & PHB_PHB4C_64BIT_MSI_EN) {
        if (!ds->msi64_mapped) {
            memory_region_add_subregion(MEMORY_REGION(&ds->dma_mr),
                                        (1ull << 60), &ds->msi64_mr);
            ds->msi64_mapped = true;
        }
    } else {
        if (ds->msi64_mapped) {
            memory_region_del_subregion(MEMORY_REGION(&ds->dma_mr),
                                        &ds->msi64_mr);
            ds->msi64_mapped = false;
        }
    }
}

static void pnv_phb4_update_all_msi_regions(PnvPHB4 *phb)
{
    PnvPhb4DMASpace *ds;

    QLIST_FOREACH(ds, &phb->dma_spaces, list) {
        pnv_phb4_update_msi_regions(ds);
    }
}

static void pnv_phb4_update_xsrc(PnvPHB4 *phb)
{
    int shift, flags, i, id_base;
    XiveSource *xsrc = &phb->xsrc;

    if (phb->regs[PHB_CTRLR >> 3] & PHB_CTRLR_IRQ_PGSZ_64K) {
        shift = XIVE_ESB_64K;
    } else {
        shift = XIVE_ESB_4K;
    }
    if (phb->regs[PHB_CTRLR >> 3] & PHB_CTRLR_IRQ_STORE_EOI) {
        flags = XIVE_SRC_STORE_EOI;
    } else {
        flags = 0;
    }
    // XXX
    //object_property_set_int(OBJECT(xsrc), shift, "shift", &error_fatal);
    //object_property_set_int(OBJECT(xsrc), flags, "flags", &error_fatal);
    phb->xsrc.esb_shift = shift;
    phb->xsrc.esb_flags = flags;

    id_base = GETFIELD(PHB_LSI_SOURCE_ID, phb->regs[PHB_LSI_SOURCE_ID >> 3]);
    id_base <<= 3;

    for (i = 0; i < xsrc->nr_irqs; i++) {
        bool lsi = (i < id_base || i >= (id_base + 8));
        if (lsi) {
            xive_source_irq_set_lsi(xsrc, i);
        }
    }
}

static void pnv_phb4_reg_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    PnvPHB4 *phb = opaque;
    bool changed;

    /* Special case outbound configuration data */
    if ((off & 0xfffc) == PHB_CONFIG_DATA) {
        pnv_phb4_config_write(phb, off & 0x3, size, val);
        return;
    }

    /* Special case RC configuration space */
    if ((off & 0xf800) == PHB_RC_CONFIG_BASE) {
        pnv_phb4_rc_config_write(phb, off & 0x7ff, size, val);
        return;
    }

    /* Other registers are 64-bit only */
    if (size != 8 || off & 0x7) {
        phb4_error("Invalid register access, offset: 0x%"PRIx64" size: %d",
                   off, size);
        return;
    }

    /* Handle masking */
    switch (off) {
    case PHB_LSI_SOURCE_ID:
        val &= PHB_LSI_SRC_ID;
        break;
    case PHB_M64_UPPER_BITS:
        val &= 0xff00000000000000ull;
        break;
    /* TCE Kill */
    case PHB_TCE_KILL:
        /* Clear top 3 bits which HW does to indicate successful queuing */
        val &= ~(PHB_TCE_KILL_ALL | PHB_TCE_KILL_PE | PHB_TCE_KILL_ONE);
        break;
    case PHB_Q_DMA_R:
        /* This is enough logic to make SW happy but we aren't actually
         * quiescing the DMAs
         */
        if (val & PHB_Q_DMA_R_AUTORESET) {
            val = 0;
        } else {
            val &= PHB_Q_DMA_R_QUIESCE_DMA;
        }
        break;
    /* LEM stuff */
    case PHB_LEM_FIR_AND_MASK:
        phb->regs[PHB_LEM_FIR_ACCUM >> 3] &= val;
        return;
    case PHB_LEM_FIR_OR_MASK:
        phb->regs[PHB_LEM_FIR_ACCUM >> 3] |= val;
        return;
    case PHB_LEM_ERROR_AND_MASK:
        phb->regs[PHB_LEM_ERROR_MASK >> 3] &= val;
        return;
    case PHB_LEM_ERROR_OR_MASK:
        phb->regs[PHB_LEM_ERROR_MASK >> 3] |= val;
        return;
    case PHB_LEM_WOF:
        val = 0;
        break;
    // XXX TODO: More..., maybe create a table with masks...
     /* Read only registers */
    case PHB_CPU_LOADSTORE_STATUS:
    case PHB_ETU_ERR_SUMMARY:
    case PHB_PHB4_GEN_CAP:
    case PHB_PHB4_TCE_CAP:
    case PHB_PHB4_IRQ_CAP:
    case PHB_PHB4_EEH_CAP:
        return;
    }

    /* Record whether it changed */
    changed = phb->regs[off >> 3] != val;

    /* Store in register cache first */
    phb->regs[off >> 3] = val;

    /* Handle side effects */
    switch (off) {
    case PHB_PHB4_CONFIG:
        if (changed) {
            pnv_phb4_update_all_msi_regions(phb);
        }
        break;
    case PHB_M32_START_ADDR:
    case PHB_M64_UPPER_BITS:
        if (changed) {
            pnv_phb4_check_all_mbt(phb);
        }
        break;

    /* IODA table accesses */
    case PHB_IODA_DATA0:
        pnv_phb4_ioda_write(phb, val);
        break;

    /* RTC invalidation */
    case PHB_RTC_INVALIDATE:
        pnv_phb4_rtc_invalidate(phb, val);
        break;

    /* PHB Control (Affects XIVE source) */
    case PHB_CTRLR:
    case PHB_LSI_SOURCE_ID:
        pnv_phb4_update_xsrc(phb);
        break;

    /* Silent simple writes */
    case PHB_ASN_CMPM:
    case PHB_CONFIG_ADDRESS:
    case PHB_IODA_ADDR:
    case PHB_TCE_KILL:
    case PHB_TCE_SPEC_CTL:
    case PHB_PEST_BAR:
    case PHB_PELTV_BAR:
    case PHB_RTT_BAR:
    case PHB_LEM_FIR_ACCUM:
    case PHB_LEM_ERROR_MASK:
    case PHB_LEM_ACTION0:
    case PHB_LEM_ACTION1:
    case PHB_TCE_TAG_ENABLE:
    case PHB_INT_NOTIFY_ADDR:
    case PHB_INT_NOTIFY_INDEX:
    case PHB_DMARD_SYNC:
       break;

    /* Noise on anything else */
    default:
        qemu_log_mask(LOG_UNIMP, "phb4: reg_write 0x%"PRIx64"=%"PRIx64"\n",
                      off, val);
    }
}

static uint64_t pnv_phb4_reg_read(void *opaque, hwaddr off, unsigned size)
{
    PnvPHB4 *phb = opaque;
    uint64_t val;

    if ((off & 0xfffc) == PHB_CONFIG_DATA) {
        return pnv_phb4_config_read(phb, off & 0x3, size);
    }

    /* Special case RC configuration space */
    if ((off & 0xf800) == PHB_RC_CONFIG_BASE) {
        return pnv_phb4_rc_config_read(phb, off & 0x7ff, size);
    }

    /* Other registers are 64-bit only */
    if (size != 8 || off & 0x7) {
        phb4_error("Invalid register access, offset: 0x%"PRIx64" size: %d",
                   off, size);
        return ~0ull;
    }

    /* Default read from cache */
    val = phb->regs[off >> 3];

    switch (off) {
    case PHB_VERSION:
        /* XXX Make that some kind of parameter */
        return 0x000000a400000002ull;
//    case PHB_PCIE_SYSTEM_CONFIG:
//        return 0x441100fc30000000;

        /* Read-only */
    case PHB_PHB4_GEN_CAP:
        return 0xe4b8000000000000ull;
    case PHB_PHB4_TCE_CAP:
        return phb->big_phb ? 0x4008440000000400ull : 0x2008440000000200ull;
    case PHB_PHB4_IRQ_CAP:
        return phb->big_phb ? 0x0800000000001000ull : 0x0800000000000800ull;
    case PHB_PHB4_EEH_CAP:
        return phb->big_phb ? 0x2000000000000000ull : 0x1000000000000000ull;

    /* IODA table accesses */
    case PHB_IODA_DATA0:
        return pnv_phb4_ioda_read(phb);

    /* Link training always appears trained */
    case PHB_PCIE_DLP_TRAIN_CTL:
        // XXX Do something sensible with speed ? */
        return PHB_PCIE_DLP_INBAND_PRESENCE | PHB_PCIE_DLP_TL_LINKACT;

    /* DMA read sync: make it look like it's complete */
    case PHB_DMARD_SYNC:
        return PHB_DMARD_SYNC_COMPLETE;

    /* Silent simple reads */
    case PHB_LSI_SOURCE_ID:
    case PHB_CPU_LOADSTORE_STATUS:
    case PHB_ASN_CMPM:
    case PHB_PHB4_CONFIG:
    case PHB_M32_START_ADDR:
    case PHB_CONFIG_ADDRESS:
    case PHB_IODA_ADDR:
    case PHB_RTC_INVALIDATE:
    case PHB_TCE_KILL:
    case PHB_TCE_SPEC_CTL:
    case PHB_PEST_BAR:
    case PHB_PELTV_BAR:
    case PHB_RTT_BAR:
    case PHB_M64_UPPER_BITS:
    case PHB_CTRLR:
    case PHB_LEM_FIR_ACCUM:
    case PHB_LEM_ERROR_MASK:
    case PHB_LEM_ACTION0:
    case PHB_LEM_ACTION1:
    case PHB_TCE_TAG_ENABLE:
    case PHB_INT_NOTIFY_ADDR:
    case PHB_INT_NOTIFY_INDEX:
    case PHB_Q_DMA_R:
    case PHB_ETU_ERR_SUMMARY:
        break;

    /* Noise on anything else */
    default:
        qemu_log_mask(LOG_UNIMP, "phb4: reg_read 0x%"PRIx64"=%"PRIx64"\n",
                      off, val);
    }
    return val;
}

static const MemoryRegionOps pnv_phb4_reg_ops = {
    .read = pnv_phb4_reg_read,
    .write = pnv_phb4_reg_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t pnv_phb4_xscom_read(void *opaque, hwaddr addr, unsigned size)
{
    PnvPHB4 *phb = PNV_PHB4(opaque);
    uint32_t reg = addr >> 3;
    uint64_t val;
    hwaddr offset;

    switch(reg) {
    case PHB_SCOM_HV_IND_ADDR:
        return phb->scom_hv_ind_addr_reg;
        break;
    case PHB_SCOM_HV_IND_DATA:
        if (!(phb->scom_hv_ind_addr_reg & PHB_SCOM_HV_IND_ADDR_VALID)) {
            /* XXX Set error */
            return ~0ull;
        }
        size = (phb->scom_hv_ind_addr_reg & PHB_SCOM_HV_IND_ADDR_4B) ? 4 : 8;
        offset = GETFIELD(PHB_SCOM_HV_IND_ADDR_ADDR, phb->scom_hv_ind_addr_reg);
        val = pnv_phb4_reg_read(phb, offset, size);
        if (phb->scom_hv_ind_addr_reg & PHB_SCOM_HV_IND_ADDR_AUTOINC) {
            offset += size;
            offset &= 0x3fff;
            phb->scom_hv_ind_addr_reg = SETFIELD(PHB_SCOM_HV_IND_ADDR_ADDR,
                                                 phb->scom_hv_ind_addr_reg, offset);
        }
        return val;
    case PHB_SCOM_ETU_LEM_FIR:
    case PHB_SCOM_ETU_LEM_FIR_AND:
    case PHB_SCOM_ETU_LEM_FIR_OR:
    case PHB_SCOM_ETU_LEM_FIR_MSK:
    case PHB_SCOM_ETU_LEM_ERR_MSK_AND:
    case PHB_SCOM_ETU_LEM_ERR_MSK_OR:
    case PHB_SCOM_ETU_LEM_ACT0:
    case PHB_SCOM_ETU_LEM_ACT1:
    case PHB_SCOM_ETU_LEM_WOF:
        offset = ((reg - PHB_SCOM_ETU_LEM_FIR) << 3) + PHB_LEM_FIR_ACCUM;
        return pnv_phb4_reg_read(phb, offset, size);
    case PHB_SCOM_ETU_PMON_CONFIG:
    case PHB_SCOM_ETU_PMON_CTR0:
    case PHB_SCOM_ETU_PMON_CTR1:
    case PHB_SCOM_ETU_PMON_CTR2:
    case PHB_SCOM_ETU_PMON_CTR3:
        offset = ((reg - PHB_SCOM_ETU_PMON_CONFIG) << 3) + PHB_PERFMON_CONFIG;
        return pnv_phb4_reg_read(phb, offset, size);
    }

    /* XXX Set error ? */
    return ~0ull;
}

static void pnv_phb4_xscom_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    PnvPHB4 *phb = PNV_PHB4(opaque);
    uint32_t reg = addr >> 3;
    hwaddr offset;

    switch(reg) {
    case PHB_SCOM_HV_IND_ADDR:
        phb->scom_hv_ind_addr_reg = val & 0xe000000000001fff;
        break;
    case PHB_SCOM_HV_IND_DATA:
        if (!(phb->scom_hv_ind_addr_reg & PHB_SCOM_HV_IND_ADDR_VALID)) {
            /* XXX Set error */
            break;
        }
        size = (phb->scom_hv_ind_addr_reg & PHB_SCOM_HV_IND_ADDR_4B) ? 4 : 8;
        offset = GETFIELD(PHB_SCOM_HV_IND_ADDR_ADDR, phb->scom_hv_ind_addr_reg);
        pnv_phb4_reg_write(phb, offset, val, size);
        if (phb->scom_hv_ind_addr_reg & PHB_SCOM_HV_IND_ADDR_AUTOINC) {
            offset += size;
            offset &= 0x3fff;
            phb->scom_hv_ind_addr_reg = SETFIELD(PHB_SCOM_HV_IND_ADDR_ADDR,
                                                 phb->scom_hv_ind_addr_reg, offset);
        }
        break;
    case PHB_SCOM_ETU_LEM_FIR:
    case PHB_SCOM_ETU_LEM_FIR_AND:
    case PHB_SCOM_ETU_LEM_FIR_OR:
    case PHB_SCOM_ETU_LEM_FIR_MSK:
    case PHB_SCOM_ETU_LEM_ERR_MSK_AND:
    case PHB_SCOM_ETU_LEM_ERR_MSK_OR:
    case PHB_SCOM_ETU_LEM_ACT0:
    case PHB_SCOM_ETU_LEM_ACT1:
    case PHB_SCOM_ETU_LEM_WOF:
        offset = ((reg - PHB_SCOM_ETU_LEM_FIR) << 3) + PHB_LEM_FIR_ACCUM;
        pnv_phb4_reg_write(phb, offset, val, size);
        break;
    case PHB_SCOM_ETU_PMON_CONFIG:
    case PHB_SCOM_ETU_PMON_CTR0:
    case PHB_SCOM_ETU_PMON_CTR1:
    case PHB_SCOM_ETU_PMON_CTR2:
    case PHB_SCOM_ETU_PMON_CTR3:
        offset = ((reg - PHB_SCOM_ETU_PMON_CONFIG) << 3) + PHB_PERFMON_CONFIG;
        pnv_phb4_reg_write(phb, offset, val, size);
        break;
    }
}

static const MemoryRegionOps pnv_phb4_xscom_ops = {
    .read = pnv_phb4_xscom_read,
    .write = pnv_phb4_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static int pnv_phb4_map_irq(PCIDevice *pci_dev, int irq_num)
{
    /* Check that out properly ... */
    return irq_num & 3;
}

static void pnv_phb4_set_irq(void *opaque, int irq_num, int level)
{
    PnvPHB4 *phb = opaque;
    uint32_t id_base;

    /* LSI only ... */
    if (irq_num > 3) {
        phb4_error("Unknown IRQ to set %d", irq_num);
    }
    id_base = GETFIELD(PHB_LSI_SOURCE_ID, phb->regs[PHB_LSI_SOURCE_ID >> 3]);
    id_base <<= 3;
    qemu_set_irq(phb->qirqs[id_base + irq_num], level);
}

static bool pnv_phb4_resolve_pe(PnvPhb4DMASpace *ds)
{
    uint64_t rtt, addr;
    uint16_t rte;
    int bus_num;
    int num_PEs;

    /* Already resolved ? */
    if (ds->pe_num != PHB_INVALID_PE) {
        return true;
    }

    /* We need to lookup the RTT */
    rtt = ds->phb->regs[PHB_RTT_BAR >> 3];
    if (!(rtt & PHB_RTT_BAR_ENABLE)) {
        phb4_error("DMA with RTT BAR disabled !");
        /* Set error bits ? fence ? ... */
        return false;
    }

    /* Read RTE */
    bus_num = pci_bus_num(ds->bus);
    addr = rtt & PHB_RTT_BASE_ADDRESS_MASK;
    addr += 2 * ((bus_num << 8) | ds->devfn);
    if (dma_memory_read(&address_space_memory, addr, &rte, sizeof(rte))) {
        phb4_error("Failed to read RTT entry at 0x%"PRIx64, addr);
        /* Set error bits ? fence ? ... */
        return false;
    }
    rte = be16_to_cpu(rte);

    /* Fail upon reading of invalid PE# */
    num_PEs = ds->phb->big_phb ? PNV_PHB4_MAX_PEs : (PNV_PHB4_MAX_PEs >> 1);
    if (rte >= num_PEs) {
        phb4_error("RTE for RID 0x%x invalid (%04x", ds->devfn, rte);
        rte &= num_PEs - 1;
    }
    ds->pe_num = rte;
    return true;
}

static void pnv_phb4_translate_tve(PnvPhb4DMASpace *ds, hwaddr addr,
                                   bool is_write, uint64_t tve,
                                   IOMMUTLBEntry *tlb)
{
    uint64_t tta = GETFIELD(IODA3_TVT_TABLE_ADDR, tve);
    int32_t  lev = GETFIELD(IODA3_TVT_NUM_LEVELS, tve);
    uint32_t tts = GETFIELD(IODA3_TVT_TCE_TABLE_SIZE, tve);
    uint32_t tps = GETFIELD(IODA3_TVT_IO_PSIZE, tve);
//    bool nt50 = !!(tve & IODA3_TVT_NON_TRANSLATE_50);

    /* Invalid levels */
    if (lev > 4) {
        phb4_error("Invalid #levels in TVE %d", lev);
        return;
    }

    /* Invalid entry */
    if (tts == 0) {
        phb4_error("Access to invalid TVE");
        return;
    }

    /* IO Page Size of 0 means untranslated, else use TCEs */
    if (tps == 0) {
        /* XXX Handle boundaries */

        /* XXX Use 4k pages like q35 ... for now */
        tlb->iova = addr & 0xfffffffffffff000ull;
        tlb->translated_addr = addr & 0x0003fffffffff000ull;
        tlb->addr_mask = 0xfffull;
        tlb->perm = IOMMU_RW;
    } else {
        uint32_t tce_shift, tbl_shift, sh;
        uint64_t base, taddr, tce, tce_mask;

        /* Address bits per bottom level TCE entry */
        tce_shift = tps + 11;

        /* Address bits per table level */
        tbl_shift = tts + 8;

        /* Top level table base address */
        base = tta << 12;

        /* Total shift to first level */
        sh = tbl_shift * lev + tce_shift;

        // XX Limit to support IO page sizes

        /* XXX Multi-level untested */
        while ((lev--) >= 0) {
            /* Grab the TCE address */
            taddr = base | (((addr >> sh) & ((1ul << tbl_shift) - 1)) << 3);
            if (dma_memory_read(&address_space_memory, taddr, &tce,
                                sizeof(tce))) {
                phb4_error("Failed to read TCE at 0x%"PRIx64, taddr);
                return;
            }
            tce = be64_to_cpu(tce);

            /* Check permission for indirect TCE */
            if ((lev >= 0) && !(tce & 3)) {
                phb4_error("Invalid indirect TCE at 0x%"PRIx64, taddr);
                phb4_error(" xlate %"PRIx64":%c TVE=%"PRIx64, addr,
                           is_write ? 'W' : 'R', tve);
                phb4_error(" tta=%"PRIx64" lev=%d tts=%d tps=%d",
                           tta, lev, tts, tps);
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
            phb4_error("TCE access fault at 0x%"PRIx64, taddr);
            phb4_error(" xlate %"PRIx64":%c TVE=%"PRIx64, addr,
                       is_write ? 'W' : 'R', tve);
            phb4_error(" tta=%"PRIx64" lev=%d tts=%d tps=%d",
                       tta, lev, tts, tps);
        }
    }
}

static IOMMUTLBEntry pnv_phb4_translate_iommu(IOMMUMemoryRegion *iommu,
                                              hwaddr addr,
                                              IOMMUAccessFlags flag,
                                              int iommu_idx)
{
    PnvPhb4DMASpace *ds = container_of(iommu, PnvPhb4DMASpace, dma_mr);
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
    if (!pnv_phb4_resolve_pe(ds)) {
        phb4_error("Failed to resolve PE# for bus @%p (%d) devfn 0x%x",
                   ds->bus, pci_bus_num(ds->bus), ds->devfn);
        return ret;
    }

    /* Check top bits */
    switch (addr >> 60) {
    case 00:
        /* DMA or 32-bit MSI ? */
        cfg = ds->phb->regs[PHB_PHB4_CONFIG >> 3];
        if ((cfg & PHB_PHB4C_32BIT_MSI_EN) &&
            ((addr & 0xffffffffffff0000ull) == 0xffff0000ull)) {
            phb4_error("xlate on 32-bit MSI region");
            return ret;
        }
        /* Choose TVE XXX Use PHB4 Control Register */
        tve_sel = (addr >> 59) & 1;
        tve = ds->phb->ioda_TVT[ds->pe_num * 2 + tve_sel];
        pnv_phb4_translate_tve(ds, addr, flag & IOMMU_WO, tve, &ret);
        break;
    case 01:
        phb4_error("xlate on 64-bit MSI region");
        break;
    default:
        phb4_error("xlate on unsupported address 0x%"PRIx64, addr);
    }
    return ret;
}

#define TYPE_PNV_PHB4_IOMMU_MEMORY_REGION "pnv-phb4-iommu-memory-region"
#define PNV_PHB4_IOMMU_MEMORY_REGION(obj) \
    OBJECT_CHECK(IOMMUMemoryRegion, (obj), TYPE_PNV_PHB4_IOMMU_MEMORY_REGION)

static void pnv_phb4_iommu_memory_region_class_init(ObjectClass *klass,
                                                    void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = pnv_phb4_translate_iommu;
}

static const TypeInfo pnv_phb4_iommu_memory_region_info = {
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .name = TYPE_PNV_PHB4_IOMMU_MEMORY_REGION,
    .class_init = pnv_phb4_iommu_memory_region_class_init,
};

/*
 * MSI/MSIX memory region implementation.
 * The handler handles both MSI and MSIX.
 */
static void pnv_phb4_msi_write(void *opaque, hwaddr addr,
                               uint64_t data, unsigned size)
{
    PnvPhb4DMASpace *ds = opaque;
    PnvPHB4 *phb = ds->phb;

    uint32_t src = ((addr >> 4) & 0xffff) | (data & 0x1f);

    /* Resolve PE# */
    if (!pnv_phb4_resolve_pe(ds)) {
        phb4_error("Failed to resolve PE# for bus @%p (%d) devfn 0x%x",
                   ds->bus, pci_bus_num(ds->bus), ds->devfn);
        return;
    }

    // XXX Check it doesn't collide with LSIs */
    if (src >= phb->xsrc.nr_irqs) {
        qemu_log_mask(LOG_GUEST_ERROR, "MSI %d out of bounds", src);
        return;
    }

    if (ds->pe_num >= 0) {
        // XXX PE check
#if 0
        if (pe != dev_pe) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "MSI %d send by PE#%d but assigned to PE#%d",
                          src, dev_pe, pe);
            return;
        }
#endif
    }
    qemu_irq_pulse(phb->qirqs[src]);
}

/* There is no .read as the read result is undefined by PCI spec */
static uint64_t pnv_phb4_msi_read(void *opaque, hwaddr addr, unsigned size)
{
    phb4_error("Invalid MSI read @ 0x%" HWADDR_PRIx, addr);
    return -1;
}

static const MemoryRegionOps pnv_phb4_msi_ops = {
    .read = pnv_phb4_msi_read,
    .write = pnv_phb4_msi_write,
    .endianness = DEVICE_LITTLE_ENDIAN
};

static AddressSpace *pnv_phb4_dma_iommu(PCIBus *bus, void *opaque, int devfn)
{
    PnvPHB4 *phb = opaque;
    PnvPhb4DMASpace *ds;

    QLIST_FOREACH(ds, &phb->dma_spaces, list) {
        if (ds->bus == bus && ds->devfn == devfn) {
            break;
        }
    }

    if (ds == NULL) {
        ds = g_malloc0(sizeof(PnvPhb4DMASpace));
        ds->bus = bus;
        ds->devfn = devfn;
        ds->pe_num = PHB_INVALID_PE;
        ds->phb = phb;
        memory_region_init_iommu(&ds->dma_mr, sizeof(ds->dma_mr),
                                 TYPE_PNV_PHB4_IOMMU_MEMORY_REGION,
                                 OBJECT(phb), "phb4-iommu", UINT64_MAX);
        address_space_init(&ds->dma_as, MEMORY_REGION(&ds->dma_mr),
                           "phb4_iommu");
        memory_region_init_io(&ds->msi32_mr, OBJECT(phb), &pnv_phb4_msi_ops,
                              ds, "msi32", 0x10000);
        memory_region_init_io(&ds->msi64_mr, OBJECT(phb), &pnv_phb4_msi_ops,
                              ds, "msi64", 0x100000);
        pnv_phb4_update_msi_regions(ds);

        QLIST_INSERT_HEAD(&phb->dma_spaces, ds, list);
    }
    return &ds->dma_as;
}

static void pnv_phb4_instance_init(Object *obj)
{
    PnvPHB4 *phb = PNV_PHB4(obj);

    QLIST_INIT(&phb->dma_spaces);

    /* XIVE interrupt source object */
    object_initialize(&phb->xsrc, sizeof(XiveSource), TYPE_XIVE_SOURCE);
    object_property_add_child(obj, "source", OBJECT(&phb->xsrc), NULL);

    /* Root Port */
    object_initialize(&phb->root, sizeof(phb->root), TYPE_PNV_PHB4_ROOT_PORT);
    object_property_add_child(obj, "root", OBJECT(&phb->root), NULL);
    qdev_prop_set_int32(DEVICE(&phb->root), "addr", PCI_DEVFN(0, 0));
    qdev_prop_set_bit(DEVICE(&phb->root), "multifunction", false);
}

static void pnv_phb4_realize(DeviceState *dev, Error **errp)
{
    PnvPHB4 *phb = PNV_PHB4(dev);
    PCIHostState *pci = PCI_HOST_BRIDGE(dev);
    XiveSource *xsrc = &phb->xsrc;
    Error *local_err = NULL;
    int nr_irqs;

    /* Attach to PEC and reparent */
    pnv_phb4_pec_attach(phb, &pnv_phb4_xscom_ops, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    qdev_set_parent_bus(dev, sysbus_get_default());

    /* Controller Registers */
    memory_region_init_io(&phb->mr_regs, OBJECT(phb), &pnv_phb4_reg_ops, phb,
                          "phb4-regs", 0x2000);

    /* PHB4 doesn't support IO space. However, qemu gets very upset if
     * we don't have an IO region to anchor IO BARs onto so we just
     * initialize one which we never hook up to anything
     */
    // XXX Make names unique
    memory_region_init(&phb->pci_io, OBJECT(phb), "pci-io", 0x10000);
    memory_region_init(&phb->pci_mmio, OBJECT(phb), "pci-mmio",
                       PCI_MMIO_TOTAL_SIZE);

    pci->bus = pci_register_root_bus(dev, "root-bus",
                                     pnv_phb4_set_irq, pnv_phb4_map_irq, phb,
                                     &phb->pci_mmio, &phb->pci_io,
                                     0, 4, TYPE_PNV_PHB4_ROOT_BUS);

    pci_setup_iommu(pci->bus, pnv_phb4_dma_iommu, phb);

    /* Add a single Root port */
    qdev_prop_set_uint8(DEVICE(&phb->root), "chassis", phb->chip_id);
    qdev_prop_set_uint16(DEVICE(&phb->root), "slot", phb->phb_id);
    qdev_set_parent_bus(DEVICE(&phb->root), BUS(pci->bus));
    qdev_init_nofail(DEVICE(&phb->root));

    /* Setup XIVE Source */
    if (phb->big_phb) {
        nr_irqs = PNV_PHB4_MAX_INTs;
    } else {
        nr_irqs = PNV_PHB4_MAX_INTs >> 1;
    }
    object_property_set_int(OBJECT(xsrc), nr_irqs, "nr-irqs", &error_fatal);
    object_property_add_const_link(OBJECT(xsrc), "xive", OBJECT(phb), &error_fatal);
    object_property_set_bool(OBJECT(xsrc), true, "realized", &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    pnv_phb4_update_xsrc(phb);
    qdev_set_parent_bus(DEVICE(xsrc), sysbus_get_default());

    phb->qirqs = qemu_allocate_irqs(xive_source_set_irq, xsrc, xsrc->nr_irqs);
}

void pnv_phb4_update_regions(PnvPHB4 *phb)
{
    PnvPhb4PecStack *stack = phb->stack;

    /* Unmap first always */
    if (phb->regs_mapped) {
        memory_region_del_subregion(&stack->phbbar, &phb->mr_regs);
        phb->regs_mapped = false;
    }
    if (phb->esb_mapped) {
        memory_region_del_subregion(&stack->intbar, &phb->xsrc.esb_mmio);
        phb->esb_mapped = false;
    }

    /* Map registers if enabled */
    if (stack->phb_mapped) {
        memory_region_add_subregion(&stack->phbbar, 0, &phb->mr_regs);
        phb->regs_mapped = true;
    }

    /* Map ESB if enabled */
    if (stack->int_mapped) {
        memory_region_add_subregion(&stack->intbar, 0, &phb->xsrc.esb_mmio);
        phb->esb_mapped = true;
    }

    /* Check/update m32 */
    pnv_phb4_check_all_mbt(phb);
}

static const char *pnv_phb4_root_bus_path(PCIHostState *host_bridge,
                                          PCIBus *rootbus)
{
    PnvPHB4 *phb = PNV_PHB4(host_bridge);

    snprintf(phb->bus_path, sizeof(phb->bus_path), "00%02x:%02x",
             phb->chip_id, phb->phb_id);
    return phb->bus_path;
}

static void pnv_phb4_xive_notify(XiveNotifier *xf, uint32_t srcno)
{
    PnvPHB4 *phb = PNV_PHB4(xf);
    uint64_t notif_port = phb->regs[PHB_INT_NOTIFY_ADDR >> 3];
    uint32_t offset = phb->regs[PHB_INT_NOTIFY_INDEX >> 3];
    uint64_t lisn = cpu_to_be64(offset | srcno);

    cpu_physical_memory_write(notif_port, &lisn, sizeof(lisn));
}

static Property pnv_phb4_properties[] = {
        DEFINE_PROP_UINT32("index", PnvPHB4, phb_id, 0),
        DEFINE_PROP_UINT32("chip-id", PnvPHB4, chip_id, 0),
        DEFINE_PROP_END_OF_LIST(),
};

static void pnv_phb4_class_init(ObjectClass *klass, void *data)
{
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    XiveNotifierClass *xfc = XIVE_NOTIFIER_CLASS(klass);

    hc->root_bus_path   = pnv_phb4_root_bus_path;
    dc->realize         = pnv_phb4_realize;
    dc->props           = pnv_phb4_properties;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->user_creatable  = true;
    xfc->notify         = pnv_phb4_xive_notify;
}

static const TypeInfo pnv_phb4_type_info = {
    .name          = TYPE_PNV_PHB4,
    .parent        = TYPE_PCIE_HOST_BRIDGE,
    .instance_size = sizeof(PnvPHB4),
    .class_init    = pnv_phb4_class_init,
    .instance_init = pnv_phb4_instance_init,
    .interfaces = (InterfaceInfo[]) {
            { TYPE_XIVE_NOTIFIER },
            { },
    }
};

static void pnv_phb4_root_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    /*
     * PHB4 has only a single root complex. Enforce the limit on the
     * parent bus
     */
    k->max_dev = 1;
}

static const TypeInfo pnv_phb4_root_bus_info = {
    .name = TYPE_PNV_PHB4_ROOT_BUS,
    .parent = TYPE_PCIE_BUS,
    .class_init = pnv_phb4_root_bus_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void pnv_phb4_root_port_reset(DeviceState *qdev)
{
    PCIDevice *d = PCI_DEVICE(qdev);
    uint8_t *conf = d->config;

    pcie_root_port_reset(qdev);

    pci_byte_test_and_set_mask(conf + PCI_IO_BASE,
                               PCI_IO_RANGE_MASK & 0xff);
    pci_byte_test_and_clear_mask(conf + PCI_IO_LIMIT,
                                 PCI_IO_RANGE_MASK & 0xff);
    pci_set_word(conf + PCI_MEMORY_BASE, 0);
    pci_set_word(conf + PCI_MEMORY_LIMIT, 0xfff0);
    pci_set_word(conf + PCI_PREF_MEMORY_BASE, 0x1);
    pci_set_word(conf + PCI_PREF_MEMORY_LIMIT, 0xfff1);
    pci_set_long(conf + PCI_PREF_BASE_UPPER32, 0x1); // Hack
    pci_set_long(conf + PCI_PREF_LIMIT_UPPER32, 0xffffffff);
}

static void pnv_phb4_root_port_realize(DeviceState *dev, Error **errp)
{
    PCIERootPortClass *rpc = PCIE_ROOT_PORT_GET_CLASS(dev);
    Error *local_err = NULL;

    rpc->parent_realize(dev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
}

static void pnv_phb4_root_port_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    PCIERootPortClass *rpc = PCIE_ROOT_PORT_CLASS(klass);

    dc->desc     = "IBM PHB4 PCIE Root Port";

    device_class_set_parent_realize(dc, pnv_phb4_root_port_realize,
                                    &rpc->parent_realize);

    k->vendor_id = PCI_VENDOR_ID_IBM;
    k->device_id = 0x04c1;
    k->revision  = 0;

    // XXX FIXME
    rpc->exp_offset = 0x48;
    rpc->aer_offset = 0x100;

    dc->reset = &pnv_phb4_root_port_reset;
}

static const TypeInfo pnv_phb4_root_port_info = {
    .name          = TYPE_PNV_PHB4_ROOT_PORT,
    .parent        = TYPE_PCIE_ROOT_PORT,
    .instance_size = sizeof(PnvPHB4RootPort),
    .class_init    = pnv_phb4_root_port_class_init,
};

static void pnv_phb4_register_types(void)
{
    type_register_static(&pnv_phb4_root_bus_info);
    type_register_static(&pnv_phb4_root_port_info);
    type_register_static(&pnv_phb4_type_info);
    type_register_static(&pnv_phb4_iommu_memory_region_info);
}

type_init(pnv_phb4_register_types)

void pnv_phb4_pic_print_info(PnvPHB4 *phb, Monitor *mon)
{
    uint32_t offset = phb->regs[PHB_INT_NOTIFY_INDEX >> 3];

    monitor_printf(mon, "PHB4[%x:%x] Source %08x .. %08x\n",
                   phb->chip_id, phb->phb_id,
                   offset, offset + phb->xsrc.nr_irqs - 1);
    xive_source_pic_print_info(&phb->xsrc, 0, mon);
}
