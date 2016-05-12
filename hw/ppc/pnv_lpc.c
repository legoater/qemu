
/*
 * QEMU PowerNV LPC bus definitions
 *
 * Copyright (c) 2010 David Gibson, IBM Corporation <dwg@au1.ibm.com>
 * Based on the s390 virtio bus code:
 * Copyright (c) 2009 Alexander Graf <agraf@suse.de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "monitor/monitor.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "sysemu/kvm.h"
#include "sysemu/device_tree.h"
#include "kvm_ppc.h"

#include "hw/isa/isa.h"
#include "hw/ppc/pnv_xscom.h"
#include "hw/ppc/pnv.h"

#include <libfdt.h>

enum {
    ECCB_CTL    = 0,
    ECCB_RESET  = 1,
    ECCB_STAT   = 2,
    ECCB_DATA   = 3,
};

#define LPCDBG(fmt...) do { } while(0)
//#define LPCDBG(fmt...) do { printf(fmt); } while(0)
#define OPBDBG(fmt...) do { } while(0)
//#define OPBDBG(fmt...) do { printf(fmt); } while(0)

typedef struct PnvLpcController {
    XScomDevice xd;
    uint64_t eccb_stat_reg;
    uint32_t eccb_data_reg;
    bool has_serirq;

    /* OPB bus */
    MemoryRegion opb_mr;
    AddressSpace opb_as;
    /* ISA IO and Memory space */
    MemoryRegion isa_io;
    MemoryRegion isa_mem;
    ISABus *isa_bus;
    /* Windows from OPB to ISA (aliases) */
    MemoryRegion opb_isa_io;
    MemoryRegion opb_isa_mem;
    MemoryRegion opb_isa_fw;
    /* Registers */
    MemoryRegion lpc_hc_regs;
    MemoryRegion opb_master_regs;

    /* OPB Master LS registers */
#define OPB_MASTER_LS_IRQ_STAT  0x50
#define   OPB_MASTER_IRQ_LPC            0x00000800
    uint32_t opb_irq_stat;
#define OPB_MASTER_LS_IRQ_MASK  0x54
    uint32_t opb_irq_mask;
#define OPB_MASTER_LS_IRQ_POL   0x58
    uint32_t opb_irq_pol;

    /* LPC HC registers */
#define LPC_HC_FW_SEG_IDSEL     0x24
    uint32_t lpc_hc_fw_seg_idsel;
#define LPC_HC_FW_RD_ACC_SIZE   0x28
#define   LPC_HC_FW_RD_1B               0x00000000
#define   LPC_HC_FW_RD_2B               0x01000000
#define   LPC_HC_FW_RD_4B               0x02000000
#define   LPC_HC_FW_RD_16B              0x04000000
#define   LPC_HC_FW_RD_128B             0x07000000
    uint32_t lpc_hc_fw_rd_acc_size;
#define LPC_HC_IRQSER_CTRL      0x30
#define   LPC_HC_IRQSER_EN              0x80000000
#define   LPC_HC_IRQSER_QMODE           0x40000000
#define   LPC_HC_IRQSER_START_MASK      0x03000000
#define   LPC_HC_IRQSER_START_4CLK      0x00000000
#define   LPC_HC_IRQSER_START_6CLK      0x01000000
#define   LPC_HC_IRQSER_START_8CLK      0x02000000
    uint32_t lpc_hc_irqser_ctrl;
#define LPC_HC_IRQMASK          0x34    /* same bit defs as LPC_HC_IRQSTAT */
    uint32_t lpc_hc_irqmask;
#define LPC_HC_IRQSTAT          0x38
#define   LPC_HC_IRQ_SERIRQ0            0x80000000 /* all bits down to ... */
#define   LPC_HC_IRQ_SERIRQ16           0x00008000 /* IRQ16=IOCHK#, IRQ2=SMI# */
#define   LPC_HC_IRQ_SERIRQ_ALL         0xffff8000
#define   LPC_HC_IRQ_LRESET             0x00000400
#define   LPC_HC_IRQ_SYNC_ABNORM_ERR    0x00000080
#define   LPC_HC_IRQ_SYNC_NORESP_ERR    0x00000040
#define   LPC_HC_IRQ_SYNC_NORM_ERR      0x00000020
#define   LPC_HC_IRQ_SYNC_TIMEOUT_ERR   0x00000010
#define   LPC_HC_IRQ_SYNC_TARG_TAR_ERR  0x00000008
#define   LPC_HC_IRQ_SYNC_BM_TAR_ERR    0x00000004
#define   LPC_HC_IRQ_SYNC_BM0_REQ       0x00000002
#define   LPC_HC_IRQ_SYNC_BM1_REQ       0x00000001
    uint32_t lpc_hc_irqstat;
#define LPC_HC_ERROR_ADDRESS    0x40
    uint32_t lpc_hc_error_addr;

} PnvLpcController;

#define ISA_IO_SIZE             0x00010000
#define ISA_MEM_SIZE            0x10000000
#define LPC_IO_OPB_ADDR         0xd0010000
#define LPC_IO_OPB_SIZE         0x00010000
#define LPC_MEM_OPB_ADDR        0xe0010000
#define LPC_MEM_OPB_SIZE        0x10000000
#define LPC_FW_OPB_ADDR         0xf0000000
#define LPC_FW_OPB_SIZE         0x10000000

#define LPC_OPB_REGS_OPB_ADDR   0xc0010000
#define LPC_OPB_REGS_OPB_SIZE   0x00002000
#define LPC_HC_REGS_OPB_ADDR    0xc0012000
#define LPC_HC_REGS_OPB_SIZE    0x00001000

#define TYPE_PNV_LPC_CONTROLLER "pnv-lpc"
#define PNV_LPC_CONTROLLER(obj) \
     OBJECT_CHECK(PnvLpcController, (obj), TYPE_PNV_LPC_CONTROLLER)

#define _FDT(exp) \
    do { \
        int ret = (exp);                                           \
        if (ret < 0) {                                             \
            fprintf(stderr, "qemu: error creating device tree: %s: %s\n", \
                    #exp, fdt_strerror(ret));                      \
            exit(1);                                               \
        }                                                          \
    } while (0)

static int pnv_lpc_devnode(XScomDevice *dev, void *fdt)
{
    _FDT((fdt_property_cell(fdt, "#address-cells", 2)));
    _FDT((fdt_property_cell(fdt, "#size-cells", 1)));
    _FDT((fdt_property(fdt, "primary", NULL, 0)));
    return 0;
}

static bool opb_read(PnvLpcController *lpc, uint32_t addr, uint8_t *data, int sz)
{
    bool success;

    /* XXX Handle access size limits and FW read caching here */
    success = !address_space_rw(&lpc->opb_as, addr, MEMTXATTRS_UNSPECIFIED,
                                data, sz, false);

    LPCDBG("OPB read @0x%08x, sz=%d data=%02x %02x %02x %02x ok=%d\n",
           addr, sz, data[0], data[1], data[2], data[3], success);

    return success;
}

static bool opb_write(PnvLpcController *lpc, uint32_t addr, uint8_t *data, int sz)
{
    bool success;

    /* XXX Handle access size limits here */
    success = !address_space_rw(&lpc->opb_as, addr, MEMTXATTRS_UNSPECIFIED,
                                data, sz, true);

    LPCDBG("OPB write @0x%08x, sz=%d data=%02x %02x %02x %02x ok=%d\n",
           addr, sz, data[0], data[1], data[2], data[3], success);

    return success;
}

#define ECCB_CTL_READ           (1ull << (63-15))
#define ECCB_CTL_SZ_LSH         (63-7)
#define ECCB_CTL_SZ_MASK        (0xfull << ECCB_CTL_SZ_LSH)
#define ECCB_CTL_ADDR_MASK      0xffffffffu;

#define ECCB_STAT_OP_DONE       (1ull << (63-52))
#define ECCB_STAT_OP_ERR        (1ull << (63-52))
#define ECCB_STAT_RD_DATA_LSH   (63-37)
#define ECCB_STAT_RD_DATA_MASK  (0xffffffff << ECCB_STAT_RD_DATA_LSH)

static void pnv_lpc_do_eccb(PnvLpcController *lpc, uint64_t cmd)
{
    /* XXX Check for magic bits at the top, addr size etc... */
    unsigned int sz = (cmd & ECCB_CTL_SZ_MASK) >> ECCB_CTL_SZ_LSH;
    uint32_t opb_addr = cmd & ECCB_CTL_ADDR_MASK;
    uint8_t data[4];
    bool success;

    LPCDBG("ECCB cmd: %016llx data: %08x\n",
           (unsigned long long)cmd, lpc->eccb_data_reg);

    if (cmd & ECCB_CTL_READ) {
        success = opb_read(lpc, opb_addr, data, sz);
        if (success) {
            lpc->eccb_stat_reg = ECCB_STAT_OP_DONE |
                    (((uint64_t)data[0]) << 24 |
                     ((uint64_t)data[1]) << 16 |
                     ((uint64_t)data[2]) <<  8 |
                     ((uint64_t)data[3])) << ECCB_STAT_RD_DATA_LSH;
        } else {
            lpc->eccb_stat_reg = ECCB_STAT_OP_DONE |
                    (0xffffffffull << ECCB_STAT_RD_DATA_LSH);
        }
    } else {
        data[0] = lpc->eccb_data_reg >> 24;
        data[1] = lpc->eccb_data_reg >> 16;
        data[2] = lpc->eccb_data_reg >>  8;
        data[3] = lpc->eccb_data_reg;

        LPCDBG("OPB write @0x%08x, sz=%d data=%02x %02x %02x %02x\n",
               opb_addr, sz, data[0], data[1], data[2], data[3]);

        success = opb_write(lpc, opb_addr, data, sz);
        lpc->eccb_stat_reg = ECCB_STAT_OP_DONE;
    }
    /* XXX Which error bit (if any) to signal OPB error ? */
}

static bool pnv_lpc_xscom_read(XScomDevice *dev, uint32_t range,
                               uint32_t offset, uint64_t *out_val)
{
    PnvLpcController *lpc = PNV_LPC_CONTROLLER(dev);

    switch(offset & 3) {
    case ECCB_CTL:
    case ECCB_RESET:
        *out_val = 0;
        break;
    case ECCB_STAT:
        *out_val = lpc->eccb_stat_reg;
        lpc->eccb_stat_reg = 0;
        break;
    case ECCB_DATA:
        *out_val = ((uint64_t)lpc->eccb_data_reg) << 32;
        break;
    }
    return true;
}

static bool pnv_lpc_xscom_write(XScomDevice *dev, uint32_t range,
                                uint32_t offset, uint64_t val)
{
    PnvLpcController *lpc = PNV_LPC_CONTROLLER(dev);

    switch(offset & 3) {
    case ECCB_CTL:
        pnv_lpc_do_eccb(lpc, val);
        break;
    case ECCB_RESET:
        /*  XXXX  */
        break;
    case ECCB_STAT:
        break;
    case ECCB_DATA:
        lpc->eccb_data_reg = val >> 32;
        break;
    }
    return true;
}

static void pnv_lpc_isa_irq_handler(void *opaque, int n, int level)
{
     /* XXX TODO */
}

static uint64_t lpc_hc_read(void *opaque, hwaddr addr, unsigned size)
{
    PnvLpcController *lpc = opaque;

    if (size != 4) {
        fprintf(stderr, "lpc_hc_read: Invalid size %d\n", size);
        return 0xfffffffffffffffful;
    }

    OPBDBG("LPC HC read @0x%08x\n", (unsigned int)addr);

    switch(addr) {
    case LPC_HC_FW_SEG_IDSEL:
        return lpc->lpc_hc_fw_seg_idsel;
    case LPC_HC_FW_RD_ACC_SIZE:
        return lpc->lpc_hc_fw_rd_acc_size;
    case LPC_HC_IRQSER_CTRL:
        return lpc->lpc_hc_irqser_ctrl;
    case LPC_HC_IRQMASK:
        return lpc->lpc_hc_irqmask;
    case LPC_HC_IRQSTAT:
        return lpc->lpc_hc_irqstat;
    case LPC_HC_ERROR_ADDRESS:
        return lpc->lpc_hc_error_addr;
    default:
        OPBDBG("LPC HC Unimplemented register !\n");
        return 0xfffffffffffffffful;
    }
}

static void lpc_hc_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    PnvLpcController *lpc = opaque;

    if (size != 4) {
        fprintf(stderr, "lpc_hc_write: Invalid size %d\n", size);
        return;
    }

    OPBDBG("LPC HC write @0x%08x\n", (unsigned int)addr);

    /* XXX Filter out reserved bits */

    switch(addr) {
    case LPC_HC_FW_SEG_IDSEL:
        /* XXX Actually figure out how that works as this impact
         * memory regions/aliases\
         */
        lpc->lpc_hc_fw_seg_idsel = val;
    case LPC_HC_FW_RD_ACC_SIZE:
        lpc->lpc_hc_fw_rd_acc_size = val;
    case LPC_HC_IRQSER_CTRL:
        lpc->lpc_hc_irqser_ctrl = val;
    case LPC_HC_IRQMASK:
        lpc->lpc_hc_irqmask = val;
    case LPC_HC_IRQSTAT:
        lpc->lpc_hc_irqstat &= ~val;
    case LPC_HC_ERROR_ADDRESS:
        break;
    default:
        OPBDBG("LPC HC Unimplemented register !\n");
    }
}

static const MemoryRegionOps lpc_hc_ops = {
    .read = lpc_hc_read,
    .write = lpc_hc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t opb_master_read(void *opaque, hwaddr addr, unsigned size)
{
    PnvLpcController *lpc = opaque;

    if (size != 4) {
        fprintf(stderr, "opb_master_read: Invalid size %d\n", size);
        return 0xfffffffffffffffful;
    }

    OPBDBG("OPB MASTER read @0x%08x\n", (unsigned int)addr);

    switch(addr) {
    case OPB_MASTER_LS_IRQ_STAT:
        return lpc->opb_irq_stat;
    case OPB_MASTER_LS_IRQ_MASK:
        return lpc->opb_irq_mask;
    case OPB_MASTER_LS_IRQ_POL:
        return lpc->opb_irq_pol;
    default:
        OPBDBG("OPB MASTER Unimplemented register !\n");
        return 0xfffffffffffffffful;
    }
}

static void opb_master_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    PnvLpcController *lpc = opaque;

    if (size != 4) {
        fprintf(stderr, "opb_master_write: Invalid size %d\n", size);
        return;
    }

    OPBDBG("OPB MASTER write @0x%08x\n", (unsigned int)addr);

    switch(addr) {
    case OPB_MASTER_LS_IRQ_STAT:
        lpc->opb_irq_stat &= ~val;
        break;
    case OPB_MASTER_LS_IRQ_MASK:
        /* XXX Filter out reserved bits */
        lpc->opb_irq_mask = val;
        break;
    case OPB_MASTER_LS_IRQ_POL:
        /* XXX Filter out reserved bits */
        lpc->opb_irq_pol = val;
        break;
    default:
        OPBDBG("OPB MASTER Unimplemented register !\n");
    }
}

static const MemoryRegionOps opb_master_ops = {
    .read = opb_master_read,
    .write = opb_master_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void pnv_lpc_realize(DeviceState *dev, Error **errp)
{
    PnvLpcController *lpc = PNV_LPC_CONTROLLER(dev);

    /* LPC XSCOM address is fixed */
    lpc->xd.ranges[0].addr = 0xb0020;
    lpc->xd.ranges[0].size = 4;

    /* Reg inits */
    lpc->lpc_hc_fw_rd_acc_size = LPC_HC_FW_RD_4B;

    /* Create address space and backing MR for the OPB bus */
    memory_region_init(&lpc->opb_mr, OBJECT(dev), "lpc-opb", 0x100000000ull);
    address_space_init(&lpc->opb_as, &lpc->opb_mr, "lpc-opb");

    /* Create ISA IO and Mem space regions which are the root of
     * the ISA bus (ie, ISA address spaces). We don't create a
     * separate one for FW which we alias to memory.
     */
    memory_region_init(&lpc->isa_io, OBJECT(dev), "isa-io", ISA_IO_SIZE);
    memory_region_init(&lpc->isa_mem, OBJECT(dev), "isa-mem", ISA_MEM_SIZE);

    /* Create windows from the OPB space to the ISA space */
    memory_region_init_alias(&lpc->opb_isa_io, OBJECT(dev), "lpc-isa-io",
                             &lpc->isa_io, 0, LPC_IO_OPB_SIZE);
    memory_region_add_subregion(&lpc->opb_mr, LPC_IO_OPB_ADDR,
                                &lpc->opb_isa_io);
    memory_region_init_alias(&lpc->opb_isa_mem, OBJECT(dev), "lpc-isa-mem",
                             &lpc->isa_mem, 0, LPC_MEM_OPB_SIZE);
    memory_region_add_subregion(&lpc->opb_mr, LPC_MEM_OPB_ADDR,
                                &lpc->opb_isa_mem);
    memory_region_init_alias(&lpc->opb_isa_fw, OBJECT(dev), "lpc-isa-fw",
                             &lpc->isa_mem, 0, LPC_FW_OPB_SIZE);
    memory_region_add_subregion(&lpc->opb_mr, LPC_FW_OPB_ADDR,
                                &lpc->opb_isa_fw);


    /* Create MMIO regions for LPC HC and OPB registers */
    memory_region_init_io(&lpc->opb_master_regs, OBJECT(dev), &opb_master_ops,
                          lpc, "lpc-opb-master", LPC_OPB_REGS_OPB_SIZE);
    memory_region_add_subregion(&lpc->opb_mr, LPC_OPB_REGS_OPB_ADDR,
                                &lpc->opb_master_regs);
    memory_region_init_io(&lpc->lpc_hc_regs, OBJECT(dev), &lpc_hc_ops, lpc,
                          "lpc-hc", LPC_HC_REGS_OPB_SIZE);
    memory_region_add_subregion(&lpc->opb_mr, LPC_HC_REGS_OPB_ADDR,
                                &lpc->lpc_hc_regs);

    /* Instanciate ISA bus */
    lpc->isa_bus = isa_bus_new(dev, &lpc->isa_mem, &lpc->isa_io, errp);

    /* Not all variants have a working serial irq decoder. If not,
     * handling of LPC interrupts becomes a platform issue (some
     * platforms have a CPLD to do it).
     */
    if (lpc->has_serirq) {
        isa_bus_irqs(lpc->isa_bus,
                     qemu_allocate_irqs(pnv_lpc_isa_irq_handler, lpc, 16));
    }
}

void pnv_lpc_create(PnvChip *chip, bool has_serirq)
{
    struct DeviceState *dev;
    PnvLpcController *lpc;

    dev = qdev_create(&chip->xscom->bus, TYPE_PNV_LPC_CONTROLLER);
    lpc = PNV_LPC_CONTROLLER(dev);
    lpc->has_serirq = has_serirq;
    qdev_init_nofail(dev);
    chip->lpc = lpc;
    chip->lpc_bus = lpc->isa_bus;
}

static void pnv_lpc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XScomDeviceClass *k = XSCOM_DEVICE_CLASS(klass);
    static const char *compat[] = { "ibm,power8-lpc", NULL };

    k->devnode = pnv_lpc_devnode;
    k->read = pnv_lpc_xscom_read;
    k->write = pnv_lpc_xscom_write;
    k->dt_name = "isa";
    k->dt_compatible = compat;

    dc->realize = pnv_lpc_realize;
}

static const TypeInfo pnv_lpc_info = {
    .name          = TYPE_PNV_LPC_CONTROLLER,
    .parent        = TYPE_XSCOM_DEVICE,
    .instance_size = sizeof(PnvLpcController),
    .class_init    = pnv_lpc_class_init,
};

static void pnv_lpc_register_types(void)
{
    type_register_static(&pnv_lpc_info);
}

type_init(pnv_lpc_register_types)
