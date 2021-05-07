/*
 * QEMU PowerPC PowerNV Processor Nvlink Processing Unit (NPU) model
 *
 * Copyright (c) 2019-2020, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"

#include "hw/ppc/pnv.h"
#include "hw/ppc/pnv_npu.h"
#include "hw/ppc/pnv_xscom.h"
#include "hw/ppc/fdt.h"

#include <libfdt.h>

/* SCOM registers */
#define NPU2_STACK1_CS_SM0_GENID_BAR            0x207
#define NPU2_STACK1_CS_SM1_GENID_BAR            0x237
#define NPU2_STACK1_CS_SM2_GENID_BAR            0x267
#define NPU2_STACK1_CS_SM3_GENID_BAR            0x297
#define NPU2_STACK2_CS_SM0_GENID_BAR            0x407
#define NPU2_STACK2_CS_SM1_GENID_BAR            0x437
#define NPU2_STACK2_CS_SM2_GENID_BAR            0x467
#define NPU2_STACK2_CS_SM3_GENID_BAR            0x497
#define  NPU2_GENID_BAR_ENABLE                  PPC_BIT(0)
#define  NPU2_GENID_BAR_ADDR                    PPC_BITMASK(3, 35)
#define NPU2_MISC_SCOM_IND_SCOM_ADDR            0x68e
#define  NPU2_MISC_DA_ADDR                      PPC_BITMASK(0, 23)
#define  NPU2_MISC_DA_LEN                       PPC_BITMASK(24, 25)
#define NPU2_MISC_SCOM_IND_SCOM_DATA            0x68f
#define NPU2_STACK1_CQ_CTL_FENCE_CONTROL0       0x2e8
#define NPU2_STACK1_CQ_CTL_FENCE_CONTROL1       0x2e9
#define NPU2_STACK2_CQ_CTL_FENCE_CONTROL0       0x4e8
#define NPU2_STACK2_CQ_CTL_FENCE_CONTROL1       0x4e9
#define  NPU2_CQ_CTL_FENCE_CONTROL_REQUEST_FENCE PPC_BITMASK(0, 1)
#define NPU2_STACK1_CQ_CTL_STATUS               0x2d2
#define NPU2_STACK2_CQ_CTL_STATUS               0x4d2
#define  NPU2_CQ_CTL_STATUS_BRK0_AM_FENCED      PPC_BITMASK(48, 49)
#define  NPU2_CQ_CTL_STATUS_BRK1_AM_FENCED      PPC_BITMASK(50, 51)

#define OBUS_ODL0_STATUS 0x2c
#define OBUS_ODL1_STATUS 0x2d

/* genid mmio addresses */
#define NPU2_MMIO_GENID_OTL0_ADDR     0
#define  NPU2_MMIO_GENID_ADDR_EN      PPC_BIT(0)
#define  NPU2_MMIO_GENID_ADDR_BUS     PPC_BITMASK(4, 11)
#define  NPU2_MMIO_GENID_ADDR_DEV     PPC_BITMASK(12, 16)
#define  NPU2_MMIO_GENID_ADDR_FUNC    PPC_BITMASK(17, 19)
#define  NPU2_MMIO_GENID_ADDR_REG     PPC_BITMASK(20, 31)
#define NPU2_MMIO_GENID_OTL0_DATA     0x80
#define NPU2_MMIO_GENID_OTL1_ADDR     0x100
#define NPU2_MMIO_GENID_OTL1_DATA     0x180

#define INVALID_SCOM_OFFSET (~0ULL)
#define INVALID_SCOM_DATA (~0ULL)
#define INVALID_MMIO_DATA (~0ULL)

#undef PNV_NPU2_DEBUG
#ifdef PNV_NPU2_DEBUG
#define DEBUG(fmt, ...) do {         \
        fprintf(stderr,  "%s: " fmt, __func__, ## __VA_ARGS__);  \
    } while (0)
#else
    #define DEBUG(...) do {} while (0)
#endif

/*
 * QEMU version of the GETFIELD/SETFIELD macros
 *
 * These are common with the PnvXive model.
 */
static inline uint64_t GETFIELD(uint64_t mask, uint64_t word)
{
    return (word & mask) >> ctz64(mask);
}

static inline uint64_t SETFIELD(uint64_t mask, uint64_t word,
                                uint64_t value)
{
    return (word & ~mask) | ((value << ctz64(mask)) & mask);
}

static void dt_add_npu_link(void *fdt, int offset, int group, int link_index)
{
    char *name;
    int link_offset;
    uint32_t lane_mask;
    const char compat[] = "ibm,npu-link";

    switch (link_index) {
    case 2:
        lane_mask = 0xf1e000; /* 0-3, 7-10 */
        break;
    case 3:
        lane_mask = 0x00078f; /* 13-16, 20-23 */
        break;
    default:
        assert(0);
    }

    name = g_strdup_printf("link@%x", link_index);
    link_offset = fdt_add_subnode(fdt, offset, name);
    _FDT(link_offset);
    g_free(name);

    _FDT(fdt_setprop(fdt, link_offset, "compatible", compat, sizeof(compat)));
    _FDT(fdt_setprop_cell(fdt, link_offset, "ibm,npu-link-index", link_index));
    _FDT(fdt_setprop_u64(fdt, link_offset, "ibm,npu-phy",
                         PNV9_XSCOM_NPU_INDIRECT0));
    _FDT(fdt_setprop_cell(fdt, link_offset, "ibm,npu-lane-mask", lane_mask));
    _FDT(fdt_setprop_cell(fdt, link_offset, "ibm,npu-group-id", group));
    _FDT(fdt_setprop_u64(fdt, link_offset, "ibm,link-speed", 25000000000ul));
}

static int pnv_npu2_dt_xscom(PnvXScomInterface *dev, void *fdt,
                             int offset)
{
    char *name;
    int npu_offset;
    static int npu_index;
    static int phb_index = 7; /* to match what skiboot does from HDAT */
    const char compat[] = "ibm,power9-npu";
    uint32_t reg[2] = {
        cpu_to_be32(PNV9_XSCOM_NPU_BASE1),
        cpu_to_be32(PNV9_XSCOM_NPU_SIZE1)
    };

    name = g_strdup_printf("npu@%x", PNV9_XSCOM_NPU_BASE1);
    npu_offset = fdt_add_subnode(fdt, offset, name);
    _FDT(npu_offset);
    g_free(name);

    _FDT(fdt_setprop(fdt, npu_offset, "reg", reg, sizeof(reg)));
    _FDT(fdt_setprop(fdt, npu_offset, "compatible", compat, sizeof(compat)));
    _FDT(fdt_setprop_cell(fdt, npu_offset, "ibm,npu-index", npu_index++));
    _FDT(fdt_setprop_cell(fdt, npu_offset, "ibm,phb-index", phb_index++));
    _FDT(fdt_setprop_cell(fdt, npu_offset, "ibm,npu-links", 2));
    dt_add_npu_link(fdt, npu_offset, 1, 2);
    dt_add_npu_link(fdt, npu_offset, 2, 3);
    return 0;
}

static hwaddr ring_to_scom(hwaddr ring_addr)
{
    int stack, block;
    hwaddr scom_offset;

    stack = GETFIELD(PPC_BITMASK(40, 43), ring_addr);
    block = GETFIELD(PPC_BITMASK(44, 47), ring_addr);

    switch (stack) {
    case 1 ... 2:
        scom_offset = 0x200 * stack;
        if (block == 0xC) /* User OTL0 */
            scom_offset += 0x178;
        else if (block == 0xD) /* User OTL1 */
            scom_offset += 0x1A8;
        else
            goto unimplemented;
        break;
    case 3:
        if (block == 0x3) /* MISC */
            scom_offset = 0x680;
        else
            goto unimplemented;
        break;
    case 4 ... 6:
        scom_offset = 0x200 * (stack - 4);
        switch (block) {
        case 0 ... 3: /* CQ SM */
            scom_offset += 0x30 * block;
            break;
        case 4: /* CQ CTL */
            scom_offset += 0xC0;
            break;
        case 5: /* CQ DAT */
            scom_offset += 0xF0;
            break;
        case 0xC: /* OTL0 */
            if (stack == 4) {
                goto unimplemented;
            }
            scom_offset += 0x150;
            break;
        case 0xD: /* OTL1 */
            if (stack == 4) {
                goto unimplemented;
            }
            scom_offset += 0x180;
            break;
        case 0xE: /* XSL */
            if (stack == 4) {
                goto unimplemented;
            }
            scom_offset += 0x1B0;
            break;
        default:
            goto unimplemented;
        }
        break;
    case 7:
        switch (block) {
        case 0: /* ATS */
            scom_offset = 0x600;
            break;
        case 1: /* XTS */
            scom_offset = 0x640;
            break;
        case 2 ... 3: /* MISC */
            scom_offset = 0x680;
            break;
        default:
            goto unimplemented;
        }
        break;
    default:
        goto unimplemented;
    }

    scom_offset += GETFIELD(PPC_BITMASK(48, 63), ring_addr) >> 3;
    DEBUG("ring addr=%lx => scom offset=%lx\n", ring_addr, scom_offset);
    assert(scom_offset <= PNV9_XSCOM_NPU_SIZE1);
    return scom_offset;

unimplemented:
    qemu_log_mask(LOG_UNIMP, "NPU: untranslated ring addr %lx\n", ring_addr);
    return INVALID_SCOM_OFFSET;
}

static hwaddr indirect_to_scom(hwaddr indirect_addr)
{
    int indirect_len;
    hwaddr ring_addr;

    indirect_len = GETFIELD(NPU2_MISC_DA_LEN, indirect_addr);
    if (indirect_len != 0b11) {
        qemu_log_mask(LOG_UNIMP, "NPU: only 8 byte mmio access supported\n");
        return INVALID_SCOM_OFFSET;
    }
    ring_addr = GETFIELD(NPU2_MISC_DA_ADDR, indirect_addr);
    return ring_to_scom(ring_addr);
}

static void update_fence_state(PnvNpu2 *npu, int brick, int val)
{
    hwaddr offset;
    uint64_t fence_mask;

    DEBUG("Setting fence state of brick %d to %d\n", brick, val);
    switch (brick) {
    case 2:
        offset = NPU2_STACK1_CQ_CTL_STATUS;
        fence_mask = NPU2_CQ_CTL_STATUS_BRK0_AM_FENCED;
        break;
    case 3:
        offset = NPU2_STACK1_CQ_CTL_STATUS;
        fence_mask = NPU2_CQ_CTL_STATUS_BRK1_AM_FENCED;
        break;
    case 4:
        offset = NPU2_STACK2_CQ_CTL_STATUS;
        fence_mask = NPU2_CQ_CTL_STATUS_BRK0_AM_FENCED;
        break;
    case 5:
        offset = NPU2_STACK2_CQ_CTL_STATUS;
        fence_mask = NPU2_CQ_CTL_STATUS_BRK1_AM_FENCED;
        break;
    default:
        assert(false);
    }
    npu->scom[offset] = SETFIELD(fence_mask, npu->scom[offset], val);
    npu->fence_state[brick] = !!val;
}

static void set_genid_bar(PnvNpu2 *npu, int stack, uint64_t val)
{
    MemoryRegion *sysmem = get_system_memory();
    uint64_t addr;

    assert(stack == 1 || stack == 2);

    /*
     * The genid bar is replicated in each SMx block, so we'll get 4
     * identical operations each time the bar is set. So don't do
     * anything is we're writing the exact same value.
     */
    if (val == npu->genid_bar[stack]) {
        return;
    }

    if (npu->genid_bar[stack]) {
        memory_region_del_subregion(sysmem, &npu->genid_mr[stack]);
    }

    npu->genid_bar[stack] = val;
    addr = (GETFIELD(NPU2_GENID_BAR_ADDR, val) << 16) | 0x6000000000000;
    if (val & NPU2_GENID_BAR_ENABLE) {
        DEBUG("mapping genid bar at %lx\n", addr);
        memory_region_add_subregion(sysmem, addr, &npu->genid_mr[stack]);
    }
}

static uint64_t pnv_npu2_xscom1_read(void *opaque, hwaddr addr, unsigned size)
{
    PnvNpu2 *npu = PNV_NPU2(opaque);
    hwaddr scom_offset;
    int reg = addr >> 3;
    uint64_t val;

    if (reg >= PNV9_XSCOM_NPU_SIZE1) {
        return INVALID_SCOM_DATA;
    }

    val = npu->scom[reg];
    switch (reg) {
    case NPU2_MISC_SCOM_IND_SCOM_DATA:
            scom_offset =
                indirect_to_scom(npu->scom[NPU2_MISC_SCOM_IND_SCOM_ADDR]);
            return pnv_npu2_xscom1_read(opaque, scom_offset << 3, size);
    }

    return val;
}

static void pnv_npu2_xscom1_write(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    PnvNpu2 *npu = PNV_NPU2(opaque);
    hwaddr scom_offset;
    int brick, reg = addr >> 3;
    uint64_t subval;

    if (reg >= PNV9_XSCOM_NPU_SIZE1) {
        return;
    }

    npu->scom[reg] = val;
    switch (reg) {
    case NPU2_MISC_SCOM_IND_SCOM_DATA:
        scom_offset = indirect_to_scom(npu->scom[NPU2_MISC_SCOM_IND_SCOM_ADDR]);
        pnv_npu2_xscom1_write(opaque, scom_offset << 3, val, size);
        return;
    case NPU2_STACK1_CQ_CTL_FENCE_CONTROL0:
    case NPU2_STACK1_CQ_CTL_FENCE_CONTROL1:
    case NPU2_STACK2_CQ_CTL_FENCE_CONTROL0:
    case NPU2_STACK2_CQ_CTL_FENCE_CONTROL1:
        brick = (reg / 0x200) * 2 + reg % 2;
        subval = GETFIELD(NPU2_CQ_CTL_FENCE_CONTROL_REQUEST_FENCE, val);
        update_fence_state(npu, brick, subval);
        break;
    case NPU2_STACK1_CS_SM0_GENID_BAR:
    case NPU2_STACK1_CS_SM1_GENID_BAR:
    case NPU2_STACK1_CS_SM2_GENID_BAR:
    case NPU2_STACK1_CS_SM3_GENID_BAR:
        set_genid_bar(npu, 1, val);
        break;
    case NPU2_STACK2_CS_SM0_GENID_BAR:
    case NPU2_STACK2_CS_SM1_GENID_BAR:
    case NPU2_STACK2_CS_SM2_GENID_BAR:
    case NPU2_STACK2_CS_SM3_GENID_BAR:
        set_genid_bar(npu, 2, val);
        break;
    }
}

static const MemoryRegionOps pnv_npu2_xscom1_ops = {
    .read = pnv_npu2_xscom1_read,
    .write = pnv_npu2_xscom1_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t pnv_npu2_xscom2_read(void *opaque, hwaddr addr, unsigned size)
{
    int reg = addr >> 3;

    if (reg >= PNV9_XSCOM_NPU_SIZE2) {
        return INVALID_SCOM_DATA;
    }

    return 0;
}

static void pnv_npu2_xscom2_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
}

static const MemoryRegionOps pnv_npu2_xscom2_ops = {
    .read = pnv_npu2_xscom2_read,
    .write = pnv_npu2_xscom2_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t pnv_obus0_xscom_read(void *opaque, hwaddr addr, unsigned size)
{
    int reg = addr >> 3;

    DEBUG("reading obus0 offset %x\n", reg);
    switch (reg) {
    case OBUS_ODL0_STATUS:
    case OBUS_ODL1_STATUS:
        return 0x2900ffff00007100; /* link is trained */
    default:
        return INVALID_SCOM_DATA;
    }
}

static void pnv_obus0_xscom_write(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    DEBUG("writing obus0 offset %x\n", addr >> 3);
}

static const MemoryRegionOps pnv_obus0_xscom_ops = {
    .read = pnv_obus0_xscom_read,
    .write = pnv_obus0_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

/*
 * fxb high-level logic should be moved to pnv_xscom, since it's
 * independent from chiplet
 */
static uint64_t pnv_obus0_xscom_indirect_read(void *opaque, hwaddr addr,
                                              unsigned size)
{
    PnvNpu2 *npu = PNV_NPU2(opaque);
    bool read;
    uint64_t res, reg, data;

    if (!npu->xscom_obus0_indirect_addr) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Indirect xscom read with no operation in progress\n");
        return INVALID_SCOM_DATA;
    }
    addr = GETFIELD(PPC_BITMASK(12, 31), npu->xscom_obus0_indirect_addr);
    reg = GETFIELD(PPC_BITMASK(10, 21), npu->xscom_obus0_indirect_addr);
    read = npu->xscom_obus0_indirect_addr & PPC_BIT(0);

    switch (reg) {
    case 0x3c1: /* ZCAL DONE, ZCAL ERROR */
        data = PPC_BIT(50);
        break;
    case 0x0ca: /* INIT DONE, DCCAL DONE, LANE BUSY */
        data = PPC_BIT(48) | PPC_BIT(49);
        break;
    default:
        data = 0;
    }
    DEBUG("fxb obus indirect op reg=%lx data=%lx\n", reg, data);
    res = XSCOM_DATA_IND_COMPLETE; /* data ready */
    res = SETFIELD(XSCOM_ADDR_IND_ADDR, res, addr);
    res = SETFIELD(XSCOM_ADDR_IND_DATA, res, data);
    if (read) {
        res |= XSCOM_DATA_IND_READ;
    }
    npu->xscom_obus0_indirect_addr = 0;
    DEBUG("fxb obus indirect op at %lx, returning %lx\n", addr, res);
    return res;
}

static void pnv_obus0_xscom_indirect_write(void *opaque, hwaddr addr,
                                           uint64_t val, unsigned size)
{
    PnvNpu2 *npu = PNV_NPU2(opaque);

    if (npu->xscom_obus0_indirect_addr) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Indirect xscom write while previous operation still in progress\n");
        /* keep going */
    }
    npu->xscom_obus0_indirect_addr = val;
}

static const MemoryRegionOps pnv_obus0_xscom_indirect_ops = {
    .read = pnv_obus0_xscom_indirect_read,
    .write = pnv_obus0_xscom_indirect_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t read_cfg(PnvNpu2 *npu, int brick, uint64_t addr, int offset,
                         unsigned size)
{
    int bus, dev, fn, reg;

    if (!npu->afu[brick]) {
        return INVALID_MMIO_DATA;
    }
    if (!(addr & NPU2_MMIO_GENID_ADDR_EN)) {
        return INVALID_MMIO_DATA;
    }

    bus = GETFIELD(NPU2_MMIO_GENID_ADDR_BUS, addr);
    dev = GETFIELD(NPU2_MMIO_GENID_ADDR_DEV, addr);
    fn = GETFIELD(NPU2_MMIO_GENID_ADDR_FUNC, addr);
    reg = GETFIELD(NPU2_MMIO_GENID_ADDR_REG, addr);

    if (bus || dev) {
        return INVALID_MMIO_DATA;
    }

    return npu->afu[brick]->read_cfg(fn, reg + offset, size);
}

static uint64_t pnv_npu2_genid_mmio_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    PnvNpu2 *npu = PNV_NPU2(opaque);
    int brick, offset;

    DEBUG("reading genid mmio addr=%lx, size %d\n", addr, size);
    switch (addr) {
    case NPU2_MMIO_GENID_OTL0_DATA:
    case NPU2_MMIO_GENID_OTL0_DATA + 1:
    case NPU2_MMIO_GENID_OTL0_DATA + 2:
    case NPU2_MMIO_GENID_OTL0_DATA + 3:
        brick = 2;
        offset = addr - NPU2_MMIO_GENID_OTL0_DATA;
        return read_cfg(npu, brick, npu->config_space_addr[brick],
                        offset, size);
    case NPU2_MMIO_GENID_OTL1_DATA:
    case NPU2_MMIO_GENID_OTL1_DATA + 1:
    case NPU2_MMIO_GENID_OTL1_DATA + 2:
    case NPU2_MMIO_GENID_OTL1_DATA + 3:
        brick = 3;
        offset = addr - NPU2_MMIO_GENID_OTL1_DATA;
        return read_cfg(npu, brick, npu->config_space_addr[brick],
                        offset, size);
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "genid mmio read at unexpected offset 0x%lx\n", addr);
    }
    return INVALID_MMIO_DATA;
}

static void write_cfg(PnvNpu2 *npu, int brick, uint64_t addr, uint64_t val,
                      unsigned size)
{
    int bus, dev, fn, reg;

    if (!npu->afu[brick]) {
        return;
    }
    if (!(addr & NPU2_MMIO_GENID_ADDR_EN)) {
        return;
    }

    bus = GETFIELD(NPU2_MMIO_GENID_ADDR_BUS, addr);
    dev = GETFIELD(NPU2_MMIO_GENID_ADDR_DEV, addr);
    fn = GETFIELD(NPU2_MMIO_GENID_ADDR_FUNC, addr);
    reg = GETFIELD(NPU2_MMIO_GENID_ADDR_REG, addr);

    if (bus || dev) {
        return;
    }

    npu->afu[brick]->write_cfg(fn, reg, val, size);
}

static void pnv_npu2_genid_mmio_write(void *opaque, hwaddr addr,
                                      uint64_t val, unsigned size)
{
    PnvNpu2 *npu = PNV_NPU2(opaque);
    int brick;

    DEBUG("writing genid mmio addr=%lx, size %d\n", addr, size);
    switch (addr) {
    case NPU2_MMIO_GENID_OTL0_ADDR:
        npu->config_space_addr[2] = val;
        break;
    case NPU2_MMIO_GENID_OTL1_ADDR:
        npu->config_space_addr[3] = val;
        break;
    case NPU2_MMIO_GENID_OTL0_DATA:
        brick = 2;
        write_cfg(npu, brick, npu->config_space_addr[brick], val, size);
        break;
    case NPU2_MMIO_GENID_OTL1_DATA:
        brick = 3;
        write_cfg(npu, brick, npu->config_space_addr[brick], val, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "genid mmio read at unexpected offset 0x%lx\n", addr);
    }
}

static const MemoryRegionOps pnv_npu2_genid_mmio_ops = {
    .read = pnv_npu2_genid_mmio_read,
    .write = pnv_npu2_genid_mmio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void pnv_npu2_realize(DeviceState *dev, Error **errp)
{
    PnvNpu2 *npu = PNV_NPU2(dev);

    assert(npu->chip);

    /* NPU xscom region */
    pnv_xscom_region_init(&npu->xscom_regs1, OBJECT(npu),
                          &pnv_npu2_xscom1_ops, npu,
                          "xscom1-npu2",
                          PNV9_XSCOM_NPU_SIZE1);

    pnv_xscom_region_init(&npu->xscom_regs2, OBJECT(npu),
                          &pnv_npu2_xscom2_ops, npu,
                          "xscom2-npu2",
                          PNV9_XSCOM_NPU_SIZE2);

    /* PHY xscom region */
    pnv_xscom_region_init(&npu->xscom_obus0_regs, OBJECT(npu),
                          &pnv_obus0_xscom_ops, npu,
                          "xscom-obus0",
                          PNV9_XSCOM_OBUS_SIZE);

    pnv_xscom_region_init(&npu->xscom_obus0_indirect_regs, OBJECT(npu),
                          &pnv_obus0_xscom_indirect_ops, npu,
                          "xscom-obus0-indirect",
                          PNV9_XSCOM_OBUS_INDIRECT_SIZE);

    /* generation ID bar, used for config space access on opencapi */
    memory_region_init_io(&npu->genid_mr[1], OBJECT(npu),
                          &pnv_npu2_genid_mmio_ops,
                          npu, "genid-stack1", PNV9_NPU_GENID_SIZE);
}

static Property pnv_npu2_properties[] = {
    DEFINE_PROP_LINK("chip", PnvNpu2, chip, TYPE_PNV_CHIP, PnvChip *),
    DEFINE_PROP_END_OF_LIST(),
};

static void pnv_npu2_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PnvXScomInterfaceClass *xscomc = PNV_XSCOM_INTERFACE_CLASS(klass);

    xscomc->dt_xscom = pnv_npu2_dt_xscom;

    dc->desc = "PowerNV NPU2";
    dc->realize = pnv_npu2_realize;
    device_class_set_props(dc, pnv_npu2_properties);
}

static const TypeInfo pnv_npu2_info = {
    .name          = TYPE_PNV_NPU2,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(PnvNpu2),
    .class_init    = pnv_npu2_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_PNV_XSCOM_INTERFACE },
        { }
    }
};

static void pnv_npu_register_types(void)
{
    type_register_static(&pnv_npu2_info);
}

type_init(pnv_npu_register_types);
