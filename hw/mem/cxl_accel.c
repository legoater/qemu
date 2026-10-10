/*
 * CXL accel (type-2) device
 *
 * Copyright(C) 2024 NVIDIA Corporation.
 *
 * This work is licensed under the terms of the GNU GPL, version 2. See the
 * COPYING file in the top-level directory.
 *
 * SPDX-License-Identifier: GPL-v2-only
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "hw/mem/memory-device.h"
#include "hw/mem/pc-dimm.h"
#include "hw/pci/pci.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "qemu/module.h"
#include "qemu/range.h"
#include "system/hostmem.h"
#include "system/numa.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_host.h"
#include "hw/pci/msix.h"
#include "hw/pci/pci_bus.h"
#include <zlib.h>
#include "hw/pci/pci_bridge.h"
#include "trace.h"

static void update_dvsecs(CXLAccelDev *acceld)
{
    CXLComponentState *cxl_cstate = &acceld->cxl_cstate;
    CXLFixedWindow *fw;
    uint8_t *dvsec;
    uint32_t range1_size_hi = 0, range1_size_lo = 0,
             range1_base_hi = 0, range1_base_lo = 0;

    if (acceld->hostvmem) {
        range1_size_hi = acceld->hostvmem->size >> 32;
        range1_size_lo = (2 << 5) | (2 << 2) | 0x3 |
                         (acceld->hostvmem->size & 0xF0000000);
    }

    fw = cxl_fmw_find_non_interleaved(PCI_DEVICE(acceld));
    if (fw) {
        range1_base_hi = fw->base >> 32;
        range1_base_lo = fw->base & 0xF0000000;
    }

    dvsec = (uint8_t *)&(CXLDVSECDevice){
        /* Cache, IO, Mem, Rst, RstMemClr capable */
        .cap = 0x88f | (acceld->hdm_decoders << 4),
        .ctrl = 0x7, /* Cache, IO, Mem enable */
        .status2 = 0x2,
        .range1_size_hi = range1_size_hi,
        .range1_size_lo = range1_size_lo,
        .range1_base_hi = range1_base_hi,
        .range1_base_lo = range1_base_lo,
    };
    cxl_component_update_dvsec(cxl_cstate, PCIE_CXL_DEVICE_DVSEC_LENGTH,
                               PCIE_CXL_DEVICE_DVSEC, dvsec);

    dvsec = (uint8_t *)&(CXLDVSECRegisterLocator){
        .rsvd         = 0,
        .reg0_base_lo = RBI_COMPONENT_REG | CXL_COMPONENT_REG_BAR_IDX,
        .reg0_base_hi = 0,
        .reg1_base_lo = RBI_CXL_DEVICE_REG | CXL_DEVICE_REG_BAR_IDX,
        .reg1_base_hi = 0,
    };
    cxl_component_update_dvsec(cxl_cstate, REG_LOC_DVSEC_LENGTH,
                               REG_LOC_DVSEC, dvsec);

    dvsec = (uint8_t *)&(CXLDVSECPortFlexBus){
        .cap                     = 0x26, /* 68B, IO, Mem, non-MLD */
        .ctrl                    = 0x02, /* IO always enabled */
        .status                  = 0x26, /* same as capabilities */
        .rcvd_mod_ts_data_phase1 = 0xef, /* WTF? */
    };
    cxl_component_update_dvsec(cxl_cstate, PCIE_CXL3_FLEXBUS_PORT_DVSEC_LENGTH,
                               PCIE_FLEXBUS_PORT_DVSEC, dvsec);
}

static void build_dvsecs(CXLAccelDev *acceld)
{
    CXLComponentState *cxl_cstate = &acceld->cxl_cstate;

    cxl_component_create_dvsec(cxl_cstate, CXL3_TYPE2_DEVICE,
                               PCIE_CXL_DEVICE_DVSEC_LENGTH,
                               PCIE_CXL_DEVICE_DVSEC,
                               PCIE_CXL31_DEVICE_DVSEC_REVID, NULL);

    cxl_component_create_dvsec(cxl_cstate, CXL3_TYPE2_DEVICE,
                               REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC,
                               REG_LOC_DVSEC_REVID, NULL);

    cxl_component_create_dvsec(cxl_cstate, CXL3_TYPE2_DEVICE,
                               PCIE_CXL3_FLEXBUS_PORT_DVSEC_LENGTH,
                               PCIE_FLEXBUS_PORT_DVSEC,
                               PCIE_CXL3_FLEXBUS_PORT_DVSEC_REVID, NULL);
    update_dvsecs(acceld);
}

static bool cxl_accel_dpa(CXLAccelDev *acceld, hwaddr host_addr, uint64_t *dpa)
{
    return cxl_host_addr_to_dpa(&acceld->cxl_cstate, host_addr, dpa);
}

static int cxl_accel_hpa_to_as_and_dpa(CXLAccelDev *acceld,
                                       hwaddr host_addr,
                                       unsigned int size,
                                       AddressSpace **as,
                                       uint64_t *dpa_offset)
{
    MemoryRegion *vmr = NULL;
    uint64_t vmr_size = 0;

    if (!acceld->hostvmem) {
        return -ENODEV;
    }

    vmr = host_memory_backend_get_memory(acceld->hostvmem);
    if (!vmr) {
        return -ENODEV;
    }

    vmr_size = memory_region_size(vmr);

    if (!cxl_accel_dpa(acceld, host_addr, dpa_offset)) {
        return -EINVAL;
    }

    if (*dpa_offset >= vmr_size) {
        return -EINVAL;
    }

    *as = &acceld->hostvmem_as;
    return 0;
}

MemTxResult cxl_accel_read(PCIDevice *d, hwaddr host_addr, uint64_t *data,
                           unsigned size, MemTxAttrs attrs)
{
    CXLAccelDev *acceld = CXL_ACCEL(d);
    uint64_t dpa_offset = 0;
    AddressSpace *as = NULL;
    int res;

    res = cxl_accel_hpa_to_as_and_dpa(acceld, host_addr, size,
                                      &as, &dpa_offset);
    if (res) {
        return MEMTX_ERROR;
    }

    return address_space_read(as, dpa_offset, attrs, data, size);
}

MemTxResult cxl_accel_write(PCIDevice *d, hwaddr host_addr, uint64_t data,
                            unsigned size, MemTxAttrs attrs)
{
    CXLAccelDev *acceld = CXL_ACCEL(d);
    uint64_t dpa_offset = 0;
    AddressSpace *as = NULL;
    int res;

    res = cxl_accel_hpa_to_as_and_dpa(acceld, host_addr, size,
                                      &as, &dpa_offset);
    if (res) {
        return MEMTX_ERROR;
    }

    return address_space_write(as, dpa_offset, attrs, &data, size);
}

/* Command BAR capability registers (read-only) */
#define CXL_ACCEL_CAP_MAGIC        0x00
#define CXL_ACCEL_CAP_VERSION      0x04
#define CXL_ACCEL_CAP_CAPS         0x08
#define CXL_ACCEL_CAP_MAX_XFER     0x0C

/* Command BAR register offsets */
#define CXL_ACCEL_CMD_BASE         0x10
#define CXL_ACCEL_CMD_OPCODE       (CXL_ACCEL_CMD_BASE + 0x00)
#define CXL_ACCEL_CMD_SRC_OFFSET   (CXL_ACCEL_CMD_BASE + 0x08)
#define CXL_ACCEL_CMD_SRC_LENGTH   (CXL_ACCEL_CMD_BASE + 0x10)
#define CXL_ACCEL_CMD_DST_OFFSET   (CXL_ACCEL_CMD_BASE + 0x18)
#define CXL_ACCEL_CMD_DST_LENGTH   (CXL_ACCEL_CMD_BASE + 0x20)
#define CXL_ACCEL_CMD_DOORBELL     (CXL_ACCEL_CMD_BASE + 0x28)
#define CXL_ACCEL_CMD_STATUS       (CXL_ACCEL_CMD_BASE + 0x30)
#define CXL_ACCEL_CMD_RESULT_LEN   (CXL_ACCEL_CMD_BASE + 0x38)
#define CXL_ACCEL_CMD_BAR_SIZE     4096

/* Capability register values */
#define CXL_ACCEL_MAGIC            0x32584C43  /* "CLX2" */
#define CXL_ACCEL_VERSION          1
#define CXL_ACCEL_MAX_XFER         (256 * 1024 * 1024)

/* Opcodes (bit positions in CAPS register) */
#define CXL_ACCEL_OP_COMPRESS      0x01
#define CXL_ACCEL_OP_DECOMPRESS    0x02
#define CXL_ACCEL_SUPPORTED_OPS    ((1 << CXL_ACCEL_OP_COMPRESS) | \
                                    (1 << CXL_ACCEL_OP_DECOMPRESS))

/* Status values */
#define CXL_ACCEL_STATUS_IDLE      0x00
#define CXL_ACCEL_STATUS_DONE      0x02
#define CXL_ACCEL_STATUS_ERROR     0x03

#define CXL_ACCEL_CMD_BAR_IDX      5
#define CXL_ACCEL_MSIX_CMD         2

static void cxl_accel_process_cmd(CXLAccelDev *acceld)
{
    MemoryRegion *vmr;
    uint64_t vmr_size;
    uint8_t *buf, *src, *dst;
    uLongf dst_len;
    int rc;

    if (!acceld->hostvmem) {
        acceld->cmd_status = CXL_ACCEL_STATUS_ERROR;
        return;
    }

    vmr = host_memory_backend_get_memory(acceld->hostvmem);
    vmr_size = memory_region_size(vmr);
    buf = memory_region_get_ram_ptr(vmr);

    if (acceld->cmd_src_offset + acceld->cmd_src_length > vmr_size ||
        acceld->cmd_dst_offset + acceld->cmd_dst_length > vmr_size) {
        acceld->cmd_status = CXL_ACCEL_STATUS_ERROR;
        return;
    }

    src = buf + acceld->cmd_src_offset;
    dst = buf + acceld->cmd_dst_offset;
    dst_len = acceld->cmd_dst_length;

    switch (acceld->cmd_opcode) {
    case CXL_ACCEL_OP_COMPRESS:
        rc = compress2(dst, &dst_len, src, acceld->cmd_src_length,
                       Z_DEFAULT_COMPRESSION);
        break;
    case CXL_ACCEL_OP_DECOMPRESS:
        rc = uncompress(dst, &dst_len, src, acceld->cmd_src_length);
        break;
    default:
        acceld->cmd_status = CXL_ACCEL_STATUS_ERROR;
        return;
    }

    if (rc != Z_OK) {
        acceld->cmd_status = CXL_ACCEL_STATUS_ERROR;
        return;
    }

    acceld->cmd_result_len = dst_len;
    acceld->cmd_status = CXL_ACCEL_STATUS_DONE;
}

static uint64_t cxl_accel_cmd_read(void *opaque, hwaddr addr, unsigned size)
{
    CXLAccelDev *acceld = opaque;

    switch (addr) {
    /* Capability registers */
    case CXL_ACCEL_CAP_MAGIC:
        return CXL_ACCEL_MAGIC;
    case CXL_ACCEL_CAP_VERSION:
        return CXL_ACCEL_VERSION;
    case CXL_ACCEL_CAP_CAPS:
        return CXL_ACCEL_SUPPORTED_OPS;
    case CXL_ACCEL_CAP_MAX_XFER:
        return CXL_ACCEL_MAX_XFER;
    /* Command registers */
    case CXL_ACCEL_CMD_OPCODE:
        return acceld->cmd_opcode;
    case CXL_ACCEL_CMD_SRC_OFFSET:
        return acceld->cmd_src_offset;
    case CXL_ACCEL_CMD_SRC_LENGTH:
        return acceld->cmd_src_length;
    case CXL_ACCEL_CMD_DST_OFFSET:
        return acceld->cmd_dst_offset;
    case CXL_ACCEL_CMD_DST_LENGTH:
        return acceld->cmd_dst_length;
    case CXL_ACCEL_CMD_STATUS:
        return acceld->cmd_status;
    case CXL_ACCEL_CMD_RESULT_LEN:
        return acceld->cmd_result_len;
    default:
        return 0;
    }
}

static void cxl_accel_cmd_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    CXLAccelDev *acceld = opaque;

    switch (addr) {
    case CXL_ACCEL_CMD_OPCODE:
        acceld->cmd_opcode = val;
        break;
    case CXL_ACCEL_CMD_SRC_OFFSET:
        acceld->cmd_src_offset = val;
        break;
    case CXL_ACCEL_CMD_SRC_LENGTH:
        acceld->cmd_src_length = val;
        break;
    case CXL_ACCEL_CMD_DST_OFFSET:
        acceld->cmd_dst_offset = val;
        break;
    case CXL_ACCEL_CMD_DST_LENGTH:
        acceld->cmd_dst_length = val;
        break;
    case CXL_ACCEL_CMD_DOORBELL:
        if (val) {
            cxl_accel_process_cmd(acceld);
            msix_notify(PCI_DEVICE(acceld), CXL_ACCEL_MSIX_CMD);
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps cxl_accel_cmd_ops = {
    .read = cxl_accel_cmd_read,
    .write = cxl_accel_cmd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void setup_cmd_bar(PCIDevice *pci_dev)
{
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);

    memory_region_init_io(&acceld->cmd_bar, OBJECT(pci_dev),
                          &cxl_accel_cmd_ops, acceld,
                          "cxl-accel-cmd", CXL_ACCEL_CMD_BAR_SIZE);
    pci_register_bar(pci_dev, CXL_ACCEL_CMD_BAR_IDX,
                     PCI_BASE_ADDRESS_SPACE_MEMORY, &acceld->cmd_bar);
}

static void clean_memory(PCIDevice *pci_dev)
{
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);

    if (acceld->hostvmem) {
        address_space_destroy(&acceld->hostvmem_as);
    }
}

static bool setup_memory(PCIDevice *pci_dev, Error **errp)
{
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);

    if (acceld->hostvmem) {
        MemoryRegion *vmr;
        char *v_name;

        vmr = host_memory_backend_get_memory(acceld->hostvmem);
        if (!vmr) {
            error_setg(errp, "volatile memdev must have backing device");
            return false;
        }
        if (host_memory_backend_is_mapped(acceld->hostvmem)) {
            error_setg(errp, "memory backend %s can't be used multiple times.",
               object_get_canonical_path_component(OBJECT(acceld->hostvmem)));
            return false;
        }
        memory_region_set_nonvolatile(vmr, false);
        memory_region_set_enabled(vmr, true);
        host_memory_backend_set_mapped(acceld->hostvmem, true);
        v_name = g_strdup("cxl-accel-dpa-vmem-space");
        address_space_init(&acceld->hostvmem_as, vmr, v_name);
        g_free(v_name);
    }
    return true;
}

static void setup_cxl_regs(PCIDevice *pci_dev)
{
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);
    CXLComponentState *cxl_cstate = &acceld->cxl_cstate;
    ComponentRegisters *regs = &cxl_cstate->crb;
    MemoryRegion *mr = &regs->component_registers;

    cxl_cstate->dvsec_offset = 0x100;
    cxl_cstate->pdev = pci_dev;

    build_dvsecs(acceld);

    cxl_component_register_block_init(OBJECT(pci_dev), cxl_cstate,
                                      TYPE_CXL_ACCEL);

    pci_register_bar(
        pci_dev, CXL_COMPONENT_REG_BAR_IDX,
        PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, mr);

    cxl_device_register_block_init(OBJECT(pci_dev), &acceld->cxl_dstate,
                                   &acceld->cci);
    pci_register_bar(pci_dev, CXL_DEVICE_REG_BAR_IDX,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                         PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &acceld->cxl_dstate.device_registers);
}

#define CXL_ACCEL_MSIX_NUM      6
#define CXL_ACCEL_MSIX_MBOX    0

static bool setup_msix(PCIDevice *pci_dev, Error **errp)
{
    int i, rc;

    rc = msix_init_exclusive_bar(pci_dev, CXL_ACCEL_MSIX_NUM, 4, errp);
    if (rc) {
        return false;
    }

    for (i = 0; i < CXL_ACCEL_MSIX_NUM; i++) {
        msix_vector_use(pci_dev, i);
    }
    return true;
}

static void cxl_accel_realize(PCIDevice *pci_dev, Error **errp)
{
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);
    uint8_t *pci_conf = pci_dev->config;

    if (!setup_memory(pci_dev, errp)) {
        return;
    }

    pci_config_set_prog_interface(pci_conf, 0x10);
    pcie_endpoint_cap_init(pci_dev, 0x80);

    setup_cxl_regs(pci_dev);
    setup_cmd_bar(pci_dev);

    if (!setup_msix(pci_dev, errp)) {
        clean_memory(pci_dev);
        return;
    }

    cxl_event_init(&acceld->cxl_dstate, CXL_ACCEL_MSIX_MBOX + 1);
}

static void cxl_accel_exit(PCIDevice *pci_dev)
{
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);

    cxl_destroy_cci(&acceld->cci);
    clean_memory(pci_dev);
}

/*
 * Mimic firmware behavior: program endpoint HDM decoder0 with the
 * HPA range from the CFMWS window and mark it committed. This is
 * what real firmware does before the OS boots, allowing the kernel
 * CXL core to discover an auto-region for the device.
 */
static void commit_hdm_decoder0(uint32_t *cache_mem,
                                uint64_t base, uint64_t size)
{
    uint32_t ctrl;

    stl_le_p(cache_mem + R_CXL_HDM_DECODER0_BASE_LO, base & 0xF0000000);
    stl_le_p(cache_mem + R_CXL_HDM_DECODER0_BASE_HI, base >> 32);
    stl_le_p(cache_mem + R_CXL_HDM_DECODER0_SIZE_LO, size & 0xF0000000);
    stl_le_p(cache_mem + R_CXL_HDM_DECODER0_SIZE_HI, size >> 32);

    ctrl = ldl_le_p(cache_mem + R_CXL_HDM_DECODER0_CTRL);
    ctrl = FIELD_DP32(ctrl, CXL_HDM_DECODER0_CTRL, ERR, 0);
    ctrl = FIELD_DP32(ctrl, CXL_HDM_DECODER0_CTRL, COMMITTED, 1);
    stl_le_p(cache_mem + R_CXL_HDM_DECODER0_CTRL, ctrl);
}

static void commit_hb_hdm_decoder0(PXBCXLDev *pxb, uint64_t base,
                                   uint64_t size)
{
    PCIHostState *hb = PCI_HOST_BRIDGE(pxb->cxl_host_bridge);
    CXLComponentState *hb_cstate = cxl_get_hb_cstate(hb);

    if (hb_cstate) {
        commit_hdm_decoder0(hb_cstate->crb.cache_mem_registers, base, size);
    }
}

static void cxl_accel_fw_commit_hdm(CXLAccelDev *acceld)
{
    CXLFixedWindow *fw;

    if (!acceld->hostvmem) {
        return;
    }

    fw = cxl_fmw_find_non_interleaved(PCI_DEVICE(acceld));
    if (!fw) {
        return;
    }

    trace_cxl_accel_fw_commit_hdm(fw->base, acceld->hostvmem->size);

    /* Commit host bridge HDM decoder0 */
    commit_hb_hdm_decoder0(fw->target_hbs[0], fw->base,
                           acceld->hostvmem->size);

    /* Commit endpoint HDM decoder0 */
    commit_hdm_decoder0(acceld->cxl_cstate.crb.cache_mem_registers,
                        fw->base, acceld->hostvmem->size);

    cfmws_update_non_interleaved(true);
}

/*
 * Program HDM decoders in the exit phase so that the host bridge
 * hold phase (which memsets its registers to zero) has already run.
 */
static void cxl_accel_reset_exit(Object *obj, ResetType type)
{
    cxl_accel_fw_commit_hdm(CXL_ACCEL(obj));
}

static void cxl_accel_reset_hold(Object *obj, ResetType type)
{
    CXLAccelDev *acceld = CXL_ACCEL(obj);
    CXLComponentState *cxl_cstate = &acceld->cxl_cstate;
    uint32_t *reg_state = cxl_cstate->crb.cache_mem_registers;
    uint32_t *write_msk = cxl_cstate->crb.cache_mem_regs_write_mask;

    acceld->cmd_opcode = 0;
    acceld->cmd_src_offset = 0;
    acceld->cmd_src_length = 0;
    acceld->cmd_dst_offset = 0;
    acceld->cmd_dst_length = 0;
    acceld->cmd_status = CXL_ACCEL_STATUS_IDLE;
    acceld->cmd_result_len = 0;

    update_dvsecs(acceld);
    cxl_component_register_init_common(reg_state, write_msk, CXL3_TYPE2_DEVICE,
                                       false);
    ARRAY_FIELD_DP32(reg_state, CXL_HDM_DECODER_CAPABILITY, DECODER_COUNT,
                     cxl_decoder_count_enc(acceld->hdm_decoders));
    if (acceld->cci.initialized) {
        cxl_destroy_cci(&acceld->cci);
    }
    cxl_device_register_init_accel(acceld, CXL_ACCEL_MSIX_MBOX);
}

static const Property cxl_accel_props[] = {
    DEFINE_PROP_LINK("volatile-memdev", CXLAccelDev, hostvmem,
                     TYPE_MEMORY_BACKEND, HostMemoryBackend *),
    DEFINE_PROP_UINT8("hdm-decoders", CXLAccelDev, hdm_decoders, 1),
};

static void cxl_accel_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    pc->realize = cxl_accel_realize;
    pc->exit = cxl_accel_exit;

    pc->class_id = PCI_CLASS_CXL_QEMU_ACCEL;
    pc->vendor_id = PCI_VENDOR_ID_REDHAT;
    pc->device_id = 0x0123;
    pc->revision = 1;

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "CXL Accelerator Device (Type 2)";
    rc->phases.hold = cxl_accel_reset_hold;
    rc->phases.exit = cxl_accel_reset_exit;
    device_class_set_props(dc, cxl_accel_props);
}

static const TypeInfo cxl_accel_dev_info = {
    .name = TYPE_CXL_ACCEL,
    .parent = TYPE_PCI_DEVICE,
    .class_size = sizeof(struct CXLAccelClass),
    .class_init = cxl_accel_class_init,
    .instance_size = sizeof(CXLAccelDev),
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CXL_DEVICE },
        { INTERFACE_PCIE_DEVICE },
        {}
    },
};

static void cxl_accel_dev_registers(void)
{
    type_register_static(&cxl_accel_dev_info);
}

type_init(cxl_accel_dev_registers);
