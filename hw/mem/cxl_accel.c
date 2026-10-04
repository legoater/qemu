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
#include "hw/pci/msix.h"

static void update_dvsecs(CXLAccelDev *acceld)
{
    CXLComponentState *cxl_cstate = &acceld->cxl_cstate;
    uint8_t *dvsec;
    uint32_t range1_size_hi = 0, range1_size_lo = 0,
             range1_base_hi = 0, range1_base_lo = 0;

    if (acceld->hostvmem) {
        range1_size_hi = acceld->hostvmem->size >> 32;
        range1_size_lo = (2 << 5) | (2 << 2) | 0x3 |
                         (acceld->hostvmem->size & 0xF0000000);
    }

    dvsec = (uint8_t *)&(CXLDVSECDevice){
        .cap = 0x1e,
        .ctrl = 0x2,
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

static int setup_msix(PCIDevice *pci_dev)
{
    int i, rc;

    /* MSI(-X) Initialization */
    rc = msix_init_exclusive_bar(pci_dev, CXL_ACCEL_MSIX_NUM, 4, NULL);
    if (rc) {
        return rc;
    }

    for (i = 0; i < CXL_ACCEL_MSIX_NUM; i++) {
        msix_vector_use(pci_dev, i);
    }
    return 0;
}

static void cxl_accel_realize(PCIDevice *pci_dev, Error **errp)
{
    ERRP_GUARD();
    CXLAccelDev *acceld = CXL_ACCEL(pci_dev);
    int rc;
    uint8_t *pci_conf = pci_dev->config;

    if (!setup_memory(pci_dev, errp)) {
        return;
    }

    pci_config_set_prog_interface(pci_conf, 0x10);
    pcie_endpoint_cap_init(pci_dev, 0x80);

    setup_cxl_regs(pci_dev);

    /* MSI(-X) Initialization */
    rc = setup_msix(pci_dev);
    if (rc) {
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

static void cxl_accel_reset(DeviceState *dev)
{
    CXLAccelDev *acceld = CXL_ACCEL(dev);
    CXLComponentState *cxl_cstate = &acceld->cxl_cstate;
    uint32_t *reg_state = cxl_cstate->crb.cache_mem_registers;
    uint32_t *write_msk = cxl_cstate->crb.cache_mem_regs_write_mask;

    update_dvsecs(acceld);
    cxl_component_register_init_common(reg_state, write_msk, CXL3_TYPE2_DEVICE,
                                       false);
    if (acceld->cci.initialized) {
        cxl_destroy_cci(&acceld->cci);
    }
    cxl_device_register_init_accel(acceld, CXL_ACCEL_MSIX_MBOX);
}

static const Property cxl_accel_props[] = {
    DEFINE_PROP_LINK("volatile-memdev", CXLAccelDev, hostvmem,
                     TYPE_MEMORY_BACKEND, HostMemoryBackend *),
};

static void cxl_accel_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->realize = cxl_accel_realize;
    pc->exit = cxl_accel_exit;

    pc->class_id = PCI_CLASS_CXL_QEMU_ACCEL;
    pc->vendor_id = PCI_VENDOR_ID_REDHAT;
    pc->device_id = 0x0123;
    pc->revision = 1;

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "CXL Accelerator Device (Type 2)";
    device_class_set_legacy_reset(dc, cxl_accel_reset);
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
