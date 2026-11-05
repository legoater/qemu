/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_host.h"
#include "hw/cxl/cxl_device.h"
#include "hw/cxl/cxl_component.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie_port.h"
#include "system/hostmem.h"
#include "monitor/hmp.h"
#include "monitor/monitor.h"
#include "qobject/qdict.h"

static void cxl_hmp_indent(MonitorHMP *hmp, int depth)
{
    int i;

    for (i = 0; i < depth; i++) {
        monitor_hmp_printf(hmp, "  ");
    }
}

static void cxl_print_fmw_restrictions(MonitorHMP *hmp, uint16_t restrictions)
{
    bool first = true;

    monitor_hmp_printf(hmp, " restrictions:");
    if (restrictions & CXL_FMW_DEVICE_COHERENT) {
        monitor_hmp_printf(hmp, "%sHDM-D", first ? " " : ", ");
        first = false;
    }
    if (restrictions & CXL_FMW_HOST_ONLY) {
        monitor_hmp_printf(hmp, "%sHDM-H", first ? " " : ", ");
        first = false;
    }
    if (restrictions & CXL_FMW_VOLATILE) {
        monitor_hmp_printf(hmp, "%svolatile", first ? " " : ", ");
        first = false;
    }
    if (restrictions & CXL_FMW_PERSISTENT) {
        monitor_hmp_printf(hmp, "%spersistent", first ? " " : ", ");
        first = false;
    }
    if (restrictions & CXL_FMW_BI) {
        monitor_hmp_printf(hmp, "%sBI", first ? " " : ", ");
        first = false;
    }
    if (first) {
        monitor_hmp_printf(hmp, " none");
    }
    monitor_hmp_printf(hmp, "\n");
}

static void cxl_print_hdm_decoders(MonitorHMP *hmp,
                                    CXLComponentState *cxl_cstate,
                                    int depth)
{
    uint32_t *cache_mem = cxl_cstate->crb.cache_mem_registers;
    int hdm_inc = R_CXL_HDM_DECODER1_BASE_LO - R_CXL_HDM_DECODER0_BASE_LO;
    uint32_t cap;
    unsigned int hdm_count;
    int i;

    cap = ldl_le_p(cache_mem + R_CXL_HDM_DECODER_CAPABILITY);
    hdm_count = cxl_decoder_count_dec(
        FIELD_EX32(cap, CXL_HDM_DECODER_CAPABILITY, DECODER_COUNT));

    for (i = 0; i < hdm_count; i++) {
        uint32_t low, high, ctrl;
        uint64_t base, size;
        bool committed;

        ctrl = ldl_le_p(cache_mem + R_CXL_HDM_DECODER0_CTRL + i * hdm_inc);
        committed = FIELD_EX32(ctrl, CXL_HDM_DECODER0_CTRL, COMMITTED);

        low = ldl_le_p(cache_mem + R_CXL_HDM_DECODER0_BASE_LO + i * hdm_inc);
        high = ldl_le_p(cache_mem + R_CXL_HDM_DECODER0_BASE_HI + i * hdm_inc);
        base = ((uint64_t)(low & 0xf0000000)) | ((uint64_t)high << 32);

        low = ldl_le_p(cache_mem + R_CXL_HDM_DECODER0_SIZE_LO + i * hdm_inc);
        high = ldl_le_p(cache_mem + R_CXL_HDM_DECODER0_SIZE_HI + i * hdm_inc);
        size = ((uint64_t)(low & 0xf0000000)) | ((uint64_t)high << 32);

        cxl_hmp_indent(hmp, depth);
        if (!committed && !size) {
            monitor_hmp_printf(hmp, "Decoder[%d]: disabled\n", i);
            continue;
        }

        monitor_hmp_printf(hmp,
            "Decoder[%d]: base 0x%016" PRIx64 " size 0x%016" PRIx64 " %s\n",
            i, base, size, committed ? "committed" : "not-committed");
    }
}

static void cxl_print_dvsec(MonitorHMP *hmp, CXLComponentState *cstate,
                            int depth)
{
    PCIDevice *pdev = cstate->pdev;
    Range *r = &cstate->dvsecs[PCIE_CXL_DEVICE_DVSEC];
    uint16_t offset;
    uint16_t cap, ctrl;
    uint64_t base, size;
    uint32_t hi, lo;
    int i;

    if (!range_size(r)) {
        return;
    }

    offset = r->lob;

    cap = pci_get_word(pdev->config + offset +
                       offsetof(CXLDVSECDevice, cap));
    ctrl = pci_get_word(pdev->config + offset +
                        offsetof(CXLDVSECDevice, ctrl));

    cxl_hmp_indent(hmp, depth);
    monitor_hmp_printf(hmp, "DVSEC cap: Cache%s IO%s Mem%s\n",
                       (cap & 0x1) ? "+" : "-",
                       (cap & 0x2) ? "+" : "-",
                       (cap & 0x4) ? "+" : "-");
    cxl_hmp_indent(hmp, depth);
    monitor_hmp_printf(hmp, "DVSEC ctrl: Cache%s IO%s Mem%s\n",
                       (ctrl & 0x1) ? "+" : "-",
                       (ctrl & 0x2) ? "+" : "-",
                       (ctrl & 0x4) ? "+" : "-");

    for (i = 0; i < 2; i++) {
        hi = pci_get_long(pdev->config + offset +
                          offsetof(CXLDVSECDevice, range1_size_hi) + i * 16);
        lo = pci_get_long(pdev->config + offset +
                          offsetof(CXLDVSECDevice, range1_size_lo) + i * 16);
        size = ((uint64_t)hi << 32) | (lo & 0xf0000000);
        if (!size) {
            continue;
        }

        hi = pci_get_long(pdev->config + offset +
                          offsetof(CXLDVSECDevice, range1_base_hi) + i * 16);
        lo = pci_get_long(pdev->config + offset +
                          offsetof(CXLDVSECDevice, range1_base_lo) + i * 16);
        base = ((uint64_t)hi << 32) | (lo & 0xf0000000);

        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp,
            "DVSEC Range%d: 0x%016" PRIx64 "-0x%016" PRIx64
            " [size=0x%" PRIx64 "]\n",
            i + 1, base, base + size - 1, size);
    }
}

static void cxl_print_type3_device(MonitorHMP *hmp, CXLType3Dev *ct3d,
                                   int depth)
{
    CXLDeviceState *cxl_dstate = &ct3d->cxl_dstate;

    if (cxl_dstate->vmem_size) {
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "Volatile: %" PRIu64 " MiB\n",
                           cxl_dstate->vmem_size >> 20);
    }
    if (cxl_dstate->pmem_size) {
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "Persistent: %" PRIu64 " MiB\n",
                           cxl_dstate->pmem_size >> 20);
    }
    if (ct3d->dc.num_regions) {
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "DC regions: %u, total: %" PRIu64 " MiB"
                           " (extents: %u)\n",
                           ct3d->dc.num_regions,
                           ct3d->dc.total_capacity >> 20,
                           ct3d->dc.total_extent_count);
    }
    if (ct3d->poison_list_cnt) {
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "Poison entries: %u%s\n",
                           ct3d->poison_list_cnt,
                           ct3d->poison_list_overflowed ? " (overflowed)" : "");
    }
    cxl_print_dvsec(hmp, &ct3d->cxl_cstate, depth);
    cxl_print_hdm_decoders(hmp, &ct3d->cxl_cstate, depth);
}

static void cxl_print_accel_device(MonitorHMP *hmp, CXLAccelDev *accel,
                                   int depth)
{
    if (accel->hostvmem) {
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "Volatile: %" PRIu64 " MiB\n",
                           accel->hostvmem->size >> 20);
    }
    cxl_print_dvsec(hmp, &accel->cxl_cstate, depth);
    cxl_hmp_indent(hmp, depth);
    monitor_hmp_printf(hmp, "HDM decoders: %u\n", accel->hdm_decoders);
    cxl_print_hdm_decoders(hmp, &accel->cxl_cstate, depth);
}

static void cxl_print_endpoint(MonitorHMP *hmp, PCIDevice *d, int depth)
{
    int bus_num = pci_dev_bus_num(d);

    if (object_dynamic_cast(OBJECT(d), TYPE_CXL_TYPE3)) {
        CXLType3Dev *ct3d = CXL_TYPE3(d);
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp,
            "Type-3 %02x:%02x.%x [%s]",
            bus_num, PCI_SLOT(d->devfn), PCI_FUNC(d->devfn), d->name);
        if (DEVICE(d)->id) {
            monitor_hmp_printf(hmp, " \"%s\"", DEVICE(d)->id);
        }
        monitor_hmp_printf(hmp, " (SN: %" PRIu64 ")\n", ct3d->sn);
        cxl_print_type3_device(hmp, ct3d, depth + 1);
    } else if (object_dynamic_cast(OBJECT(d), TYPE_CXL_ACCEL)) {
        CXLAccelDev *accel = CXL_ACCEL(d);
        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "Type-2 %02x:%02x.%x [%s]",
                           bus_num, PCI_SLOT(d->devfn), PCI_FUNC(d->devfn),
                           d->name);
        if (DEVICE(d)->id) {
            monitor_hmp_printf(hmp, " \"%s\"", DEVICE(d)->id);
        }
        monitor_hmp_printf(hmp, "\n");
        cxl_print_accel_device(hmp, accel, depth + 1);
    }
}

static void cxl_print_switch(MonitorHMP *hmp, CXLUpstreamPort *usp, int depth)
{
    PCIDevice *usp_dev = PCI_DEVICE(usp);
    PCIBus *sec_bus = &PCI_BRIDGE(usp_dev)->sec_bus;
    int i;

    cxl_hmp_indent(hmp, depth);
    monitor_hmp_printf(hmp, "Switch [%s]\n", usp_dev->name);
    cxl_print_hdm_decoders(hmp, cxl_usp_to_cstate(usp), depth + 1);

    for (i = 0; i < ARRAY_SIZE(sec_bus->devices); i++) {
        PCIDevice *dsp_dev = sec_bus->devices[i];
        PCIDevice *ep;

        if (!dsp_dev) {
            continue;
        }

        if (!object_dynamic_cast(OBJECT(dsp_dev), TYPE_CXL_DSP)) {
            continue;
        }

        ep = pci_bridge_get_sec_bus(PCI_BRIDGE(dsp_dev))->devices[0];
        if (!ep) {
            continue;
        }

        cxl_hmp_indent(hmp, depth + 1);
        monitor_hmp_printf(hmp, "DSP port %d:\n", PCIE_PORT(dsp_dev)->port);
        cxl_print_endpoint(hmp, ep, depth + 2);
    }
}

static void cxl_print_rp_devices(MonitorHMP *hmp, PCIBus *bus, int depth)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(bus->devices); i++) {
        PCIDevice *rp = bus->devices[i];
        PCIDevice *d;

        if (!rp) {
            continue;
        }

        if (!object_dynamic_cast(OBJECT(rp), "cxl-rp")) {
            continue;
        }

        d = pci_bridge_get_sec_bus(PCI_BRIDGE(rp))->devices[0];
        if (!d) {
            continue;
        }

        cxl_hmp_indent(hmp, depth);
        monitor_hmp_printf(hmp, "Root Port %d %02x:%02x.%x",
                           PCIE_PORT(rp)->port,
                           pci_dev_bus_num(rp),
                           PCI_SLOT(rp->devfn), PCI_FUNC(rp->devfn));
        if (DEVICE(rp)->id) {
            monitor_hmp_printf(hmp, " \"%s\"", DEVICE(rp)->id);
        }
        monitor_hmp_printf(hmp, ":\n");

        if (object_dynamic_cast(OBJECT(d), TYPE_CXL_USP)) {
            cxl_print_switch(hmp, CXL_USP(d), depth + 1);
        } else {
            cxl_print_endpoint(hmp, d, depth + 1);
        }
    }
}

void hmp_info_cxl(MonitorHMP *hmp, const QDict *qdict)
{
    GSList *cfmws_list, *iter;

    cfmws_list = cxl_fmws_get_all_sorted();
    if (!cfmws_list) {
        monitor_hmp_printf(hmp, "No CXL Fixed Memory Windows found\n");
        return;
    }

    for (iter = cfmws_list; iter; iter = iter->next) {
        CXLFixedWindow *fw = CXL_FMW(iter->data);
        int i;

        monitor_hmp_printf(hmp,
            "CFMWS[%d]: base 0x%016" PRIx64 " size 0x%016" PRIx64
            " interleave %d-way",
            fw->index, fw->base, fw->size, fw->num_targets);
        cxl_print_fmw_restrictions(hmp, fw->restrictions);

        for (i = 0; i < fw->num_targets; i++) {
            PXBCXLDev *pxb = fw->target_hbs[i];
            PCIHostState *hb;

            if (!pxb || !pxb->cxl_host_bridge) {
                continue;
            }

            hb = PCI_HOST_BRIDGE(pxb->cxl_host_bridge);
            if (!hb || !hb->bus) {
                continue;
            }

            cxl_hmp_indent(hmp, 1);
            monitor_hmp_printf(hmp, "Host Bridge[%d] %02x:%02x.%x",
                               i, pci_dev_bus_num(PCI_DEVICE(pxb)),
                               PCI_SLOT(PCI_DEVICE(pxb)->devfn),
                               PCI_FUNC(PCI_DEVICE(pxb)->devfn));
            if (DEVICE(pxb)->id) {
                monitor_hmp_printf(hmp, " \"%s\"", DEVICE(pxb)->id);
            }
            if (cxl_get_hb_passthrough(hb)) {
                monitor_hmp_printf(hmp, " (passthrough)");
            }
            monitor_hmp_printf(hmp, "\n");

            cxl_print_hdm_decoders(hmp, cxl_get_hb_cstate(hb), 2);
            cxl_print_rp_devices(hmp, hb->bus, 2);
        }
        monitor_hmp_printf(hmp, "\n");
    }

    g_slist_free(cfmws_list);
}
