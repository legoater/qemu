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
#include "hw/pci/pcie_port.h"

static void pnv_phb3_rc_write_config(PCIDevice *d,
                                     uint32_t address, uint32_t val, int len)
{
    uint32_t root_cmd =
        pci_get_long(d->config + d->exp.aer_cap + PCI_ERR_ROOT_COMMAND);

    pci_bridge_write_config(d, address, val, len);
    pcie_cap_slot_write_config(d, address, val, len);
    pcie_aer_write_config(d, address, val, len);
    pcie_aer_root_write_config(d, address, val, len, root_cmd);
}

static void pnv_phb3_rc_reset(DeviceState *qdev)
{
    PCIDevice *d = PCI_DEVICE(qdev);

    pcie_cap_root_reset(d);
    pcie_cap_deverr_reset(d);
    pcie_cap_slot_reset(d);
    pcie_cap_arifwd_reset(d);
    pcie_aer_root_reset(d);
    pci_bridge_reset(qdev);
    pci_bridge_disable_base_limit(d);
}

static int pnv_phb3_rc_initfn(PCIDevice *d)
{
    PCIEPort *p = PCIE_PORT(d);
    PCIESlot *s = PCIE_SLOT(d);
    int rc;

    DEVICE(d)->id = "pcie";
    pci_bridge_initfn(d, TYPE_PCIE_BUS);

    /* XXX Make that a property ? Allow for only one device (8 functions) */
    pci_bridge_get_sec_bus(PCI_BRIDGE(d))->devfn_max = 8;

    pcie_port_init_reg(d);

    rc = pcie_cap_init(d, 0x48, PCI_EXP_TYPE_ROOT_PORT, p->port);
    if (rc < 0) {
        printf("phb3-rc: pcie_cap_init() error %d !\n", rc);
        goto err_bridge;
    }
    pcie_cap_arifwd_init(d);
    pcie_cap_deverr_init(d);
    pcie_cap_slot_init(d, s->slot);
    pcie_chassis_create(s->chassis);
    rc = pcie_chassis_add_slot(s);
    if (rc < 0) {
        printf("phb3-rc: pcie_chassis_add_slot() error %d !\n", rc);
        goto err_pcie_cap;
    }
    pcie_cap_root_init(d);
    rc = pcie_aer_init(d, 0x100, PCI_ERR_SIZEOF);
    if (rc < 0) {
        printf("phb3-rc: pcie_aer_init() error %d !\n", rc);
        goto err_slot;
    }
    pcie_aer_root_init(d);
    return 0;

err_slot:
    pcie_chassis_del_slot(s);
err_pcie_cap:
    pcie_cap_exit(d);
err_bridge:
    pci_bridge_exitfn(d);
    return rc;
}

static void pnv_phb3_rc_exitfn(PCIDevice *d)
{
    PCIESlot *s = PCIE_SLOT(d);

    pcie_aer_exit(d);
    pcie_chassis_del_slot(s);
    pcie_cap_exit(d);
    pci_bridge_exitfn(d);
}

static void pnv_phb3_rc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->is_express = 1;
    k->is_bridge = 1;
    k->init = pnv_phb3_rc_initfn;
    k->exit = pnv_phb3_rc_exitfn;
    k->config_write = pnv_phb3_rc_write_config;
    k->vendor_id = PCI_VENDOR_ID_IBM;
    k->device_id = 0x03dc;
    k->revision = 0;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->desc = "IBM PHB3 PCIE Root Port";
    dc->reset = pnv_phb3_rc_reset;
}

static const TypeInfo pnv_phb3_rc_info = {
    .name          = TYPE_PNV_PHB3_RC,
    .parent        = TYPE_PCIE_SLOT,
    .class_init    = pnv_phb3_rc_class_init,
};

static void pnv_phb3_rc_register_types(void)
{
    type_register_static(&pnv_phb3_rc_info);
}

type_init(pnv_phb3_rc_register_types)
