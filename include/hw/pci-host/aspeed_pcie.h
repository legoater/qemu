/*
 * Aspeed PCIe host controller
 *
 * Copyright (c) 2022 Cédric Le Goater <clg@kaod.org>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef ASPEED_PCIE_H
#define ASPEED_PCIE_H

#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pcie_host.h"
#include "qom/object.h"

#define TYPE_ASPEED_PCIE_ROOT "aspeed-pcie-root"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedPCIERoot, ASPEED_PCIE_ROOT);

struct AspeedPCIERoot {
    PCIBridge parent_obj;
};

#define TYPE_ASPEED_PCIE_RC "aspeed-pcie-rc"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedPCIERc, ASPEED_PCIE_RC);

#define ASPEED_PCIE_RC_REGS (0xD0 >> 2)

struct AspeedPCIERc {
    PCIExpressHost parent_obj;

    char name[16];

    uint32_t bus_nr;
    uint64_t mmio_base, mmio_size;

    MemoryRegion mmio;
    MemoryRegion io;
    MemoryRegion mmio_window;
    MemoryRegion io_window;

    MemoryRegion reg_rc_mmio;
    uint32_t regs[ASPEED_PCIE_RC_REGS];
    qemu_irq irq;

    AspeedPCIERoot root;
};

/* bridge between AHB bus and PCIe RC. */
#define TYPE_ASPEED_PCIE_CFG "aspeed-pcie-cfg"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedPCIECfg, ASPEED_PCIE_CFG);

#define ASPEED_PCIE_CFG_REGS (0xD0 >> 2)

struct AspeedPCIECfg {
    SysBusDevice parent_obj;

    MemoryRegion reg_mmio_container;
    MemoryRegion reg_cfg_mmio;
    uint32_t regs[ASPEED_PCIE_CFG_REGS];

    AspeedPCIERc rcs[2];
};


#define TYPE_ASPEED_PCIE_PHY "aspeed-pcie-phy"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedPCIEPhy, ASPEED_PCIE_PHY);

#define ASPEED_PCIE_PHY_REGS (0xD0 >> 2)

struct AspeedPCIEPhy {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[ASPEED_PCIE_PHY_REGS];
};

#endif /* ASPEED_PCIE_H */
