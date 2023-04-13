/*
 * Aspeed PCIe host controller
 *
 * Copyright (c) 2022 Cédric Le Goater <clg@kaod.org>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/pci-host/aspeed_pcie.h"
#include "hw/pci/msi.h"

#include "trace.h"

#define TO_REG(offset) ((offset) >> 2)

#define PCI_VENDOR_ID_ASPEED       0x1A03
#define PCI_DEVICE_ID_AST2600_RC   0x1150

/*
 * RC
 */

#define ASPEED_PCIE_RC_CTRL             0x00
#define ASPEED_PCIE_RC_CTRL_RCL            BIT(0)
#define ASPEED_PCIE_RC_CTRL_RCL_RX         BIT(1)
#define ASPEED_PCIE_RC_CTRL_RX_WAIT_FW     BIT(2)
#define ASPEED_PCIE_RC_CTRL_RX_UNLOCK      BIT(4)
#define ASPEED_PCIE_RC_CTRL_RX_MSI         BIT(6)
#define ASPEED_PCIE_RC_CTRL_RX_MSI_SELECT  BIT(7)
#define ASPEED_PCIE_RC_CTRL_RX_LINEAR      BIT(8)
#define ASPEED_PCIE_RC_CTRL_RX_DMA         BIT(9)
#define ASPEED_PCIE_RC_CTRL_RX_TAG_MASK    0x00ff0000

#define ASPEED_PCIE_RC_INT_ENABLE       0x04
#define   ASPEED_PCIE_RC_INT_ENABLE_INTA   BIT(0)
#define   ASPEED_PCIE_RC_INT_ENABLE_INTB   BIT(1)
#define   ASPEED_PCIE_RC_INT_ENABLE_INTC   BIT(2)
#define   ASPEED_PCIE_RC_INT_ENABLE_INTD   BIT(3)
#define   ASPEED_PCIE_RC_INT_ENABLE_RX     BIT(4)
#define ASPEED_PCIE_RC_INT_STATUS       0x08
#define   ASPEED_PCIE_RC_INT_STATUS_INTA   BIT(0)
#define   ASPEED_PCIE_RC_INT_STATUS_INTB   BIT(1)
#define   ASPEED_PCIE_RC_INT_STATUS_INTC   BIT(2)
#define   ASPEED_PCIE_RC_INT_STATUS_INTD   BIT(3)
#define   ASPEED_PCIE_RC_INT_STATUS_INTX_MASK 0xf
#define   ASPEED_PCIE_RC_INT_STATUS_RX     BIT(4)

#define ASPEED_PCIE_RC_RX_DATA  0x0C
#define ASPEED_PCIE_RC_RX_DW0   0x10
#define ASPEED_PCIE_RC_RX_DW1   0x14
#define ASPEED_PCIE_RC_RX_DW2   0x18
#define ASPEED_PCIE_RC_RX_DW3   0x1C
#define ASPEED_PCIE_RC_MSI_ENABLE0      0x20
#define ASPEED_PCIE_RC_MSI_ENABLE1      0x24
#define ASPEED_PCIE_RC_MSI_STATUS0      0x28
#define ASPEED_PCIE_RC_MSI_STATUS1      0x2C
#define ASPEED_PCIE_RC_TX_TAG           0x3C

static void aspeed_pcie_rc_update_irq(AspeedPCIERc *s)
{
    bool intx = !!(s->regs[TO_REG(ASPEED_PCIE_RC_INT_STATUS)] &
                   s->regs[TO_REG(ASPEED_PCIE_RC_INT_ENABLE)]);
    bool msi0 = !!(s->regs[TO_REG(ASPEED_PCIE_RC_MSI_STATUS0)] &
                   s->regs[TO_REG(ASPEED_PCIE_RC_MSI_ENABLE0)]);
    bool msi1 = !!(s->regs[TO_REG(ASPEED_PCIE_RC_MSI_STATUS1)] &
                   s->regs[TO_REG(ASPEED_PCIE_RC_MSI_ENABLE1)]);
    bool level = intx || msi0 || msi1;

    qemu_set_irq(s->irq, level);
}

static uint64_t aspeed_pcie_rc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    AspeedPCIERc *s = ASPEED_PCIE_RC(opaque);
    uint64_t val = 0;

    switch (addr) {
    case ASPEED_PCIE_RC_CTRL:
    case ASPEED_PCIE_RC_INT_ENABLE:
    case ASPEED_PCIE_RC_INT_STATUS:
    case ASPEED_PCIE_RC_RX_DATA:
    case ASPEED_PCIE_RC_RX_DW0:
    case ASPEED_PCIE_RC_RX_DW1:
    case ASPEED_PCIE_RC_RX_DW2:
    case ASPEED_PCIE_RC_RX_DW3:
    case ASPEED_PCIE_RC_MSI_ENABLE0:
    case ASPEED_PCIE_RC_MSI_ENABLE1:
    case ASPEED_PCIE_RC_MSI_STATUS0:
    case ASPEED_PCIE_RC_MSI_STATUS1:
    case ASPEED_PCIE_RC_TX_TAG:
        val = s->regs[TO_REG(addr)];
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_aspeed_pcie_rc_read(addr, val);
    return val;
}

static void aspeed_pcie_rc_write(void *opaque, hwaddr addr, uint64_t data,
                                  unsigned int size)
{
    AspeedPCIERc *s = ASPEED_PCIE_RC(opaque);

    trace_aspeed_pcie_rc_write(addr, data);

    switch (addr) {
    case ASPEED_PCIE_RC_CTRL:  /* TODO: unlock/lock RX */
    case ASPEED_PCIE_RC_INT_ENABLE:
    case ASPEED_PCIE_RC_MSI_ENABLE0:
    case ASPEED_PCIE_RC_MSI_ENABLE1:
    case ASPEED_PCIE_RC_TX_TAG:
        s->regs[TO_REG(addr)] = data;
        break;
    case ASPEED_PCIE_RC_MSI_STATUS0:
    case ASPEED_PCIE_RC_MSI_STATUS1:
        s->regs[TO_REG(addr)] &= ~data;
        break;
    case ASPEED_PCIE_RC_INT_STATUS:  /* preserve INTx status*/
        s->regs[TO_REG(addr)] &= ~data | ASPEED_PCIE_RC_INT_STATUS_INTX_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps aspeed_pcie_rc_ops = {
    .read = aspeed_pcie_rc_read,
    .write = aspeed_pcie_rc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void aspeed_pcie_rc_set_irq(void *opaque, int irq, int level)
{
    AspeedPCIERc *s = (AspeedPCIERc *) opaque;

    assert(irq < PCI_NUM_PINS);

    if (level) {
        s->regs[TO_REG(ASPEED_PCIE_RC_INT_STATUS)] |= BIT(irq);
    } else {
        s->regs[TO_REG(ASPEED_PCIE_RC_INT_STATUS)] &= ~BIT(irq);
    }

    aspeed_pcie_rc_update_irq(s);
}

static int aspeed_pcie_rc_map_irq(PCIDevice *pci_dev, int irq_num)
{
    return irq_num % PCI_NUM_PINS;
}

static void aspeed_pcie_rc_realize(DeviceState *dev, Error **errp)
{
    PCIHostState *pci = PCI_HOST_BRIDGE(dev);
    AspeedPCIERc *s = ASPEED_PCIE_RC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    PCIExpressHost *pex = PCIE_HOST_BRIDGE(dev);

    memory_region_init_io(&s->reg_rc_mmio, OBJECT(s), &aspeed_pcie_rc_ops, s,
                          TYPE_ASPEED_PCIE_RC ".regs", 0x40);

    /* PCI configuration space */
    pcie_host_mmcfg_init(pex, PCIE_MMCFG_SIZE_MAX / 2);
    sysbus_init_mmio(sbd, &pex->mmio);

    /* MMIO and IO region */
    memory_region_init(&s->mmio, OBJECT(s), "mmio", UINT64_MAX);
    memory_region_init(&s->io, OBJECT(s), "io", 0x10000);

    memory_region_init_io(&s->mmio_window, OBJECT(s), &unassigned_io_ops,
                          OBJECT(s), "mmio_window", UINT64_MAX);
    memory_region_init_io(&s->io_window, OBJECT(s), &unassigned_io_ops,
                          OBJECT(s), "ioport_window", 64 * 1024);

    memory_region_add_subregion(&s->mmio_window, 0, &s->mmio);
    memory_region_add_subregion(&s->io_window, 0, &s->io);
    sysbus_init_mmio(sbd, &s->mmio_window);
    sysbus_init_mmio(sbd, &s->io_window);

    sysbus_init_irq(sbd, &s->irq);

    pci->bus = pci_register_root_bus(dev, dev->id, aspeed_pcie_rc_set_irq,
                                     aspeed_pcie_rc_map_irq, s, &s->mmio,
                                     &s->io, 0, 4, TYPE_PCIE_BUS);
    pci->bus->flags |= PCI_BUS_EXTENDED_CONFIG_SPACE;

    qdev_realize(DEVICE(&s->root), BUS(pci->bus), &error_fatal);
}

static void aspeed_pcie_rc_reset(DeviceState *dev)
{
    AspeedPCIERc *s = ASPEED_PCIE_RC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static const char *aspeed_pcie_rc_root_bus_path(PCIHostState *host_bridge,
                                                  PCIBus *rootbus)
{
    AspeedPCIERc *s = ASPEED_PCIE_RC(host_bridge);

    snprintf(s->name, sizeof(s->name), "0000:%02x", s->bus_nr);
    return s->name;
}

static void aspeed_pcie_rc_init(Object *obj)
{
    AspeedPCIERc *s = ASPEED_PCIE_RC(obj);
    AspeedPCIERoot *root = &s->root;

    object_initialize_child(obj, "root", root, TYPE_ASPEED_PCIE_ROOT);
    qdev_prop_set_int32(DEVICE(root), "addr", PCI_DEVFN(0, 0));
    qdev_prop_set_bit(DEVICE(root), "multifunction", false);
}

static Property aspeed_pcie_rc_props[] = {
    DEFINE_PROP_UINT32("bus-nr", AspeedPCIERc, bus_nr, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void aspeed_pcie_rc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

    hc->root_bus_path = aspeed_pcie_rc_root_bus_path;
    dc->reset = aspeed_pcie_rc_reset;
    dc->realize = aspeed_pcie_rc_realize;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->fw_name = "pci";
    device_class_set_props(dc, aspeed_pcie_rc_props);

    msi_nonbroken = true;
}

static const TypeInfo aspeed_pcie_rc_info = {
    .name       = TYPE_ASPEED_PCIE_RC,
    .parent     = TYPE_PCIE_HOST_BRIDGE,
    .instance_size = sizeof(AspeedPCIERc),
    .instance_init = aspeed_pcie_rc_init,
    .class_init = aspeed_pcie_rc_class_init,
};

static void aspeed_pcie_root_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->desc = "Aspeed PCIe Host Bridge";

    k->vendor_id = PCI_VENDOR_ID_ASPEED;
    k->device_id = PCI_DEVICE_ID_AST2600_RC;
    k->revision = 0;
    k->class_id = PCI_CLASS_BRIDGE_HOST;

    /* */
    dc->user_creatable = false;
}

static const TypeInfo aspeed_pcie_root_info = {
    .name = TYPE_ASPEED_PCIE_ROOT,
    .parent =  TYPE_PCI_DEVICE,
    .instance_size = sizeof(AspeedPCIERoot),
    .class_init = aspeed_pcie_root_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

/*
 * AHB to PCIe bridge (PCIECFG)
 */

#define ASPEED_PCIE_CFG_CTRL        0x00
#define   ASPEED_PCIE_CFG_CTRL_ENABLE      BIT(0)
#define   ASPEED_PCIE_CFG_CTRL_CLEAR_RX     BIT(4)
#define ASPEED_PCIE_CFG_EN_IRQ      0x04
#define ASPEED_PCIE_CFG_TX_CLEAR    0x08
#define ASPEED_PCIE_CFG_RX_DATA     0x0C
#define ASPEED_PCIE_CFG_TX_DW0      0x10
#define ASPEED_PCIE_CFG_TX_DW1      0x14
#define ASPEED_PCIE_CFG_TX_DW2      0x18
#define ASPEED_PCIE_CFG_TX_DW3      0x1C
#define ASPEED_PCIE_CFG_TX_DATA             0x20
#define ASPEED_PCIE_CFG_TX_STATUS           0x24
#define   ASPEED_PCIE_CFG_TX_STATUS_IDLE         BIT(31)
#define   ASPEED_PCIE_CFG_TX_STATUS_RC_H_RX_DONE BIT(27)
#define   ASPEED_PCIE_CFG_TX_STATUS_RC_L_RX_DONE BIT(26)
#define   ASPEED_PCIE_CFG_TX_STATUS_RC_H_TX_DONE BIT(25)
#define   ASPEED_PCIE_CFG_TX_STATUS_RC_L_TX_DONE BIT(24)
#define   ASPEED_PCIE_CFG_TX_STATUS_TRIG         BIT(0)
#define ASPEED_PCIE_CFG_MSI0        0x58
#define ASPEED_PCIE_CFG_MSI1        0x5C
#define ASPEED_PCIE_CFG_REG60       0x60
#define ASPEED_PCIE_CFG_REG64       0x64
#define ASPEED_PCIE_CFG_REG68       0x68

#define ASPEED_PCIE_CFG_RC_MAX_MSI 64

static uint64_t aspeed_pcie_cfg_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    AspeedPCIECfg *s = ASPEED_PCIE_CFG(opaque);
    uint64_t val = 0;

    switch (addr) {
    case ASPEED_PCIE_CFG_CTRL:
    case ASPEED_PCIE_CFG_EN_IRQ:
    case ASPEED_PCIE_CFG_TX_CLEAR:
    case ASPEED_PCIE_CFG_RX_DATA:
    case ASPEED_PCIE_CFG_TX_DW0:
    case ASPEED_PCIE_CFG_TX_DW1:
    case ASPEED_PCIE_CFG_TX_DW2:
    case ASPEED_PCIE_CFG_TX_DW3:
    case ASPEED_PCIE_CFG_TX_STATUS:
    case ASPEED_PCIE_CFG_REG60:
    case ASPEED_PCIE_CFG_REG64:
    case ASPEED_PCIE_CFG_REG68:
        val = s->regs[TO_REG(addr)];
        break;
    case ASPEED_PCIE_CFG_MSI0:
    case ASPEED_PCIE_CFG_MSI1:
        printf("%s: 0x%" HWADDR_PRIx "\n", __func__, addr);
        val = s->regs[TO_REG(addr)];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_aspeed_pcie_cfg_read(addr, val);
    return val;
}

#define TLP_FMTTYPE_CFGRD0      0x04 /* Configuration Read  Type 0 */
#define TLP_FMTTYPE_CFGWR0      0x44 /* Configuration Write Type 0 */
#define TLP_FMTTYPE_CFGRD1      0x05 /* Configuration Read  Type 1 */
#define TLP_FMTTYPE_CFGWR1      0x45 /* Configuration Write Type 1 */

#define PCIE_CFG_FMTTYPE_MASK(dw0) (((dw0) >> 24) & 0xff)
#define PCIE_CFG_BYTE_EN(dw1) ((dw1) & 0xf)

#define PCIE_MMCFG_ADDR(bus, devfn, offset)                             \
    ((((bus) & PCIE_MMCFG_BUS_MASK) << PCIE_MMCFG_BUS_BIT) |            \
     (((devfn) & PCIE_MMCFG_DEVFN_MASK) << PCIE_MMCFG_DEVFN_BIT) |      \
     ((offset) & PCIE_MMCFG_CONFOFFSET_MASK))

/* TODO : find a better way to deduce len/addr/val from byte enable */
static void aspeed_pcie_cfg_translate_write(AspeedPCIECfg *s, uint32_t *addr,
                                            uint64_t *val, int *len)
{
    uint8_t byte_en = PCIE_CFG_BYTE_EN(s->regs[TO_REG(ASPEED_PCIE_CFG_TX_DW1)]);

    *len = ctpop8(byte_en);

    switch (byte_en) {
    case 0x1:
    case 0x3:
    case 0xf:
        break;
    case 0x4:
        *addr += 2;
        *val = (*val >> (2 * 8)) & MAKE_64BIT_MASK(0, *len * 8);
        break;
    case 0x2:
    case 0xc:
        *addr += *len;
        *val = (*val >> (*len * 8)) & MAKE_64BIT_MASK(0, *len * 8);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid byte enable: %d\n",
                      __func__, byte_en);
        g_assert_not_reached();
    }
}

static void aspeed_pcie_cfg_readwrite(AspeedPCIECfg *s)
{
    bool is_write = !!(s->regs[TO_REG(ASPEED_PCIE_CFG_TX_DW0)] & (1ul << 30));
    uint32_t cfg_addr = s->regs[TO_REG(ASPEED_PCIE_CFG_TX_DW2)];
    uint8_t bus = (cfg_addr >> 24) & 0xff;
    uint8_t devfn  = (cfg_addr >> 16) & 0xff;
    uint32_t offset = cfg_addr & 0xffc;
    uint8_t rc_index = !!(bus & 0x80);
    AspeedPCIERc *rc = &s->rcs[rc_index];

    uint64_t val = ~0;
    int len;

    PCIHostState *pci = PCI_HOST_BRIDGE(rc);
    PCIDevice *pdev;

    /* HACK: rework host bridge */
    if (bus == 0x80) {
        bus = 0;
    }

    pdev = pci_find_device(pci->bus, bus, devfn);
    if (!pdev) {
        rc->regs[TO_REG(ASPEED_PCIE_RC_RX_DATA)] = ~0;
        goto out;
    }

    switch (PCIE_CFG_FMTTYPE_MASK(s->regs[TO_REG(ASPEED_PCIE_CFG_TX_DW0)])) {
    case TLP_FMTTYPE_CFGWR0:
    case TLP_FMTTYPE_CFGWR1:
        val = s->regs[TO_REG(ASPEED_PCIE_CFG_TX_DATA)];
        aspeed_pcie_cfg_translate_write(s, &offset, &val, &len);

        pci_host_config_write_common(pdev, offset, pci_config_size(pdev),
                                     val, len);
        break;

    case TLP_FMTTYPE_CFGRD0:
    case TLP_FMTTYPE_CFGRD1:
        val = pci_host_config_read_common(pdev, offset,
                                          pci_config_size(pdev), 4);
        rc->regs[TO_REG(ASPEED_PCIE_RC_RX_DATA)] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid CFG type. DW0=0x%x\n",
                      __func__, s->regs[TO_REG(ASPEED_PCIE_CFG_TX_DW0)]);
    }

out:
    rc->regs[TO_REG(ASPEED_PCIE_RC_INT_STATUS)] |=
        ASPEED_PCIE_RC_INT_STATUS_RX;

    s->regs[TO_REG(ASPEED_PCIE_CFG_TX_STATUS)] |= BIT(24 + rc_index);
    s->regs[TO_REG(ASPEED_PCIE_CFG_TX_STATUS)] |=
        ASPEED_PCIE_CFG_TX_STATUS_IDLE;

    trace_aspeed_pcie_cfg_rw(is_write ?  "write" : "read", bus, devfn,
                             cfg_addr, val);
}

static void aspeed_pcie_cfg_msi_notify(AspeedPCIECfg *s, hwaddr addr,
                                       uint64_t data)
{
    bool rc_index = !!(addr == ASPEED_PCIE_CFG_MSI1);
    AspeedPCIERc *rc = &s->rcs[rc_index];
    int reg;

    trace_aspeed_pcie_cfg_msi_notify(addr, data);

    /* Written data is the HW IRQ number */
    if (data >= ASPEED_PCIE_CFG_RC_MAX_MSI) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid MSI vector %"PRIx64"\n",
                      __func__, data);
        return;
    }

    /* TODO: what is ASPEED_PCIE_RC_CTRL_RX_MSI_SELECT ? */
    if (!(rc->regs[TO_REG(ASPEED_PCIE_RC_CTRL)] &
          ASPEED_PCIE_RC_CTRL_RX_MSI)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: MSI are not enabled\n", __func__);
        return;
    }

    reg = data < 32 ? ASPEED_PCIE_RC_MSI_STATUS0 : ASPEED_PCIE_RC_MSI_STATUS1;

    rc->regs[TO_REG(reg)] |= BIT(data % 32);
    aspeed_pcie_rc_update_irq(rc);
}

static void aspeed_pcie_cfg_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned int size)
{
    AspeedPCIECfg *s = ASPEED_PCIE_CFG(opaque);

    trace_aspeed_pcie_cfg_write(addr, data);

    /* TODO: test ASPEED_PCIE_CFG_CTRL_ENABLE */

    switch (addr) {
    case ASPEED_PCIE_CFG_EN_IRQ:
    case ASPEED_PCIE_CFG_TX_DW0:
    case ASPEED_PCIE_CFG_TX_DW1:
    case ASPEED_PCIE_CFG_TX_DW2:
    case ASPEED_PCIE_CFG_TX_DW3:
    case ASPEED_PCIE_CFG_TX_DATA:
    case ASPEED_PCIE_CFG_REG60:
    case ASPEED_PCIE_CFG_REG64:
    case ASPEED_PCIE_CFG_REG68:
        s->regs[TO_REG(addr)] = data;
        break;
    case ASPEED_PCIE_CFG_MSI0:
    case ASPEED_PCIE_CFG_MSI1:
        aspeed_pcie_cfg_msi_notify(s, addr, data);
        break;
    case ASPEED_PCIE_CFG_CTRL:
        if (data & ASPEED_PCIE_CFG_CTRL_CLEAR_RX) {
            s->regs[TO_REG(ASPEED_PCIE_CFG_RX_DATA)] = ~0;
        }
        break;

    case ASPEED_PCIE_CFG_TX_CLEAR:
        if (data == 0x1) {
            s->regs[TO_REG(ASPEED_PCIE_CFG_TX_STATUS)] &=
                ~ASPEED_PCIE_CFG_TX_STATUS_IDLE;
        }
        break;
    case ASPEED_PCIE_CFG_TX_STATUS:
        if (data & ASPEED_PCIE_CFG_TX_STATUS_TRIG) {
            aspeed_pcie_cfg_readwrite(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps aspeed_pcie_cfg_ops = {
    .read = aspeed_pcie_cfg_read,
    .write = aspeed_pcie_cfg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void aspeed_pcie_cfg_reset(DeviceState *dev)
{
    AspeedPCIECfg *s = ASPEED_PCIE_CFG(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void aspeed_pcie_cfg_instance_init(Object *obj)
{
    AspeedPCIECfg *s = ASPEED_PCIE_CFG(obj);
    int i;

    for (i = 0; i < ARRAY_SIZE(s->rcs); i++) {
        object_initialize_child(obj, "rcs[*]", &s->rcs[i],
                                TYPE_ASPEED_PCIE_RC);
    }
}

static void aspeed_pcie_cfg_realize(DeviceState *dev, Error **errp)
{
    AspeedPCIECfg *s = ASPEED_PCIE_CFG(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int nr_rcs = ARRAY_SIZE(s->rcs);
    int i;

    memory_region_init(&s->reg_mmio_container, OBJECT(s),
                       TYPE_ASPEED_PCIE_CFG ".container", 0x100);
    sysbus_init_mmio(sbd, &s->reg_mmio_container);

    memory_region_init_io(&s->reg_cfg_mmio, OBJECT(s), &aspeed_pcie_cfg_ops, s,
                          TYPE_ASPEED_PCIE_CFG ".regs", 0x80);
    memory_region_add_subregion(&s->reg_mmio_container, 0x0, &s->reg_cfg_mmio);

    for (i = 0; i < nr_rcs; i++) {
        object_property_set_int(OBJECT(&s->rcs[i]), "bus-nr",
                                i * PCI_BUS_MAX / nr_rcs, &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcs[i]), errp)) {
            return;
        }

        /* RC registers */
        memory_region_add_subregion(&s->reg_mmio_container, 0x80 + i * 0x40,
                                    &s->rcs[i].reg_rc_mmio);
    }
}

static Property aspeed_pcie_cfg_props[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void aspeed_pcie_cfg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_pcie_cfg_realize;
    dc->reset = aspeed_pcie_cfg_reset;
    device_class_set_props(dc, aspeed_pcie_cfg_props);
}

static const TypeInfo aspeed_pcie_cfg_info = {
    .name       = TYPE_ASPEED_PCIE_CFG,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedPCIECfg),
    .instance_init = aspeed_pcie_cfg_instance_init,
    .class_init = aspeed_pcie_cfg_class_init,
};


/*
 * PHY
 */

#define ASPEED_PCIE_PHY_DEVID           0x00
#define ASPEED_PCIE_PHY_CLASS_CODE      0x04
#define   ASPEED_PCIE_PHY_CLASS_CODE_A3   0x6
#define ASPEED_PCIE_PHY_DATALINK        0x10
#define ASPEED_PCIE_PHY_HOTPLUG         0x14
#define ASPEED_PCIE_PHY_CTRL1           0x30
#define   ASPEED_PCIE_PHY_CTRL1_ROOTPORT  (BIT(5) | BIT(4))
#define   ASPEED_PCIE_PHY_CTRL1_ENABLE     BIT(1)
#define ASPEED_PCIE_PHY_PROTECT         0x7C
#define ASPEED_PCIE_PHY_LINK            0xC0
#define   ASPEED_PCIE_PHY_LINK_STATUS     BIT(5)
#define ASPEED_PCIE_PHY_BDF             0xC4
#define ASPEED_PCIE_PHY_LINK_STS        0xD0

static uint64_t aspeed_pcie_phy_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    AspeedPCIEPhy *s = ASPEED_PCIE_PHY(opaque);
    uint64_t val = 0;

    switch (addr) {
    case ASPEED_PCIE_PHY_DEVID:
    case ASPEED_PCIE_PHY_CLASS_CODE:
    case ASPEED_PCIE_PHY_DATALINK:
    case ASPEED_PCIE_PHY_CTRL1:
    case ASPEED_PCIE_PHY_PROTECT:
    case ASPEED_PCIE_PHY_LINK:
    case ASPEED_PCIE_PHY_BDF:
    case ASPEED_PCIE_PHY_LINK_STS:
        val = s->regs[TO_REG(addr)];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }

    trace_aspeed_pcie_phy_read(addr, val);
    return val;
}

static void aspeed_pcie_phy_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned int size)
{
    AspeedPCIEPhy *s = ASPEED_PCIE_PHY(opaque);

    trace_aspeed_pcie_phy_write(addr, data);

    /* TODO: test protect */

    switch (addr) {
    case ASPEED_PCIE_PHY_PROTECT:
        data &= 0xff;
        s->regs[TO_REG(addr)] = !!(data == 0xA8);
        break;

    case ASPEED_PCIE_PHY_DEVID:
    case ASPEED_PCIE_PHY_CLASS_CODE:
    case ASPEED_PCIE_PHY_DATALINK:
    case ASPEED_PCIE_PHY_CTRL1: /* 0x30 == root port, else bridge */
        s->regs[TO_REG(addr)] = data;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps aspeed_pcie_phy_ops = {
    .read = aspeed_pcie_phy_read,
    .write = aspeed_pcie_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void aspeed_pcie_phy_reset(DeviceState *dev)
{
    AspeedPCIEPhy *s = ASPEED_PCIE_PHY(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[TO_REG(ASPEED_PCIE_PHY_DEVID)] =
        (PCI_DEVICE_ID_AST2600_RC << 16) | PCI_VENDOR_ID_ASPEED;
    s->regs[TO_REG(ASPEED_PCIE_PHY_CLASS_CODE)] =
        0x06040000 | ASPEED_PCIE_PHY_CLASS_CODE_A3;
    s->regs[TO_REG(ASPEED_PCIE_PHY_DATALINK)] = 0xD7040022;
    s->regs[TO_REG(ASPEED_PCIE_PHY_LINK)] = ASPEED_PCIE_PHY_LINK_STATUS;
    s->regs[TO_REG(ASPEED_PCIE_PHY_BDF)] = 0x0; /* TODO: BUS+DEV */
}

static void aspeed_pcie_phy_realize(DeviceState *dev, Error **errp)
{
    AspeedPCIEPhy *s = ASPEED_PCIE_PHY(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &aspeed_pcie_phy_ops, s,
                          TYPE_ASPEED_PCIE_PHY, 0x200);
    sysbus_init_mmio(sbd, &s->mmio);
}

static Property aspeed_pcie_phy_props[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void aspeed_pcie_phy_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_pcie_phy_realize;
    dc->reset = aspeed_pcie_phy_reset;
    device_class_set_props(dc, aspeed_pcie_phy_props);
}

static const TypeInfo aspeed_pcie_phy_info = {
    .name       = TYPE_ASPEED_PCIE_PHY,
    .parent     = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedPCIEPhy),
    .class_init = aspeed_pcie_phy_class_init,
};

static void aspeed_pcie_register(void)
{
    type_register_static(&aspeed_pcie_root_info);
    type_register_static(&aspeed_pcie_rc_info);
    type_register_static(&aspeed_pcie_phy_info);
    type_register_static(&aspeed_pcie_cfg_info);
}

type_init(aspeed_pcie_register)
