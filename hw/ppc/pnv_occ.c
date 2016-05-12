/*
 * Emulation of a few OCC related registers
 *
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
#include "hw/hw.h"
#include "hw/ppc/pnv.h"
#include "hw/ppc/pnv_xscom.h"

struct PnvOCCState {
    XScomDevice xd;
    PnvPsiController *psi;

    /* OCC Misc interrupt */
    uint64_t occmisc;
};

#define TYPE_PNV_OCC "pnv-occ"
#define PNV_OCC(obj) OBJECT_CHECK(PnvOCCState, (obj), TYPE_PNV_OCC)

static void pnv_occ_set_misc(PnvOCCState *occ, uint64_t val)
{
    bool irq_state;

    val &= 0xffff000000000000ull;

    occ->occmisc = val;
    irq_state = !!(val >> 63);
    pnv_psi_irq_set(occ->psi, PSIHB_IRQ_OCC, irq_state);
}

static bool pnv_occ_xscom_read(XScomDevice *dev, uint32_t range,
                               uint32_t offset, uint64_t *out_val)
{
    PnvOCCState *occ = PNV_OCC(dev);
    uint32_t pcb_addr = dev->ranges[range].addr + offset;

    switch(pcb_addr) {
    case 0x6a020:
        *out_val = occ->occmisc;
        return true;
    }
    return false;
}

static bool pnv_occ_xscom_write(XScomDevice *dev, uint32_t range,
                                uint32_t offset, uint64_t val)
{
    PnvOCCState *occ = PNV_OCC(dev);
    uint32_t pcb_addr = dev->ranges[range].addr + offset;

    switch(pcb_addr) {
    default:
    case 0x6a020:
        pnv_occ_set_misc(occ, val);
        return true;
    case 0x6a021:
        pnv_occ_set_misc(occ, occ->occmisc & val);
        return true;
    case 0x6a022:
        pnv_occ_set_misc(occ, occ->occmisc | val);
        return true;
   }
    return false;
}

static void pnv_occ_realize(DeviceState *dev, Error **errp)
{
    PnvOCCState *occ = PNV_OCC(dev);
    XScomDevice *xd = XSCOM_DEVICE(dev);

    xd->ranges[0].addr = 0x66000;
    xd->ranges[0].size = 0x6000;

    occ->occmisc = 0;
}

static void pnv_occ_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XScomDeviceClass *k = XSCOM_DEVICE_CLASS(klass);

    k->read = pnv_occ_xscom_read;
    k->write = pnv_occ_xscom_write;

    dc->realize = pnv_occ_realize;
}

static const TypeInfo pnv_occ_type_info = {
    .name          = TYPE_PNV_OCC,
    .parent        = TYPE_XSCOM_DEVICE,
    .instance_size = sizeof(PnvOCCState),
    .class_init    = pnv_occ_class_init,
};

static void pnv_occ_register_types(void)
{
    type_register_static(&pnv_occ_type_info);
}

type_init(pnv_occ_register_types)

void pnv_occ_create(PnvChip *chip)
{
    struct DeviceState *dev;
    PnvOCCState *occ;

    dev = qdev_create(&chip->xscom->bus, TYPE_PNV_OCC);
    occ = PNV_OCC(dev);
    occ->psi = chip->psi;
    qdev_init_nofail(dev);
    chip->occ = occ;
}
