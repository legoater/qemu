/*
 * QEMU PowerPC PowerNV Emulation of a few OCC related registers
 *
 * Copyright (c) 2015-2017, IBM Corporation.
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
 */

#include "qemu/osdep.h"
#include "target/ppc/cpu.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "include/sysemu/reset.h"
#include "hw/qdev-properties.h"
#include "hw/ppc/pnv.h"
#include "hw/ppc/pnv_xscom.h"
#include "hw/ppc/pnv_occ.h"

#define OCB_OCI_OCCMISC         0x4020
#define OCB_OCI_OCCMISC_AND     0x4021
#define OCB_OCI_OCCMISC_OR      0x4022

static void pnv_occ_set_misc(PnvOCC *occ, uint64_t val)
{
    bool irq_state;
    PnvOCCClass *poc = PNV_OCC_GET_CLASS(occ);

    val &= 0xffff000000000000ull;

    occ->occmisc = val;
    irq_state = !!(val >> 63);
    pnv_psi_irq_set(occ->psi, poc->psi_irq, irq_state);
}

static uint64_t pnv_occ_power8_xscom_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    PnvOCC *occ = PNV_OCC(opaque);
    uint32_t offset = addr >> 3;
    uint64_t val = 0;

    switch (offset) {
    case OCB_OCI_OCCMISC:
        val = occ->occmisc;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "OCC Unimplemented register: Ox%"
                      HWADDR_PRIx "\n", addr >> 3);
    }
    return val;
}

static void pnv_occ_power8_xscom_write(void *opaque, hwaddr addr,
                                       uint64_t val, unsigned size)
{
    PnvOCC *occ = PNV_OCC(opaque);
    uint32_t offset = addr >> 3;

    switch (offset) {
    case OCB_OCI_OCCMISC_AND:
        pnv_occ_set_misc(occ, occ->occmisc & val);
        break;
    case OCB_OCI_OCCMISC_OR:
        pnv_occ_set_misc(occ, occ->occmisc | val);
        break;
    case OCB_OCI_OCCMISC:
        pnv_occ_set_misc(occ, val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "OCC Unimplemented register: Ox%"
                      HWADDR_PRIx "\n", addr >> 3);
    }
}

#undef OCC_SENSOR_DEBUG

static uint64_t pnv_occ_common_area_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    uint64_t ret = 0;
    PnvOCC *occ = PNV_OCC(opaque);
    int i;

    for (i = 0; i < size; i++) {
        ret |= (uint64_t) occ->sensor_data[addr + i]  << (8 * i);
    }

#ifdef OCC_SENSOR_DEBUG
    printf("%s: @%"HWADDR_PRIx" sz=%d -> 0x%016"PRIx64"\n", __func__,
           addr, size, ret);
#endif
    return ret;
}

static void pnv_occ_common_area_write(void *opaque, hwaddr addr,
                                             uint64_t val, unsigned width)
{
    qemu_log_mask(LOG_UNIMP, "OCC: write to unimplemented address: Ox%"
                  HWADDR_PRIx "\n", addr);
}

static const MemoryRegionOps pnv_occ_power8_xscom_ops = {
    .read = pnv_occ_power8_xscom_read,
    .write = pnv_occ_power8_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

const MemoryRegionOps pnv_occ_sram_ops = {
    .read = pnv_occ_common_area_read,
    .write = pnv_occ_common_area_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void pnv_occ_power8_class_init(ObjectClass *klass, void *data)
{
    PnvOCCClass *poc = PNV_OCC_CLASS(klass);

    poc->xscom_size = PNV_XSCOM_OCC_SIZE;
    poc->xscom_ops = &pnv_occ_power8_xscom_ops;
    poc->psi_irq = PSIHB_IRQ_OCC;
}

static const TypeInfo pnv_occ_power8_type_info = {
    .name          = TYPE_PNV8_OCC,
    .parent        = TYPE_PNV_OCC,
    .instance_size = sizeof(PnvOCC),
    .class_init    = pnv_occ_power8_class_init,
};

#define P9_OCB_OCI_OCCMISC              0x6080
#define P9_OCB_OCI_OCCMISC_CLEAR        0x6081
#define P9_OCB_OCI_OCCMISC_OR           0x6082

#define OCCFLG_BASE                     0x608A
#define OCCFLG_CLEAR                    0x608B
#define OCCFLG_SET                      0x608C

static uint64_t pnv_occ_power9_xscom_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    PnvOCC *occ = PNV_OCC(opaque);
    uint32_t offset = addr >> 3;
    uint64_t val = 0;

    switch (offset) {
    case P9_OCB_OCI_OCCMISC:
        val = occ->occmisc;
        break;
    case OCCFLG_BASE:
        val = occ->occbase;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "OCC Unimplemented register: Ox%"
                      HWADDR_PRIx "\n", addr >> 3);
    }
    return val;
}

static void pnv_occ_power9_xscom_write(void *opaque, hwaddr addr,
                                       uint64_t val, unsigned size)
{
    PnvOCC *occ = PNV_OCC(opaque);
    uint32_t offset = addr >> 3;

    switch (offset) {
    case P9_OCB_OCI_OCCMISC_CLEAR:
        pnv_occ_set_misc(occ, 0);
        break;
    case P9_OCB_OCI_OCCMISC_OR:
        pnv_occ_set_misc(occ, occ->occmisc | val);
        break;
    case P9_OCB_OCI_OCCMISC:
        pnv_occ_set_misc(occ, val);
       break;
    case OCCFLG_SET:
        occ->occbase |= val;
        break;
    case OCCFLG_CLEAR:
        occ->occbase &= ~val;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "OCC Unimplemented register: Ox%"
                      HWADDR_PRIx "\n", addr >> 3);
    }
}

static const MemoryRegionOps pnv_occ_power9_xscom_ops = {
    .read = pnv_occ_power9_xscom_read,
    .write = pnv_occ_power9_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void pnv_occ_power9_class_init(ObjectClass *klass, void *data)
{
    PnvOCCClass *poc = PNV_OCC_CLASS(klass);

    poc->xscom_size = PNV9_XSCOM_OCC_SIZE;
    poc->xscom_ops = &pnv_occ_power9_xscom_ops;
    poc->psi_irq = PSIHB9_IRQ_OCC;
}

static const TypeInfo pnv_occ_power9_type_info = {
    .name          = TYPE_PNV9_OCC,
    .parent        = TYPE_PNV_OCC,
    .instance_size = sizeof(PnvOCC),
    .class_init    = pnv_occ_power9_class_init,
};

/* OCC sensors */
struct occ_sensor_record {
        uint16_t gsid;
        uint64_t timestamp;
        uint16_t sample;
        uint16_t sample_min;
        uint16_t sample_max;
        uint16_t csm_min;
        uint16_t csm_max;
        uint16_t profiler_min;
        uint16_t profiler_max;
        uint16_t job_scheduler_min;
        uint16_t job_scheduler_max;
        uint64_t accumulator;
        uint32_t update_tag;
        uint8_t  pad[8];
} __attribute__((__packed__));

static struct occ_sensor_record sensor_records[] = {
    {
        .gsid = 1,
        .sample = 38,
        .csm_max = 48,
        .csm_min = 28,
    },
};

#define MAX_CHARS_SENSOR_NAME 16
#define MAX_CHARS_SENSOR_UNIT 4

struct occ_sensor_name {
        char     name[MAX_CHARS_SENSOR_NAME];
        char     units[MAX_CHARS_SENSOR_UNIT];
        uint16_t gsid;
        uint32_t freq;
        uint32_t scale_factor;
        uint16_t type;
        uint16_t location;
        uint8_t  structure_type;
        uint32_t reading_offset;
        uint8_t  sensor_data;
        uint8_t  pad[8];
} __attribute__((__packed__));

static struct occ_sensor_name sensor_names[] = {
    {
        .name           = "TEMPPROCTHRMC01",
        .units          = "",
        .gsid           = 0,
        .freq           = 0,
        .scale_factor   = 0,
        .type           = 0x8,  /* OCC_SENSOR_TYPE_TEMPERATURE */
        .location       = 0x40, /* OCC_SENSOR_LOC_CORE */
        .structure_type = 1,    /* OCC_SENSOR_READING_FULL */
        .reading_offset = 0,
        .sensor_data    = 0,
    },
};

struct occ_sensor_data_header {
        uint8_t  valid;
        uint8_t  version;
        uint16_t nr_sensors;
        uint8_t  reading_version;
        uint8_t  pad[3];
        uint32_t names_offset;
        uint8_t  names_version;
        uint8_t  name_length;
        uint16_t reserved;
        uint32_t reading_ping_offset;
        uint32_t reading_pong_offset;
} __attribute__((__packed__));

static struct occ_sensor_data_header header = {
    .valid               = 1,
    .version             = 1,
    .nr_sensors          = ARRAY_SIZE(sensor_names),
    .reading_version     = 1,
    .names_offset        = 0x400,
    .names_version       = 1,
    .name_length         = sizeof(struct occ_sensor_name),
    .reading_ping_offset = 0xDC00,
    .reading_pong_offset = 0x18C00,
};

/*
 * OCC N Sensor Data Block Layout (150kB)
 *
 * The sensor data block layout is the same for each OCC N. It contains
 * sensor-header-block, sensor-names buffer, sensor-readings-ping buffer and
 * sensor-readings-pong buffer.
 *
 * ----------------------------------------------------------------------------
 * | Start (Offset from OCC |   End        | Size |Description                |
 * | N Sensor Data Block)   |              |      |                           |
 * ----------------------------------------------------------------------------
 * |    0x00000000          |  0x000003FF  |1kB   |Sensor Data Header Block   |
 * |    0x00000400          |  0x0000CBFF  |50kB  |Sensor Names               |
 * |    0x0000CC00          |  0x0000DBFF  |4kB   |Reserved                   |
 * |    0x0000DC00          |  0x00017BFF  |40kB  |Sensor Readings ping buffer|
 * |    0x00017C00          |  0x00018BFF  |4kB   |Reserved                   |
 * |    0x00018C00          |  0x00022BFF  |40kB  |Sensor Readings pong buffer|
 * |    0x00022C00          |  0x000257FF  |11kB  |Reserved                   |
 * ----------------------------------------------------------------------------
 */
static void pnv_occ_reset(void *dev)
{
    PnvOCC *occ = PNV_OCC(dev);

    memcpy(&occ->sensor_data[0], &header, sizeof(header));
    memcpy(&occ->sensor_data[0x400], &sensor_names, sizeof(sensor_names));
    memcpy(&occ->sensor_data[0xDC00], &sensor_records, sizeof(sensor_records));
    memcpy(&occ->sensor_data[0x18C00], &sensor_records, sizeof(sensor_records));
}

static struct occ_sensor_record *pnv_occ_get_record(PnvOCC *occ,
                                                    const char *name)
{
    struct occ_sensor_data_header *h = (struct occ_sensor_data_header *)
        &occ->sensor_data[0];
    struct occ_sensor_name *names = (struct occ_sensor_name *)
        &occ->sensor_data[0x400];
    struct occ_sensor_record *record = NULL;
    int i;

    for (i = 0; i < h->nr_sensors; i++) {
        uint32_t offset;
        if (strcmp(names[i].name, name)) {
            continue;
        }

        offset = h->reading_ping_offset + names[i].reading_offset;
        record = (struct occ_sensor_record *) &occ->sensor_data[offset];
    }

    return record;
}

static void pnv_occ_get_sensor(Object *obj, Visitor *v, const char *name,
                               void *opaque, Error **errp)
{
    PnvOCC *occ = PNV_OCC(obj);
    struct occ_sensor_record *record;
    uint16_t value;

    record = pnv_occ_get_record(occ, name);
    if (!record) {
        error_setg(errp, "%s: error reading %s", __func__, name);
        return;
    }

    value = record->sample;
    visit_type_uint16(v, name, &value, errp);
}

static void pnv_occ_set_sensor(Object *obj, Visitor *v, const char *name,
                               void *opaque, Error **errp)
{
    PnvOCC *occ = PNV_OCC(obj);
    struct occ_sensor_record *record;
    uint16_t value;
    Error *local_err = NULL;

    visit_type_uint16(v, name, &value, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    record = pnv_occ_get_record(occ, name);
    if (!record) {
        error_setg(errp, "%s: error reading %s", __func__, name);
        return;
    }

    record->sample = value;
}

static void pnv_occ_instance_init(Object *obj)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(sensor_names); i++) {
        object_property_add(obj, sensor_names[i].name, "uint16",
                            pnv_occ_get_sensor,
                            pnv_occ_set_sensor, NULL, NULL);
    }
}

static void pnv_occ_realize(DeviceState *dev, Error **errp)
{
    PnvOCC *occ = PNV_OCC(dev);
    PnvOCCClass *poc = PNV_OCC_GET_CLASS(occ);

    assert(occ->psi);

    occ->occmisc = 0;

    /* XScom region for OCC registers */
    pnv_xscom_region_init(&occ->xscom_regs, OBJECT(dev), poc->xscom_ops,
                          occ, "xscom-occ", poc->xscom_size);

    /* OCC common area mmio region for OCC SRAM registers */
    memory_region_init_io(&occ->sram_regs, OBJECT(dev), &pnv_occ_sram_ops,
                          occ, "occ-common-area",
                          PNV_OCC_SENSOR_DATA_BLOCK_SIZE);

    qemu_register_reset(pnv_occ_reset, dev);
}

static Property pnv_occ_properties[] = {
    DEFINE_PROP_LINK("psi", PnvOCC, psi, TYPE_PNV_PSI, PnvPsi *),
    DEFINE_PROP_END_OF_LIST(),
};

static void pnv_occ_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pnv_occ_realize;
    dc->desc = "PowerNV OCC Controller";
    device_class_set_props(dc, pnv_occ_properties);
    dc->user_creatable = false;
}

static const TypeInfo pnv_occ_type_info = {
    .name          = TYPE_PNV_OCC,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(PnvOCC),
    .instance_init = pnv_occ_instance_init,
    .class_init    = pnv_occ_class_init,
    .class_size    = sizeof(PnvOCCClass),
    .abstract      = true,
};

static void pnv_occ_register_types(void)
{
    type_register_static(&pnv_occ_type_info);
    type_register_static(&pnv_occ_power8_type_info);
    type_register_static(&pnv_occ_power9_type_info);
}

type_init(pnv_occ_register_types);
