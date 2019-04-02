/*
 * IBM CFF power supplies device
 *
 * Copyright (c) 2019 IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/misc/pmbus_regs.h"
#include "migration/vmstate.h"

#define CFFPS_CCIN                              0xBD
#define CFFPS_FW_CMD_START                      0xFA
#define CFFPS_FW_NUM_BYTES                      4
#define CFFPS_SYS_CONFIG                        0xDA
#define CFFPS_INPUT_HISTORY                     0xD6
#define CFFPS_INPUT_HISTORY_SIZE                100

#define FW_VERSION      0x04030201
#define CCIN            0x2B1D

typedef struct IBMCffpsState {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t regs[0x100];

    uint8_t len;
    uint8_t buf[32];
    uint8_t pointer;

} IBMCffpsState;

#define TYPE_IBM_CFFPS "ibm-cffps"
#define IBM_CFFPS(obj) OBJECT_CHECK(IBMCffpsState, (obj), TYPE_IBM_CFFPS)

static void ibm_cffps_read(IBMCffpsState *s)
{
    s->len = 0;

    switch (s->pointer) {
    case PMBUS_MFR_MODEL: /* FRU */
        s->buf[s->len++] = 7;  /* Byte count */
        s->buf[s->len++] = 'F';
        s->buf[s->len++] = 'R';
        s->buf[s->len++] = 'U';
        s->buf[s->len++] = '0';
        s->buf[s->len++] = '1';
        s->buf[s->len++] = '2';
        s->buf[s->len++] = '3';
        break;
    case PMBUS_MFR_REVISION: /* PART NUMBER */
        s->buf[s->len++] = 7;   /* Byte count */
        s->buf[s->len++] = 'P';
        s->buf[s->len++] = 'A';
        s->buf[s->len++] = 'R';
        s->buf[s->len++] = '0';
        s->buf[s->len++] = '1';
        s->buf[s->len++] = '2';
        s->buf[s->len++] = '3';
        break;
    case PMBUS_MFR_SERIAL:
        s->buf[s->len++] = 6;   /* Byte count */
        s->buf[s->len++] = 'X';
        s->buf[s->len++] = 'Y';
        s->buf[s->len++] = 'Z';
        s->buf[s->len++] = '0';
        s->buf[s->len++] = '1';
        s->buf[s->len++] = '2';
        break;
    case CFFPS_CCIN:
        s->buf[s->len++] = (CCIN >> 8) & 0xFF;
        s->buf[s->len++] = CCIN & 0xFF;
        break;
    case CFFPS_FW_CMD_START ... CFFPS_FW_CMD_START + CFFPS_FW_NUM_BYTES - 1:
        s->buf[s->len++] = (FW_VERSION >> (CFFPS_FW_NUM_BYTES -
                           (s->pointer - CFFPS_FW_CMD_START) - 1) * 8) & 0xFF;
        break;
    case CFFPS_INPUT_HISTORY: /* TODO */
        s->buf[s->len++] = 0x0;
        break;
    default:
        s->buf[s->len++] = s->regs[s->pointer];
        break;
    }
}

static void ibm_cffps_write(IBMCffpsState *s)
{
    switch (s->pointer) {
    case CFFPS_SYS_CONFIG:
        s->regs[s->pointer] = s->buf[0];
        break;
    }
}

static uint8_t ibm_cffps_recv(I2CSlave *i2c)
{
    IBMCffpsState *s = IBM_CFFPS(i2c);

    if (s->len < sizeof(s->buf)) {
        return s->buf[s->len++];
    } else {
        return 0xff;
    }
}

static int ibm_cffps_send(I2CSlave *i2c, uint8_t data)
{
    IBMCffpsState *s = IBM_CFFPS(i2c);

    if (s->len == 0) {
        /*
         * first byte is the register pointer for a read or write
         * operation
         */
        s->pointer = data;
        s->len++;
    } else {
        /*
         * next bytes are data to write.
         */
        if (s->len <= sizeof(s->buf)) {
            s->buf[s->len - 1] = data;
        }
        s->len++;
        ibm_cffps_write(s);
    }

    return 0;
}

static int ibm_cffps_event(I2CSlave *i2c, enum i2c_event event)
{
    IBMCffpsState *s = IBM_CFFPS(i2c);

    /* TODO: handle SMBus "block read" */

    switch (event) {
    case I2C_START_RECV:
        ibm_cffps_read(s);
        break;
    case I2C_START_SEND:
    case I2C_NACK:
    case I2C_FINISH:
         s->pointer = 0xFF;
        break;
    default:
        return -1;
    }

    s->len = 0;
    return 0;
}

static const VMStateDescription vmstate_ibm_cffps = {
    .name = TYPE_IBM_CFFPS,
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, IBMCffpsState),
        VMSTATE_UINT8_ARRAY(buf, IBMCffpsState, 32),
        VMSTATE_UINT8(pointer, IBMCffpsState),
        VMSTATE_UINT8_ARRAY(regs, IBMCffpsState, 0x100),
        VMSTATE_I2C_SLAVE(i2c, IBMCffpsState),
        VMSTATE_END_OF_LIST()
    }
};

static void ibm_cffps_reset(DeviceState *dev)
{
    IBMCffpsState *s = IBM_CFFPS(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0xFF;
}

static void ibm_cffps_realize(DeviceState *dev, Error **errp)
{
    ;
}

static void ibm_cffps_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = ibm_cffps_realize;
    device_class_set_legacy_reset(dc, ibm_cffps_reset);
    k->event = ibm_cffps_event;
    k->recv = ibm_cffps_recv;
    k->send = ibm_cffps_send;
    dc->vmsd = &vmstate_ibm_cffps;
}

static const TypeInfo ibm_cffps_info = {
    .name          = TYPE_IBM_CFFPS,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IBMCffpsState),
    .class_init    = ibm_cffps_class_init,
};

static void ibm_cffps_register_types(void)
{
    type_register_static(&ibm_cffps_info);
}

type_init(ibm_cffps_register_types)
