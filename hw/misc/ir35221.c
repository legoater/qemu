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

#define IR35221_MFR_VIN_PEAK		0xc5
#define IR35221_MFR_VOUT_PEAK		0xc6
#define IR35221_MFR_IOUT_PEAK		0xc7
#define IR35221_MFR_TEMP_PEAK		0xc8
#define IR35221_MFR_VIN_VALLEY		0xc9
#define IR35221_MFR_VOUT_VALLEY		0xca
#define IR35221_MFR_IOUT_VALLEY		0xcb
#define IR35221_MFR_TEMP_VALLEY		0xcc

typedef struct Ir35221State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t regs[0x100];

    uint8_t len;
    uint8_t buf[32];
    uint8_t pointer;

} Ir35221State;

#define TYPE_IR35221 "ir35221"
#define IR35221(obj) OBJECT_CHECK(Ir35221State, (obj), TYPE_IR35221)

static void ir35221_read(Ir35221State *s)
{
    s->len = 0;

    switch (s->pointer) {
    case PMBUS_MFR_ID:
        s->buf[s->len++] = 2;  /* Byte count */
        s->buf[s->len++] = 'R';
        s->buf[s->len++] = 'I';
        break;
    case PMBUS_MFR_MODEL:
        s->buf[s->len++] = 2;   /* Byte count */
        s->buf[s->len++] = 0x6c;
        s->buf[s->len++] = 0x00;
        break;
    default:
        s->buf[s->len++] = s->regs[s->pointer];
        break;
    }
}

static void ir35221_write(Ir35221State *s)
{
    /* No writes */
}

static uint8_t ir35221_recv(I2CSlave *i2c)
{
    Ir35221State *s = IR35221(i2c);

    if (s->len < sizeof(s->buf)) {
        return s->buf[s->len++];
    } else {
        return 0xff;
    }
}

static int ir35221_send(I2CSlave *i2c, uint8_t data)
{
    Ir35221State *s = IR35221(i2c);

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
        ir35221_write(s);
    }

    return 0;
}

static int ir35221_event(I2CSlave *i2c, enum i2c_event event)
{
    Ir35221State *s = IR35221(i2c);

    /* TODO: handle SMBus "block read" */

    switch (event) {
    case I2C_START_RECV:
        ir35221_read(s);
        break;
    case I2C_START_SEND:
    case I2C_NACK:
    case I2C_FINISH:
         s->pointer = 0xFF;
        break;
    }

    s->len = 0;
    return 0;
}

static const VMStateDescription vmstate_ir35221 = {
    .name = TYPE_IR35221,
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, Ir35221State),
        VMSTATE_UINT8_ARRAY(buf, Ir35221State, 32),
        VMSTATE_UINT8(pointer, Ir35221State),
        VMSTATE_UINT8_ARRAY(regs, Ir35221State, 0x100),
        VMSTATE_I2C_SLAVE(i2c, Ir35221State),
        VMSTATE_END_OF_LIST()
    }
};

static void ir35221_reset(DeviceState *dev)
{
    Ir35221State *s = IR35221(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0xFF;
}

static void ir35221_realize(DeviceState *dev, Error **errp)
{
    ;
}

static void ir35221_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = ir35221_realize;
    dc->reset = ir35221_reset;
    k->event = ir35221_event;
    k->recv = ir35221_recv;
    k->send = ir35221_send;
    dc->vmsd = &vmstate_ir35221;
}

static const TypeInfo ir35221_info = {
    .name          = TYPE_IR35221,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ir35221State),
    .class_init    = ir35221_class_init,
};

static void ir35221_register_types(void)
{
    type_register_static(&ir35221_info);
}

type_init(ir35221_register_types)
