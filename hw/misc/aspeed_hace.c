/*
 * ASPEED Hash and Crypto Engine
 *
 * Copyright (c) 2024 Seagate Technology LLC and/or its Affiliates
 * Copyright (C) 2021 IBM Corp.
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/misc/aspeed_hace.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "crypto/hash.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "trace.h"

#define R_CRYPT_CMD     (0x10 / 4)

#define R_STATUS        (0x1c / 4)
#define HASH_IRQ        BIT(9)
#define CRYPT_IRQ       BIT(12)
#define TAG_IRQ         BIT(15)

#define R_HASH_SRC      (0x20 / 4)
#define R_HASH_DEST     (0x24 / 4)
#define R_HASH_KEY_BUFF (0x28 / 4)
#define R_HASH_SRC_LEN  (0x2c / 4)
#define R_HASH_SRC_HI      (0x90 / 4)
#define R_HASH_DEST_HI     (0x94 / 4)
#define R_HASH_KEY_BUFF_HI (0x98 / 4)

#define R_HASH_CMD      (0x30 / 4)
/* Hash algorithm selection */
#define  HASH_ALGO_MASK                 (BIT(4) | BIT(5) | BIT(6))
#define  HASH_ALGO_MD5                  0
#define  HASH_ALGO_SHA1                 BIT(5)
#define  HASH_ALGO_SHA224               BIT(6)
#define  HASH_ALGO_SHA256               (BIT(4) | BIT(6))
#define  HASH_ALGO_SHA512_SERIES        (BIT(5) | BIT(6))
/* SHA512 algorithm selection */
#define  SHA512_HASH_ALGO_MASK          (BIT(10) | BIT(11) | BIT(12))
#define  HASH_ALGO_SHA512_SHA512        0
#define  HASH_ALGO_SHA512_SHA384        BIT(10)
#define  HASH_ALGO_SHA512_SHA256        BIT(11)
#define  HASH_ALGO_SHA512_SHA224        (BIT(10) | BIT(11))
/* HMAC modes */
#define  HASH_HMAC_MASK                 (BIT(7) | BIT(8))
#define  HASH_DIGEST                    0
#define  HASH_DIGEST_HMAC               BIT(7)
#define  HASH_DIGEST_ACCUM              BIT(8)
#define  HASH_HMAC_KEY                  (BIT(7) | BIT(8))
/* Cascaded operation modes */
#define  HASH_ONLY                      0
#define  HASH_ONLY2                     BIT(0)
#define  HASH_CRYPT_THEN_HASH           BIT(1)
#define  HASH_HASH_THEN_CRYPT           (BIT(0) | BIT(1))
/* Other cmd bits */
#define  HASH_IRQ_EN                    BIT(9)
#define  HASH_SG_EN                     BIT(18)
#define  CRYPT_IRQ_EN                   BIT(12)
/* Scatter-gather data list */
#define SG_LIST_LEN_SIZE                4
#define SG_LIST_LEN_MASK                0x0FFFFFFF
#define SG_LIST_LEN_LAST                BIT(31)
#define SG_LIST_ADDR_SIZE               4
#define SG_LIST_ADDR_MASK               0x7FFFFFFF
#define SG_LIST_ENTRY_SIZE              (SG_LIST_LEN_SIZE + SG_LIST_ADDR_SIZE)

static const struct {
    uint32_t mask;
    QCryptoHashAlgo algo;
} hash_algo_map[] = {
    { HASH_ALGO_MD5, QCRYPTO_HASH_ALGO_MD5 },
    { HASH_ALGO_SHA1, QCRYPTO_HASH_ALGO_SHA1 },
    { HASH_ALGO_SHA224, QCRYPTO_HASH_ALGO_SHA224 },
    { HASH_ALGO_SHA256, QCRYPTO_HASH_ALGO_SHA256 },
    { HASH_ALGO_SHA512_SERIES | HASH_ALGO_SHA512_SHA512,
      QCRYPTO_HASH_ALGO_SHA512 },
    { HASH_ALGO_SHA512_SERIES | HASH_ALGO_SHA512_SHA384,
      QCRYPTO_HASH_ALGO_SHA384 },
    { HASH_ALGO_SHA512_SERIES | HASH_ALGO_SHA512_SHA256,
      QCRYPTO_HASH_ALGO_SHA256 },
};

static int hash_algo_lookup(uint32_t reg)
{
    int i;

    reg &= HASH_ALGO_MASK | SHA512_HASH_ALGO_MASK;

    for (i = 0; i < ARRAY_SIZE(hash_algo_map); i++) {
        if (reg == hash_algo_map[i].mask) {
            return hash_algo_map[i].algo;
        }
    }

    return -1;
}

/**
 * Check whether the request contains padding message.
 *
 * @param s             aspeed hace state object
 * @param iov           iov of current request
 * @param req_len       length of the current request
 * @param total_msg_len length of all acc_mode requests(excluding padding msg)
 * @param pad_offset    start offset of padding message
 */
static bool has_padding(AspeedHACEState *s, struct iovec *iov,
                        hwaddr req_len, uint32_t *total_msg_len,
                        uint32_t *pad_offset)
{
    *total_msg_len = (uint32_t)(ldq_be_p(iov->iov_base + req_len - 8) / 8);
    /*
     * SG_LIST_LEN_LAST asserted in the request length doesn't mean it is the
     * last request. The last request should contain padding message.
     * We check whether message contains padding by
     *   1. Get total message length. If the current message contains
     *      padding, the last 8 bytes are total message length.
     *   2. Check whether the total message length is valid.
     *      If it is valid, the value should less than or equal to
     *      total_req_len.
     *   3. Current request len - padding_size to get padding offset.
     *      The padding message's first byte should be 0x80
     */
    if (*total_msg_len <= s->total_req_len) {
        uint32_t padding_size = s->total_req_len - *total_msg_len;
        uint8_t *padding = iov->iov_base;

        if (padding_size > req_len) {
            return false;
        }

        *pad_offset = req_len - padding_size;
        if (padding[*pad_offset] == 0x80) {
            return true;
        }
    }

    return false;
}

static void do_hash_operation(AspeedHACEState *s, int algo, bool sg_mode,
                              bool acc_mode)
{
    AspeedHACEClass *ahc = ASPEED_HACE_GET_CLASS(s);
    bool sg_acc_mode_final_request = false;
    g_autofree uint8_t *digest_buf = NULL;
    struct iovec iov[ASPEED_HACE_MAX_SG];
    uint64_t digest_addr = 0;
    Error *local_err = NULL;
    uint32_t total_msg_len;
    size_t digest_len = 0;
    uint32_t sg_addr = 0;
    uint32_t pad_offset;
    uint32_t len = 0;
    uint64_t src = 0;
    void *haddr;
    hwaddr plen;
    int i;

    if (acc_mode && s->hash_ctx == NULL) {
        s->hash_ctx = qcrypto_hash_new(algo, &local_err);
        if (s->hash_ctx == NULL) {
            qemu_log_mask(LOG_GUEST_ERROR, "qcrypto hash failed : %s",
                          error_get_pretty(local_err));
            error_free(local_err);
            return;
        }
    }

    if (sg_mode) {
        for (i = 0; !(len & SG_LIST_LEN_LAST); i++) {
            if (i == ASPEED_HACE_MAX_SG) {
                qemu_log_mask(LOG_GUEST_ERROR,
                        "aspeed_hace: guest failed to set end of sg list marker\n");
                break;
            }

            src = deposit64(src, 0, 32, s->regs[R_HASH_SRC]);
            if (ahc->has_dma64) {
                src = deposit64(src, 32, 32, s->regs[R_HASH_SRC_HI]);
            }
            trace_aspeed_hace_addr("src", src);
            src += i * SG_LIST_ENTRY_SIZE;

            len = address_space_ldl_le(&s->dram_as, src,
                                       MEMTXATTRS_UNSPECIFIED, NULL);

            sg_addr = address_space_ldl_le(&s->dram_as, src + SG_LIST_LEN_SIZE,
                                           MEMTXATTRS_UNSPECIFIED, NULL);
            sg_addr &= SG_LIST_ADDR_MASK;
            trace_aspeed_hace_sg(i, sg_addr, len);
            /*
             * Ideally, sg_addr should be 64-bit for the AST2700, using the
             * following program to obtain the 64-bit sg_addr and convert it
             * to a DRAM offset:
             * sg_addr = deposit64(sg_addr, 32, 32,
             *      address_space_ldl_le(&s->dram_as, src + SG_ADDR_LEN_SIZE,
             *                           MEMTXATTRS_UNSPECIFIED, NULL);
             * sg_addr -= 0x400000000;
             *
             * To maintain compatibility with older SoCs such as the AST2600,
             * the AST2700 HW automatically set bit 34 of the 64-bit sg_addr.
             * As a result, the firmware only needs to provide a 32-bit sg_addr
             * containing bits [31:0]. This is sufficient for the AST2700, as
             * it uses a DRAM offset rather than a DRAM address.
             */

            plen = len & SG_LIST_LEN_MASK;
            haddr = address_space_map(&s->dram_as, sg_addr, &plen, false,
                                      MEMTXATTRS_UNSPECIFIED);
            if (haddr == NULL) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: qcrypto failed\n", __func__);
                return;
            }
            iov[i].iov_base = haddr;
            if (acc_mode) {
                s->total_req_len += plen;

                if (has_padding(s, &iov[i], plen, &total_msg_len,
                                &pad_offset)) {
                    /* Padding being present indicates the final request */
                    sg_acc_mode_final_request = true;
                    iov[i].iov_len = pad_offset;
                } else {
                    iov[i].iov_len = plen;
                }
            } else {
                iov[i].iov_len = plen;
            }
        }
    } else {
        plen = s->regs[R_HASH_SRC_LEN];
        src = deposit64(src, 0, 32, s->regs[R_HASH_SRC]);
        trace_aspeed_hace_addr("src", src);
        if (ahc->has_dma64) {
            src = deposit64(src, 32, 32, s->regs[R_HASH_SRC_HI]);
        }
        haddr = address_space_map(&s->dram_as, src,
                                  &plen, false, MEMTXATTRS_UNSPECIFIED);
        if (haddr == NULL) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: qcrypto failed\n", __func__);
            return;
        }
        iov[0].iov_base = haddr;
        i = 1;
        if (acc_mode) {
            s->total_req_len += plen;

            if (has_padding(s, &iov[0], plen, &total_msg_len,
                            &pad_offset)) {
                /* Padding being present indicates the final request */
                sg_acc_mode_final_request = true;
                iov[0].iov_len = pad_offset;
            } else {
                iov[0].iov_len = plen;
            }
        } else {
            iov[0].iov_len = plen;
        }
    }

    if (acc_mode) {
        if (qcrypto_hash_updatev(s->hash_ctx, iov, i, &local_err) < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "qcrypto hash update failed : %s",
                          error_get_pretty(local_err));
            error_free(local_err);
            return;
        }

        if (sg_acc_mode_final_request) {
            if (qcrypto_hash_finalize_bytes(s->hash_ctx, &digest_buf,
                                            &digest_len, &local_err)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "qcrypto hash finalize failed : %s",
                              error_get_pretty(local_err));
                error_free(local_err);
                local_err = NULL;
            }

            qcrypto_hash_free(s->hash_ctx);

            s->hash_ctx = NULL;
            s->total_req_len = 0;
        }
    } else if (qcrypto_hash_bytesv(algo, iov, i, &digest_buf,
                                   &digest_len, &local_err) < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "qcrypto hash bytesv failed : %s",
                      error_get_pretty(local_err));
        error_free(local_err);
        return;
    }

    digest_addr = deposit64(digest_addr, 0, 32, s->regs[R_HASH_DEST]);
    if (ahc->has_dma64) {
        digest_addr = deposit64(digest_addr, 32, 32, s->regs[R_HASH_DEST_HI]);
    }
    trace_aspeed_hace_addr("digest", digest_addr);
    if (address_space_write(&s->dram_as, digest_addr,
                            MEMTXATTRS_UNSPECIFIED,
                            digest_buf, digest_len)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "aspeed_hace: address space write failed\n");
    }

    for (; i > 0; i--) {
        address_space_unmap(&s->dram_as, iov[i - 1].iov_base,
                            iov[i - 1].iov_len, false,
                            iov[i - 1].iov_len);
    }
}

static uint64_t aspeed_hace_read(void *opaque, hwaddr addr, unsigned int size)
{
    AspeedHACEState *s = ASPEED_HACE(opaque);

    addr >>= 2;

    if (addr >= ASPEED_HACE_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
        return 0;
    }

    trace_aspeed_hace_read(addr << 2, s->regs[addr]);
    return s->regs[addr];
}

static void aspeed_hace_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned int size)
{
    AspeedHACEState *s = ASPEED_HACE(opaque);
    AspeedHACEClass *ahc = ASPEED_HACE_GET_CLASS(s);

    addr >>= 2;

    if (addr >= ASPEED_HACE_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
        return;
    }

    trace_aspeed_hace_write(addr << 2, data);

    switch (addr) {
    case R_STATUS:
        if (data & HASH_IRQ) {
            data &= ~HASH_IRQ;

            if (s->regs[addr] & HASH_IRQ) {
                qemu_irq_lower(s->irq);
            }
        }
        if (ahc->raise_crypt_interrupt_workaround) {
            if (data & CRYPT_IRQ) {
                data &= ~CRYPT_IRQ;

                if (s->regs[addr] & CRYPT_IRQ) {
                    qemu_irq_lower(s->irq);
                }
            }
        }
        break;
    case R_HASH_SRC:
        data &= ahc->src_mask;
        break;
    case R_HASH_DEST:
        data &= ahc->dest_mask;
        break;
    case R_HASH_KEY_BUFF:
        data &= ahc->key_mask;
        break;
    case R_HASH_SRC_LEN:
        data &= 0x0FFFFFFF;
        break;
    case R_HASH_CMD: {
        int algo;
        data &= ahc->hash_mask;

        if ((data & HASH_DIGEST_HMAC)) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: HMAC mode not implemented\n",
                          __func__);
        }
        if (data & BIT(1)) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: Cascaded mode not implemented\n",
                          __func__);
        }
        algo = hash_algo_lookup(data);
        if (algo < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: Invalid hash algorithm selection 0x%"PRIx64"\n",
                        __func__, data & ahc->hash_mask);
        } else {
            do_hash_operation(s, algo, data & HASH_SG_EN,
                    ((data & HASH_HMAC_MASK) == HASH_DIGEST_ACCUM));
        }

        /*
         * Set status bits to indicate completion. Testing shows hardware sets
         * these irrespective of HASH_IRQ_EN.
         */
        s->regs[R_STATUS] |= HASH_IRQ;

        if (data & HASH_IRQ_EN) {
            qemu_irq_raise(s->irq);
        }
        break;
    }
    case R_CRYPT_CMD:
        qemu_log_mask(LOG_UNIMP, "%s: Crypt commands not implemented\n",
                       __func__);
        if (ahc->raise_crypt_interrupt_workaround) {
            s->regs[R_STATUS] |= CRYPT_IRQ;
            if (data & CRYPT_IRQ_EN) {
                qemu_irq_raise(s->irq);
            }
        }
        break;
    case R_HASH_SRC_HI:
        data &= ahc->src_hi_mask;
        break;
    case R_HASH_DEST_HI:
        data &= ahc->dest_hi_mask;
        break;
    case R_HASH_KEY_BUFF_HI:
        data &= ahc->key_hi_mask;
        break;
    default:
        break;
    }

    s->regs[addr] = data;
}

static const MemoryRegionOps aspeed_hace_ops = {
    .read = aspeed_hace_read,
    .write = aspeed_hace_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void aspeed_hace_reset(DeviceState *dev)
{
    struct AspeedHACEState *s = ASPEED_HACE(dev);

    if (s->hash_ctx != NULL) {
        qcrypto_hash_free(s->hash_ctx);
        s->hash_ctx = NULL;
    }

    memset(s->regs, 0, sizeof(s->regs));
    s->total_req_len = 0;
}

static void aspeed_hace_realize(DeviceState *dev, Error **errp)
{
    AspeedHACEState *s = ASPEED_HACE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    AspeedHACEClass *ahc = ASPEED_HACE_GET_CLASS(s);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_hace_ops, s,
            TYPE_ASPEED_HACE, ahc->mem_size);

    if (!s->dram_mr) {
        error_setg(errp, TYPE_ASPEED_HACE ": 'dram' link not set");
        return;
    }

    address_space_init(&s->dram_as, s->dram_mr, "dram");

    sysbus_init_mmio(sbd, &s->iomem);
}

static const Property aspeed_hace_properties[] = {
    DEFINE_PROP_LINK("dram", AspeedHACEState, dram_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};


static const VMStateDescription vmstate_aspeed_hace = {
    .name = TYPE_ASPEED_HACE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedHACEState, ASPEED_HACE_NR_REGS),
        VMSTATE_UINT32(total_req_len, AspeedHACEState),
        VMSTATE_END_OF_LIST(),
    }
};

static void aspeed_hace_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_hace_realize;
    device_class_set_legacy_reset(dc, aspeed_hace_reset);
    device_class_set_props(dc, aspeed_hace_properties);
    dc->vmsd = &vmstate_aspeed_hace;
}

static const TypeInfo aspeed_hace_info = {
    .name = TYPE_ASPEED_HACE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedHACEState),
    .class_init = aspeed_hace_class_init,
    .class_size = sizeof(AspeedHACEClass)
};

static void aspeed_ast2400_hace_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedHACEClass *ahc = ASPEED_HACE_CLASS(klass);

    dc->desc = "AST2400 Hash and Crypto Engine";

    ahc->mem_size = 0x1000;
    ahc->src_mask = 0x0FFFFFFF;
    ahc->dest_mask = 0x0FFFFFF8;
    ahc->key_mask = 0x0FFFFFC0;
    ahc->hash_mask = 0x000003ff; /* No SG or SHA512 modes */
}

static const TypeInfo aspeed_ast2400_hace_info = {
    .name = TYPE_ASPEED_AST2400_HACE,
    .parent = TYPE_ASPEED_HACE,
    .class_init = aspeed_ast2400_hace_class_init,
};

static void aspeed_ast2500_hace_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedHACEClass *ahc = ASPEED_HACE_CLASS(klass);

    dc->desc = "AST2500 Hash and Crypto Engine";

    ahc->mem_size = 0x1000;
    ahc->src_mask = 0x3fffffff;
    ahc->dest_mask = 0x3ffffff8;
    ahc->key_mask = 0x3FFFFFC0;
    ahc->hash_mask = 0x000003ff; /* No SG or SHA512 modes */
}

static const TypeInfo aspeed_ast2500_hace_info = {
    .name = TYPE_ASPEED_AST2500_HACE,
    .parent = TYPE_ASPEED_HACE,
    .class_init = aspeed_ast2500_hace_class_init,
};

static void aspeed_ast2600_hace_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedHACEClass *ahc = ASPEED_HACE_CLASS(klass);

    dc->desc = "AST2600 Hash and Crypto Engine";

    ahc->mem_size = 0x10000;
    ahc->src_mask = 0x7FFFFFFF;
    ahc->dest_mask = 0x7FFFFFF8;
    ahc->key_mask = 0x7FFFFFF8;
    ahc->hash_mask = 0x00147FFF;
}

static const TypeInfo aspeed_ast2600_hace_info = {
    .name = TYPE_ASPEED_AST2600_HACE,
    .parent = TYPE_ASPEED_HACE,
    .class_init = aspeed_ast2600_hace_class_init,
};

static void aspeed_ast1030_hace_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedHACEClass *ahc = ASPEED_HACE_CLASS(klass);

    dc->desc = "AST1030 Hash and Crypto Engine";

    ahc->mem_size = 0x10000;
    ahc->src_mask = 0x7FFFFFFF;
    ahc->dest_mask = 0x7FFFFFF8;
    ahc->key_mask = 0x7FFFFFF8;
    ahc->hash_mask = 0x00147FFF;
}

static const TypeInfo aspeed_ast1030_hace_info = {
    .name = TYPE_ASPEED_AST1030_HACE,
    .parent = TYPE_ASPEED_HACE,
    .class_init = aspeed_ast1030_hace_class_init,
};

static void aspeed_ast2700_hace_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedHACEClass *ahc = ASPEED_HACE_CLASS(klass);

    dc->desc = "AST2700 Hash and Crypto Engine";

    ahc->mem_size = 0x100;
    ahc->src_mask = 0x7FFFFFFF;
    ahc->dest_mask = 0x7FFFFFF8;
    ahc->key_mask = 0x7FFFFFF8;
    ahc->hash_mask = 0x00147FFF;

    /*
     * The AST2700 supports a maximum DRAM size of 8 GB, with a DRAM
     * addressable range from 0x0_0000_0000 to 0x1_FFFF_FFFF. Since this range
     * fits within 34 bits, only bits [33:0] are needed to store the DRAM
     * offset. To optimize address storage, the high physical address bits
     * [1:0] of the source, digest and key buffer addresses are stored as
     * dram_offset bits [33:32].
     *
     * This approach eliminates the need to reduce the high part of the DRAM
     * physical address for DMA operations. Previously, this was calculated as
     * (high physical address bits [7:0] - 4), since the DRAM start address is
     * 0x4_00000000, making the high part address [7:0] - 4.
     */
    ahc->src_hi_mask = 0x00000003;
    ahc->dest_hi_mask = 0x00000003;
    ahc->key_hi_mask = 0x00000003;

    /*
     * Currently, it does not support the CRYPT command. Instead, it only
     * sends an interrupt to notify the firmware that the crypt command
     * has completed. It is a temporary workaround.
     */
    ahc->raise_crypt_interrupt_workaround = true;
    ahc->has_dma64 = true;
}

static const TypeInfo aspeed_ast2700_hace_info = {
    .name = TYPE_ASPEED_AST2700_HACE,
    .parent = TYPE_ASPEED_HACE,
    .class_init = aspeed_ast2700_hace_class_init,
};

static void aspeed_hace_register_types(void)
{
    type_register_static(&aspeed_ast2400_hace_info);
    type_register_static(&aspeed_ast2500_hace_info);
    type_register_static(&aspeed_ast2600_hace_info);
    type_register_static(&aspeed_ast1030_hace_info);
    type_register_static(&aspeed_ast2700_hace_info);
    type_register_static(&aspeed_hace_info);
}

type_init(aspeed_hace_register_types);
