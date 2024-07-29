/*
 * QEMU Crypto hash driver supports
 *
 * Copyright (c) 2024 Seagate Technology LLC and/or its Affiliates
 * Copyright (c) 2017 HUAWEI TECHNOLOGIES CO., LTD.
 *
 * Authors:
 *    Longpeng(Mike) <longpeng2@huawei.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * (at your option) any later version.  See the COPYING file in the
 * top-level directory.
 *
 */

#ifndef QCRYPTO_HASHPRIV_H
#define QCRYPTO_HASHPRIV_H

typedef struct QCryptoHashDriver QCryptoHashDriver;

struct QCryptoHashDriver {
    int (*hash_bytesv)(QCryptoHashAlgorithm alg,
                       const struct iovec *iov,
                       size_t niov,
                       uint8_t **result,
                       size_t *resultlen,
                       Error **errp);
    int (*hash_accumulate_bytesv)(QCryptoHashAlgorithm alg,
                                  void *hash_ctx,
                                  const struct iovec *iov,
                                  size_t niov,
                                  uint8_t **result,
                                  size_t *resultlen,
                                  Error **errp);
    int (*accumulate_new_ctx)(QCryptoHashAlgorithm alg, void **hash_ctx,
                              Error **errp);
    int (*accumulate_free_ctx)(void *hash_ctx, Error **errp);
};

extern QCryptoHashDriver qcrypto_hash_lib_driver;

#ifdef CONFIG_AF_ALG

#include "afalgpriv.h"

extern QCryptoHashDriver qcrypto_hash_afalg_driver;

#endif

#endif
