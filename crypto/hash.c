/*
 * QEMU Crypto hash algorithms
 *
 * Copyright (c) 2024 Seagate Technology LLC and/or its Affiliates
 * Copyright (c) 2015 Red Hat, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "qemu/osdep.h"
#include "crypto/hash.h"
#include "hashpriv.h"

static size_t qcrypto_hash_alg_size[QCRYPTO_HASH_ALG__MAX] = {
    [QCRYPTO_HASH_ALG_MD5] = 16,
    [QCRYPTO_HASH_ALG_SHA1] = 20,
    [QCRYPTO_HASH_ALG_SHA224] = 28,
    [QCRYPTO_HASH_ALG_SHA256] = 32,
    [QCRYPTO_HASH_ALG_SHA384] = 48,
    [QCRYPTO_HASH_ALG_SHA512] = 64,
    [QCRYPTO_HASH_ALG_RIPEMD160] = 20,
};

size_t qcrypto_hash_digest_len(QCryptoHashAlgorithm alg)
{
    assert(alg < G_N_ELEMENTS(qcrypto_hash_alg_size));
    return qcrypto_hash_alg_size[alg];
}

int qcrypto_hash_bytesv(QCryptoHashAlgorithm alg,
                        const struct iovec *iov,
                        size_t niov,
                        uint8_t **result,
                        size_t *resultlen,
                        Error **errp)
{
    int fail;
    QCryptoHash *ctx = qcrypto_hash_new(alg, errp);

    if (ctx) {
        fail = qcrypto_hash_updatev(ctx, iov, niov, errp) ||
               qcrypto_hash_finalize_bytes(ctx, result, resultlen, errp);

        /* Ensure context is always freed regardless of error */
        qcrypto_hash_free(ctx);
    } else {
        fail = -1;
    }

    return fail;
}


int qcrypto_hash_bytes(QCryptoHashAlgorithm alg,
                       const char *buf,
                       size_t len,
                       uint8_t **result,
                       size_t *resultlen,
                       Error **errp)
{
    struct iovec iov = { .iov_base = (char *)buf,
                         .iov_len = len };
    return qcrypto_hash_bytesv(alg, &iov, 1, result, resultlen, errp);
}

int qcrypto_hash_updatev(QCryptoHash *hash,
                         const struct iovec *iov,
                         size_t niov,
                         Error **errp)
{
#ifdef CONFIG_AF_ALG
    return qcrypto_hash_afalg_driver.hash_update(hash, iov, niov, errp);
#else
    return qcrypto_hash_lib_driver.hash_update(hash, iov, niov, errp);
#endif /* CONFIG_AF_ALG */
}

int qcrypto_hash_update(QCryptoHash *hash,
                        const char *buf,
                        size_t len,
                        Error **errp)
{
    struct iovec iov = { .iov_base = (char *)buf, .iov_len = len };

    return qcrypto_hash_updatev(hash, &iov, 1, errp);
}

QCryptoHash *qcrypto_hash_new(QCryptoHashAlgorithm alg, Error **errp)
{
#ifdef CONFIG_AF_ALG
    return qcrypto_hash_afalg_driver.hash_new(alg, errp);
#else
    return qcrypto_hash_lib_driver.hash_new(alg, errp);
#endif /* CONFIG_AF_ALG */
}

void qcrypto_hash_free(QCryptoHash *hash)
{
#ifdef CONFIG_AF_ALG
    qcrypto_hash_afalg_driver.hash_free(hash);
#else
    qcrypto_hash_lib_driver.hash_free(hash);
#endif /* CONFIG_AF_ALG */
}

int qcrypto_hash_finalize_bytes(QCryptoHash *hash,
                                uint8_t **result,
                                size_t *result_len,
                                Error **errp)
{
#ifdef CONFIG_AF_ALG
    return qcrypto_hash_afalg_driver.hash_finalize(hash, result, result_len,
                                                   errp);
#else
    return qcrypto_hash_lib_driver.hash_finalize(hash, result, result_len, errp);
#endif /* CONFIG_AF_ALG */
}

static const char hex[] = "0123456789abcdef";

int qcrypto_hash_finalize_digest(QCryptoHash *hash,
                                 char **digest,
                                 Error **errp)
{
    int ret;
    uint8_t *result = NULL;
    size_t resultlen = 0;
    size_t i;

    ret = qcrypto_hash_finalize_bytes(hash, &result, &resultlen, errp);
    if (ret == 0) {
        *digest = g_new0(char, (resultlen * 2) + 1);
        for (i = 0 ; i < resultlen ; i++) {
            (*digest)[(i * 2)] = hex[(result[i] >> 4) & 0xf];
            (*digest)[(i * 2) + 1] = hex[result[i] & 0xf];
        }
        (*digest)[resultlen * 2] = '\0';
        g_free(result);
    }

    return ret;
}

int qcrypto_hash_finalize_base64(QCryptoHash *hash,
                                 char **base64,
                                 Error **errp)
{
    int ret;
    uint8_t *result = NULL;
    size_t resultlen = 0;

    ret = qcrypto_hash_finalize_bytes(hash, &result, &resultlen, errp);
    if (ret == 0) {
        *base64 = g_base64_encode(result, resultlen);
        g_free(result);
    }

    return ret;
}

int qcrypto_hash_digestv(QCryptoHashAlgorithm alg,
                         const struct iovec *iov,
                         size_t niov,
                         char **digest,
                         Error **errp)
{
    bool fail;
    QCryptoHash *ctx = qcrypto_hash_new(alg, errp);

    if (ctx) {
        fail = qcrypto_hash_updatev(ctx, iov, niov, errp) ||
               qcrypto_hash_finalize_digest(ctx, digest, errp);

        /* Ensure context is always freed regardless of error */
        qcrypto_hash_free(ctx);
    } else {
        fail = false;
    }

    return fail;
}

int qcrypto_hash_digest(QCryptoHashAlgorithm alg,
                        const char *buf,
                        size_t len,
                        char **digest,
                        Error **errp)
{
    struct iovec iov = { .iov_base = (char *)buf, .iov_len = len };

    return qcrypto_hash_digestv(alg, &iov, 1, digest, errp);
}

int qcrypto_hash_base64v(QCryptoHashAlgorithm alg,
                         const struct iovec *iov,
                         size_t niov,
                         char **base64,
                         Error **errp)
{
    bool fail;
    QCryptoHash *ctx = qcrypto_hash_new(alg, errp);

    if (ctx) {
        fail = qcrypto_hash_updatev(ctx, iov, niov, errp) ||
               qcrypto_hash_finalize_base64(ctx, base64, errp);

        /* Ensure context is always freed regardless of error */
        qcrypto_hash_free(ctx);
    } else {
        fail = 1;
    }

    return fail;
}

int qcrypto_hash_base64(QCryptoHashAlgorithm alg,
                        const char *buf,
                        size_t len,
                        char **base64,
                        Error **errp)
{
    struct iovec iov = { .iov_base = (char *)buf, .iov_len = len };

    return qcrypto_hash_base64v(alg, &iov, 1, base64, errp);
}
