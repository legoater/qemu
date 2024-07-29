/*
 * QEMU Crypto hash algorithms
 *
 * Copyright (c) 2024 Seagate Technology LLC and/or its Affiliates
 * Copyright (c) 2016 Red Hat, Inc.
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
#include "qapi/error.h"
#include "crypto/hash.h"
#include "hashpriv.h"


static int qcrypto_hash_alg_map[QCRYPTO_HASH_ALG__MAX] = {
    [QCRYPTO_HASH_ALG_MD5] = G_CHECKSUM_MD5,
    [QCRYPTO_HASH_ALG_SHA1] = G_CHECKSUM_SHA1,
    [QCRYPTO_HASH_ALG_SHA224] = -1,
    [QCRYPTO_HASH_ALG_SHA256] = G_CHECKSUM_SHA256,
    [QCRYPTO_HASH_ALG_SHA384] = -1,
    [QCRYPTO_HASH_ALG_SHA512] = G_CHECKSUM_SHA512,
    [QCRYPTO_HASH_ALG_RIPEMD160] = -1,
};

gboolean qcrypto_hash_supports(QCryptoHashAlgorithm alg)
{
    if (alg < G_N_ELEMENTS(qcrypto_hash_alg_map) &&
        qcrypto_hash_alg_map[alg] != -1) {
        return true;
    }
    return false;
}


static int
qcrypto_glib_hash_bytesv(QCryptoHashAlgorithm alg,
                         const struct iovec *iov,
                         size_t niov,
                         uint8_t **result,
                         size_t *resultlen,
                         Error **errp)
{
    int i, ret;
    GChecksum *cs;

    if (!qcrypto_hash_supports(alg)) {
        error_setg(errp,
                   "Unknown hash algorithm %d",
                   alg);
        return -1;
    }

    cs = g_checksum_new(qcrypto_hash_alg_map[alg]);

    for (i = 0; i < niov; i++) {
        g_checksum_update(cs, iov[i].iov_base, iov[i].iov_len);
    }

    ret = g_checksum_type_get_length(qcrypto_hash_alg_map[alg]);
    if (ret < 0) {
        error_setg(errp, "%s",
                   "Unable to get hash length");
        goto error;
    }
    if (*resultlen == 0) {
        *resultlen = ret;
        *result = g_new0(uint8_t, *resultlen);
    } else if (*resultlen != ret) {
        error_setg(errp,
                   "Result buffer size %zu is smaller than hash %d",
                   *resultlen, ret);
        goto error;
    }

    g_checksum_get_digest(cs, *result, resultlen);

    g_checksum_free(cs);
    return 0;

 error:
    g_checksum_free(cs);
    return -1;
}


static
int qcrypto_glib_hash_accumulate_new_ctx(QCryptoHashAlgorithm alg,
                                         qcrypto_hash_accumulate_ctx_t **accumulate_ctx,
                                         Error **errp)
{
    if (!qcrypto_hash_supports(alg)) {
        error_setg(errp,
                   "Unknown hash algorithm %d",
                   alg);
        return -1;
    }

    *accumulate_ctx = g_checksum_new(qcrypto_hash_alg_map[alg]);

    return 0;
}

static
int qcrypto_glib_hash_accumulate_free_ctx(qcrypto_hash_accumulate_ctx_t *hash_ctx,
                                          Error **errp)
{
    if (hash_ctx != NULL) {
        g_checksum_free((GChecksum *) hash_ctx);
    }

    return 0;
}


static
int qcrypto_glib_hash_accumulate_bytesv(QCryptoHashAlgorithm alg,
                                        qcrypto_hash_accumulate_ctx_t *accumulate_ctx,
                                        const struct iovec *iov,
                                        size_t niov,
                                        uint8_t **result,
                                        size_t *resultlen,
                                        Error **errp)
{
    int i, ret;
    GChecksum *ctx_copy;

    if (!qcrypto_hash_supports(alg)) {
        error_setg(errp,
                   "Unknown hash algorithm %d",
                   alg);
        return -1;
    }

    for (i = 0; i < niov; i++) {
        g_checksum_update((GChecksum *) accumulate_ctx, iov[i].iov_base, iov[i].iov_len);
    }

    ret = g_checksum_type_get_length(qcrypto_hash_alg_map[alg]);
    if (ret < 0) {
        error_setg(errp, "%s",
                   "Unable to get hash length");
        return -1;
    }
    if (*resultlen == 0) {
        *resultlen = ret;
        *result = g_new0(uint8_t, *resultlen);
    } else if (*resultlen != ret) {
        error_setg(errp,
                   "Result buffer size %zu is smaller than hash %d",
                   *resultlen, ret);
        return -1;
    }

    /*
    Make a copy so we don't distort the main context
    by calculating the intermediate hash.
    */
    ctx_copy = g_checksum_copy((GChecksum *) accumulate_ctx);
    if (ctx_copy == NULL) {
        error_setg(errp, "Unable to make copy: %s", __func__);
        return -1;
    }

    g_checksum_get_digest((GChecksum *) ctx_copy, *result, resultlen);
    g_checksum_free(ctx_copy);

    return 0;
}


QCryptoHashDriver qcrypto_hash_lib_driver = {
    .hash_bytesv = qcrypto_glib_hash_bytesv,
    .hash_accumulate_bytesv = qcrypto_glib_hash_accumulate_bytesv,
    .accumulate_new_ctx = qcrypto_glib_hash_accumulate_new_ctx,
    .accumulate_free_ctx = qcrypto_glib_hash_accumulate_free_ctx,
};
