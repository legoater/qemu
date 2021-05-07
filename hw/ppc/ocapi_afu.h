/*
 * QEMU PowerPC PowerNV OpenCAPI AFU model
 *
 * Copyright (c) 2019-2020, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#ifndef PPC_OCAPI_AFU_H
#define PPC_OCAPI_AFU_H

typedef struct {
    void (*write_cfg)(int func, int reg, uint64_t val, int size);
    uint64_t (*read_cfg)(int func, int reg, int size);
} ocapi_afu_t;

#endif /* PPC_OCAPI_AFU_H */
