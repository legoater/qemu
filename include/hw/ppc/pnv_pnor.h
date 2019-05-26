/*
 * QEMU PowerNV PNOR related functions
 *
 * Copyright (c) 2015-2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */
#ifndef _PPC_PNV_PNOR_H
#define _PPC_PNV_PNOR_H

extern int pnv_pnor_load_skiboot(DriveInfo *dinfo, hwaddr addr, size_t max_size,
                                 Error **errp);

#endif /* _PPC_PNV_PNOR_H */
