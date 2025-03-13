/*
 * VFIO migration interface
 *
 * Copyright Red Hat, Inc. 2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_VFIO_VFIO_MIGRATION_H
#define HW_VFIO_VFIO_MIGRATION_H

bool vfio_mig_active(void);
int64_t vfio_mig_bytes_transferred(void);
void vfio_mig_reset_bytes_transferred(void);
void vfio_mig_add_bytes_transferred(unsigned long val);

#endif /* HW_VFIO_VFIO_MIGRATION_H */
