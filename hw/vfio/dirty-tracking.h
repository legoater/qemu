/*
 * VFIO dirty page tracking routines
 *
 * Copyright Red Hat, Inc. 2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_VFIO_DIRTY_TRACKING_H
#define HW_VFIO_DIRTY_TRACKING_H

extern const MemoryListener vfio_memory_listener;

bool vfio_devices_all_dirty_tracking_started(const VFIOContainerBase *bcontainer);
bool vfio_devices_all_device_dirty_tracking(const VFIOContainerBase *bcontainer);
int vfio_devices_query_dirty_bitmap(const VFIOContainerBase *bcontainer,
                                    VFIOBitmap *vbmap, hwaddr iova, hwaddr size,
                                    Error **errp);
int vfio_get_dirty_bitmap(const VFIOContainerBase *bcontainer, uint64_t iova,
                          uint64_t size, ram_addr_t ram_addr, Error **errp);

#endif /* HW_VFIO_DIRTY_TRACKING_H */
