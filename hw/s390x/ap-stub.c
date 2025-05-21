/*
 * VFIO based AP matrix device assignment
 *
 * Copyright 2025 IBM Corp.
 * Author(s): Rorie Reyes <rreyes@linux.ibm.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/s390x/ap-bridge.h"

/**
 * @brief Stub implementation for retrieving an AP matrix device event.
 *
 * This function does not process or retrieve any event and always returns 0.
 *
 * @param res Unused pointer for event result data.
 * @return int Always returns 0, indicating no event.
 */
int ap_chsc_sei_nt0_get_event(void *res)
{
    return 0;
}

/**
 * @brief Indicates whether an AP matrix device event is present.
 *
 * @return int Always returns 0, indicating no event is available.
 */
int ap_chsc_sei_nt0_have_event(void)
{
    return 0;
}
