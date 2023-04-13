/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2019 IBM Corp.
 *
 * IBM On-chip Peripheral Bus
 */
#ifndef FSI_ENGINE_SCRATCHPAD_H
#define FSI_ENGINE_SCRATCHPAD_H

#include "hw/fsi/lbus.h"

#define TYPE_SCRATCHPAD "scratchpad"
#define SCRATCHPAD(obj) OBJECT_CHECK(ScratchPad, (obj), TYPE_SCRATCHPAD)

typedef struct ScratchPad {
        LBusDevice parent;

        uint32_t reg;
} ScratchPad;

#endif /* FSI_ENGINE_SCRATCHPAD_H */
