// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// The dev board's RGB LED (WS2812-type) shows the machine state in the manipulator's button colours.
#pragma once

#include "machine_state.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the LED task; it follows ctx's state
void status_led_start(const machine_state_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
