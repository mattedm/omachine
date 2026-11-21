// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Firmware rollback. An image installed by an over-the-air update boots "pending verify"; unless
// it is marked good before the next reboot, the bootloader goes back to the previous image. It is
// marked good once the link to the other board comes up, or after FIRMWARE_VALID_AFTER_MS of
// running without a crash (the other board may be off, or this one on its charging screen).
// Images flashed over USB are never pending, so none of this applies to them.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void firmware_valid_start(void);              // early in app_main: logs the running slot, starts the fallback timer
void firmware_mark_valid(const char *reason); // cheap to call often; acts only on a pending image

#ifdef __cplusplus
}
#endif
