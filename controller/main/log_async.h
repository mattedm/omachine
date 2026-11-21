// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Logging that never holds up the task that logs. ESP_LOGx output goes into a ring buffer, and a
// low-priority task writes it to the console. With the USB console connected, writing a line
// waits until the host has taken it, which would hold up the CAN and radio tasks. If the buffer
// is full a line is dropped, and the count is reported later.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void log_async_start(void);   // early in app_main; ESP_LOGx before it goes straight to the console

#ifdef __cplusplus
}
#endif
