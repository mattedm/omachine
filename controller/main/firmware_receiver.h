// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Receives a new firmware image from the manipulator over ESP-NOW (omachine_update_protocol.h)
// and installs it in the spare app slot. The new image then boots on trial (firmware_valid.c).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Set while an update is being received: the link stops polling the manipulator, and the ODrive
// has been put in IDLE
extern volatile bool g_firmware_update_active;

void firmware_receiver_init(void);
// From the ESP-NOW receive callback (Wi-Fi task) for frames starting with OM_UPDATE_MAGIC
void firmware_receiver_on_recv(const uint8_t *mac, const uint8_t *data, int len);

// Provided by twai_odrive_control.c: put the ODrive in IDLE before the flash writes start
void odrive_idle_for_update(void);

#ifdef __cplusplus
}
#endif
