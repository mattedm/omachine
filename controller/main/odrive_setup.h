// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * "Set up ODrive", started from the manipulator's menu: over CAN, write every setting in
 * odrive/odrive-s1-config.json (odrive_setup_table.h), run the motor and encoder calibration and
 * the anticogging calibration, put the operating settings back and save. The motor must be off
 * the linear rail: the anticogging calibration turns it about 40 turns each way.
 *
 * Nothing is saved until the end, so a cancel or a failure at any point reboots the ODrive
 * without saving and it comes back with its old settings. The controller restarts after the
 * setup, whatever the result, so the machine starts afresh (closed loop, homing).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Start the setup task (once, after the TWAI node is up)
void odrive_setup_init(void);

// machine_state_task: start a setup. False if one is already running.
bool odrive_setup_request(void);
// machine_state_task: stop the running setup (the ODrive keeps its old settings)
void odrive_setup_cancel(void);
// True from the request until the controller restarts
bool odrive_setup_active(void);

// The link task: the manipulator's setup_count it has passed on, and the status for its frame
void odrive_setup_ack(uint8_t setup_count);
void odrive_setup_status(uint8_t *ack, uint8_t *stage, uint8_t *progress);

// A float setting from odrive/odrive-s1-config.json (as the setup writes it), or fallback
float odrive_setup_config_float(const char *path, float fallback);

// twai_odrive_control.c: frames for the setup (Get_Version and TxSdo replies)
void odrive_setup_on_rx(uint32_t cmd, const uint8_t data[8]);

#ifdef __cplusplus
}
#endif
