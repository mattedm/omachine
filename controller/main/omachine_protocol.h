// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * omachine controller <-> manipulator ESP-NOW protocol.
 *
 * KEEP THIS FILE IDENTICAL in both projects:
 *   controller/main/omachine_protocol.h
 *   manipulator/main/omachine_protocol.h
 *
 * One ESP-NOW packet = one frame. The controller polls; the manipulator answers each poll.
 * Both frames carry complete state rather than events, so a lost packet is simply made
 * good by the next one:
 *   - the controller sends every display element in every frame
 *   - the manipulator sends running counters (rotary clicks, button presses) that the
 *     controller turns into events by differencing, so lost replies lose no input
 *
 * Multi-byte fields are little-endian (both chips are).
 */

#ifndef OMACHINE_PROTOCOL_H
#define OMACHINE_PROTOCOL_H

#include <stdint.h>

#define OM_PROTOCOL_VERSION   8      // bump on any layout change; frames with another version are ignored
#define OM_ELEMENT_COUNT      7      // must equal ELEMENT_COUNT in machine_state.h and elements[] in omachine-manipulator
#define OM_POS_MAX            32767  // positions are scaled 0..OM_POS_MAX across the stroke
#define OM_SYNC_TOLERANCE     0.02f  // handle and ODrive count as matched within this share of the stroke

// Element indexes (match machine_element_key_t in machine_state.h)
#define OM_ELEMENT_POWER          0
#define OM_ELEMENT_SPEED          1
#define OM_ELEMENT_LENGTH         2
#define OM_ELEMENT_STROKE_INDEX   3
#define OM_ELEMENT_ACTIVE_INDEX   4
#define OM_ELEMENT_STATE_INDEX    5
#define OM_ELEMENT_BATTERY        6
// elements[OM_ELEMENT_BATTERY] until the ODrive has reported its bus voltage: not shown
#define OM_BATTERY_UNKNOWN        255

// Machine states, sent in elements[OM_ELEMENT_STATE_INDEX] (machine_state_t in machine_state.h)
#define OM_STATE_INITIAL      0
#define OM_STATE_HOMING       1
#define OM_STATE_SYNCING      2      // ODrive held while the handle comes over to it, then MANUAL
#define OM_STATE_MANUAL       3
#define OM_STATE_AUTO         4
#define OM_STATE_PREAUTO      5      // ODrive moving onto the handle before strokes start
#define OM_STATE_DISCONNECTED 6
#define OM_STATE_ODRIVE_SETUP 7      // the menu's "Set up ODrive" is running; the controller restarts after it
#define OM_STATE_ODRIVE_FAULT 8      // the ODrive reports an error or left closed loop; odrive_* say why.
                                     // The main button retries; once it runs again, SYNCING (or homing)
#define OM_STATE_COUNT        9

// odrive_errors / odrive_disarm: the ODrive's active errors and the errors that disarmed it
// (axis0.active_errors and axis0.disarm_reason, both ODriveError bits in firmware 0.6), from its
// Get_Error reply. Both 0 in OM_STATE_ODRIVE_FAULT: it isn't running but reports no error.
#define OM_FAULT_RETRYING         0x01  // fault_flags: the controller is still retrying by itself
// odrive_vbus_dv: the ODrive's latest bus voltage in tenths of a volt, 0 until it has reported one

// "Set up ODrive" stages, sent in setup_stage. Running stages, then one of the results, which the
// controller holds for OM_SETUP_RESULT_HOLD_MS before it restarts.
#define OM_SETUP_IDLE             0
#define OM_SETUP_CHECKING         1   // ODrive firmware version and CAN settings
#define OM_SETUP_CONFIGURING      2   // writing every setting from the controller's ODrive config
#define OM_SETUP_CALIBRATING      3   // motor and encoder calibration
#define OM_SETUP_ANTICOGGING      4   // anticogging calibration (about 10 minutes)
#define OM_SETUP_SAVING           5   // operating settings back, save, ODrive reboots
#define OM_SETUP_DONE             6   // saved
#define OM_SETUP_CANCELLED        7   // stopped from the menu; the ODrive keeps its old settings
#define OM_SETUP_FAILED_NO_ODRIVE 8   // no answer from the ODrive over CAN
#define OM_SETUP_FAILED_VERSION   9   // ODrive firmware isn't the one the settings table is for
#define OM_SETUP_FAILED_CONFIG    10  // a setting didn't write
#define OM_SETUP_FAILED_CALIBRATE 11  // motor or encoder calibration reported an error
#define OM_SETUP_FAILED_STALL     12  // the motor stopped turning during anticogging (still on the rail?)
#define OM_SETUP_FAILED_ANTICOG   13  // anticogging calibration reported an error or ran too long
#define OM_SETUP_FAILED_SAVE      14  // the ODrive didn't come back after saving
#define OM_SETUP_FIRST_RESULT     OM_SETUP_DONE
#define OM_SETUP_RESULT_HOLD_MS   5000

// Manipulator status flags (informational; events come from the press counters)
#define OM_FLAG_AUTO_HELD     0x01   // main (auto) button currently held
#define OM_FLAG_ROTARY_HELD   0x02   // rotary push button currently held

// controller -> manipulator, 27 bytes
typedef struct __attribute__((packed)) {
    uint8_t  version;                         // OM_PROTOCOL_VERSION
    uint8_t  seq;                             // increments every frame
    uint16_t odrive_pos;                      // ODrive position, 0..OM_POS_MAX
    uint8_t  elements[OM_ELEMENT_COUNT];      // current value of every display element
    uint8_t  flags;                           // reserved, 0
    uint8_t  saved_count;                     // running count of settings saves done (wraps); answers save_count
    uint8_t  setup_ack;                       // the last setup_count the controller has acted on
    uint8_t  setup_stage;                     // OM_SETUP_*
    uint8_t  setup_progress;                  // 0..100 through the current stage
    uint32_t odrive_errors;                   // ODrive active errors (in OM_STATE_ODRIVE_FAULT; else 0)
    uint32_t odrive_disarm;                   // ODrive disarm reason (in OM_STATE_ODRIVE_FAULT; else 0)
    uint16_t odrive_vbus_dv;                  // ODrive bus voltage, 0.1 V
    uint8_t  fault_flags;                     // OM_FAULT_*
} om_controller_frame_t;

// manipulator -> controller, 12 bytes
typedef struct __attribute__((packed)) {
    uint8_t  version;                         // OM_PROTOCOL_VERSION
    uint8_t  seq;                             // increments every frame
    uint16_t encoder_pos;                     // handle position, 0..OM_POS_MAX
    uint8_t  rotary_count;                    // running rotary click count (wraps; signed difference = clicks), for the active element
    uint8_t  auto_presses;                    // running count of debounced main-button presses (wraps)
    uint8_t  rotary_presses;                  // running count of debounced rotary-button presses (wraps)
    uint8_t  flags;                           // OM_FLAG_*
    uint8_t  power_count;                     // running Power click count from the manipulator's menu (wraps, like rotary_count)
    uint8_t  save_count;                      // running count of "Save settings" requests from the menu (wraps)
    uint8_t  setup_count;                     // running count of "Set up ODrive" start requests (wraps)
    uint8_t  setup_cancel_count;              // running count of requests to stop it (wraps)
} om_manipulator_frame_t;

#ifdef __cplusplus
static_assert(sizeof(om_controller_frame_t) == 27, "controller frame layout");
static_assert(sizeof(om_manipulator_frame_t) == 12, "manipulator frame layout");
#else
_Static_assert(sizeof(om_controller_frame_t) == 27, "controller frame layout");
_Static_assert(sizeof(om_manipulator_frame_t) == 12, "manipulator frame layout");
#endif

#endif // OMACHINE_PROTOCOL_H
