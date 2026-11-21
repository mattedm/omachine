// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// The motor's sensor alignment (zero electric angle and sensor direction), kept in NVS so the
// handle doesn't have to align at every start. SimpleFOC's alignment turns the motor a little each
// way, which fails if the handle is against an end stop.
//
// With nothing stored, motorTask aligns wherever the handle is (as before), homes, then aligns
// again at the middle of the travel and stores that. With values stored it skips the alignment;
// if homing then fails, it aligns afresh and tries once more, in case the encoder or magnet has
// moved since. The menu's "Calibrate motor" redoes the middle-of-travel alignment while running.
// Kept out of motor_task.h so display_task can include it (SimpleFOC's headers clash there).
#pragma once

#include <stdint.h>

typedef enum {
  MOTOR_CAL_IDLE,
  MOTOR_CAL_REQUESTED,   // set by the menu; motorTask picks it up
  MOTOR_CAL_RUNNING,     // the handle is moving to the middle and aligning; the link holds its position
  MOTOR_CAL_DONE,        // stored
  MOTOR_CAL_FAILED,      // the old alignment is kept (handle held, or the result didn't check out)
} MotorCalState;

extern volatile MotorCalState motorCalState;
