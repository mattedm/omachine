// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// SimpleFOC haptic handle: finds the end stops, then follows the ODrive position (core 1)
#pragma once

#include "esp_simplefoc.h"

// Which magnetic encoder breakout is on J5: 0 for the AS5048A board, 1 for the MT6701 board.
// Both use the same cable and pins.
#define ENCODER_MT6701 0

#if ENCODER_MT6701
#include "fast_mt6701.h"
using EncoderSensor = FastMT6701;
#else
#include "fast_as5048a.h"
using EncoderSensor = FastAS5048A;
#endif

extern EncoderSensor sensor;
extern BLDCMotor motor;

// Handle travel found at startup, in shaft radians
extern float endlimit;
extern float startlimit;
extern float midpoint;

extern volatile float odriveposition;   // written by link_task, read by motorTask on core 1
extern volatile uint32_t motorLoopCount;

// Set when motorTask gave up finding the end stops (the motor turned HOMING_MAX_TURNS without
// stopping); it then notifies as usual, with the motor off, and ends
extern volatile bool homingFailed;
// With homingFailed: the handle didn't move at all (rather than no end stop being found)
extern volatile bool homingStalled;
// Set when the end stops were found but less than HOMING_SHORT_PCT of the full stroke apart
// (homingTravelPct says how much): motorTask notifies as usual with the motor off, then waits for
// a notification of its own to home again
extern volatile bool homingShort;
extern volatile uint8_t homingTravelPct;

// Tell the driver what VM is, so its duty cycles give the commanded voltages as the battery runs down
void motorSetSupplyVolts(float volts);

// Sets up the motor and finds the end stops, notifies the task passed as the parameter
// (xTaskNotifyGive) once that is done, then runs the control loop forever. If no end stop is found
// it sets homingFailed, notifies, and ends with the motor off.
void motorTask(void *parameters);
