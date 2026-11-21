// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Soft power latch, charging status and motor supply monitoring.
//
// The button or USB turns the regulator on; from the battery, the firmware must then hold it on
// with POWER_HOLD_PIN or the board dies when the button is released. It only does that once the
// button has been held for POWER_ON_HOLD_MS, so a knock, a bounce, or the restart after turning
// off can't turn the board on. Dropping that pin is how the
// board turns off. USB keeps the regulator on regardless, so "off" while plugged in is the
// charging screen instead: the motor and the link are stopped, and only the display runs.
#pragma once

#include "display_task.h"
#include <stdint.h>

// Diagnostic: 1 logs the motor's peak phase current from the DRV8311's current sense outputs
#define MOTOR_CURRENT_LOG 0

typedef enum {
  CHARGE_NOT_ON_USB,
  CHARGE_CHARGING,
  CHARGE_DONE,
  CHARGE_NO_BATTERY,
} ChargeState;

// Sets up the power pins and the supply reading. Call first thing in app_main.
void power_init();

// From the battery, latches the power on once the button has been held for POWER_ON_HOLD_MS
// (counted from boot), lights the button blue and returns true. Returns false if the button is let
// go sooner (or wasn't pressed at all): the caller should then call power_off_now. On USB, returns
// true at once. Call after inputs_init.
bool power_wait_for_on();

// True while USB power is valid (the BQ24075's PGOOD)
bool power_on_usb();

// Holds the regulator on from the battery (true), or lets it go (false), so that on the charging
// screen unplugging USB turns the board off
void power_set_hold(bool on);

// Turns the board off from the battery: lets go of the latch, and stops the ESP32's USB from
// holding the regulator on through VBUS. Never returns. If the power doesn't go, blinks the button
// light (red: button line released, yellow: still pressed) and then sleeps until USB is plugged in.
void power_off_now();

ChargeState power_charge_state();

// Battery charge estimated from its voltage, 0-100. Reads a little high while charging.
uint8_t power_battery_percent();

// Running from the battery, the board turns off once the battery has stayed below
// BATTERY_CUTOFF_VOLTS for BATTERY_CUTOFF_MS, after showing SCREEN_BATTERY_LOW. Turned on from
// the battery, it won't start below BATTERY_START_VOLTS (the cell recovers a little once the load
// is off, so it needs some margin over the cut-off). On USB neither applies.
#define BATTERY_CUTOFF_VOLTS 3.2f
#define BATTERY_CUTOFF_MS    5000
#define BATTERY_START_VOLTS  3.5f

// Battery too low to start (only from the battery; a fresh reading)
bool power_battery_too_low_to_start();

// True once a shutdown with a message (low battery, no link) has begun: the motor stays off
bool power_shutting_down();

// Asks the power task to turn the board off, showing 'screen' (which says why) first: motor off,
// the message for a few seconds, then off (on USB, a restart onto the charging screen)
void power_request_off(DisplayScreen screen);

// Most phase current the motor may use with no battery on J9, running from USB alone. The charger
// (U2) passes only about 1 A from USB and has nothing to make up a peak above that, so VSYS and
// 3V3 collapse (brownout reset) if the motor draws more.
#define NO_BATTERY_MOTOR_AMPS 0.35f

// The motor current limit to use: 'amps', or NO_BATTERY_MOTOR_AMPS if lower while there is no battery
float power_motor_amps(float amps);

// True while the motor is held to NO_BATTERY_MOTOR_AMPS
bool power_motor_limited();

// Motor supply (VM) right now, in volts
float power_motor_supply_volts();

// Starts the task that keeps the motor's supply voltage and the battery reading up to date,
// reports driver faults, and turns the board off when the button asks for it (powerOffRequested
// in inputs.h). On USB, off restarts the board onto the charging screen.
void power_start_task();
