// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// GC9A01 round display: redraws from the latest elements when they change (core 0)
#pragma once

typedef enum {
  SCREEN_OPERATING,   // the controller's elements, link state and signal
  SCREEN_CHARGING,    // board "off" on USB: charge status and the board's own battery
  SCREEN_STARTING,    // the button was pressed on the charging screen; the handle is homing
  SCREEN_POWERING_ON, // from the battery: the button is being held, a bar counts down to on
  SCREEN_BLANK,       // let go too soon: black until the power goes
  SCREEN_HOMING_SHORT, // the end stops were found short of the full stroke: press to home again
  SCREEN_HOMING_FAILED, // no end stop found: the handle isn't on the rail; hold the button to turn off
  SCREEN_BATTERY_LOW, // battery too low to run (or to start): "plug in to charge", then off
  SCREEN_NO_LINK_OFF, // no link to the controller for LINK_IDLE_OFF_MS: turning off
} DisplayScreen;

// Which screen displayTask draws; set by app_main
extern volatile DisplayScreen displayScreen;

void displayTask(void *parameters);
