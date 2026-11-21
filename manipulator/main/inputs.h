// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Rotary encoder and buttons, sampled every 1 ms by an esp_timer callback
#pragma once

#include <stdint.h>
#include "board_pins.h"

#define BUTTON_DEBOUNCE_MS 30
#define POWER_OFF_HOLD_MS 3000   // hold the main button this long to turn the board off
#define POWER_OFF_COUNTDOWN_MS 1000   // held this long it's no longer a press: the off countdown shows
#define POWER_ON_HOLD_MS 3000    // and this long to turn it on from the battery
#define MENU_HOLD_MS 3000        // hold the rotary button this long to open the menu
#define MENU_COUNTDOWN_MS 1000   // held this long it's no longer a press: the menu countdown shows

typedef struct {
  gpio_num_t pin;
  volatile bool pressed;      // debounced state
  uint8_t stableMs;           // how long the raw input has disagreed with 'pressed'
  volatile uint8_t presses;   // running count, incremented on each debounced release
  bool ignoreRelease;         // this press was for the power, so its release is not a press
  uint16_t heldMs;            // how long it has been pressed, up to POWER_OFF_HOLD_MS
  volatile uint8_t menuPresses;   // like 'presses', counting instead while the menu is open
} DebouncedButton;

// 1 flips the knob's direction (for an encoder whose A and B are the other way round on J7)
#define ROTARY_REVERSED 1

// Running input counters, sent in every reply (the controller differences them, so a lost
// reply loses no input). Each is written only by the 1 ms timer callback and read by link_task.
extern volatile uint8_t rotaryCount;
extern DebouncedButton autoButton;
extern DebouncedButton rotaryButton;

// Set by the timer callback when the main button has been held long enough; cleared by power.cpp
extern volatile bool powerOffRequested;

// True while the main button has been held past POWER_OFF_COUNTDOWN_MS on its way to turning the
// board off (autoButton.heldMs says how far); the display shows the countdown. Letting go before
// POWER_OFF_HOLD_MS clears it, and that press counts for nothing.
extern volatile bool powerOffCountdown;

// The menu, opened by holding the rotary button (only once menuAllowed is set, i.e. on the
// operating screen). menuCountdown is true while it's held past MENU_COUNTDOWN_MS on the way to
// MENU_HOLD_MS (rotaryButton.heldMs says how far); letting go before that clears it, and that
// press counts for nothing. At MENU_HOLD_MS menuOpen is set; the display closes it again. While
// it's open the knob turns menuRotaryCount instead of rotaryCount, and rotary button presses
// count in rotaryButton.menuPresses, so none of it reaches the controller.
extern volatile bool menuAllowed;
extern volatile bool menuCountdown;
extern volatile bool menuOpen;
extern volatile uint8_t menuRotaryCount;

// On the charging screen: the rotary is ignored, and the next press of the main button sets
// powerOnRequested (and clears standby) instead of counting as a press for the controller.
// Holding that press can't turn the board off again, like the press that turns it on from the battery.
extern volatile bool standby;
extern volatile bool powerOnRequested;

// Configure the pins and start the 1 ms sampling timer
void inputs_init();

typedef enum {
  BUTTON_LIGHT_OFF,
  BUTTON_LIGHT_RED,
  BUTTON_LIGHT_GREEN,
  BUTTON_LIGHT_BLUE,
  BUTTON_LIGHT_YELLOW,
} ButtonLightColor;

void setButtonLight(ButtonLightColor color);
