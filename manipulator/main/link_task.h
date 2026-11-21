// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// ESP-NOW link to the controller: frame layouts in omachine_protocol.h. The controller polls
// with the ODrive position and every element; we answer each poll.
#pragma once

#include <stdint.h>
#include "omachine_protocol.h"   // frame layouts, shared with omachine-controller (keep identical)

//Power, Speed, Length, StrokeIndex, ActiveElementIndex, MachineStateIndex, BatteryPercent (OM_ELEMENT_*)
// Written by linkTask from every controller frame; read by the motor and display tasks.
extern uint8_t elements[OM_ELEMENT_COUNT];

extern volatile bool linkUp;
extern volatile int8_t linkRssi;   // weakest RSSI (dBm) of the controller's polls in the last second
extern volatile uint8_t powerCount;   // running Power click count, turned by the menu's Power page (sent as power_count)
extern volatile uint8_t saveCount;    // running count of the menu's "Save settings" requests (sent as save_count)
extern volatile uint8_t controllerSavedCount;   // the controller's saved_count: changes once it has saved
// "Set up ODrive" from the menu: running counts sent as setup_count / setup_cancel_count, and the
// controller's answer (setup_ack: the last setup_count it acted on; OM_SETUP_* stage; 0..100)
extern volatile uint8_t setupCount;
extern volatile uint8_t setupCancelCount;
extern volatile uint8_t controllerSetupAck;
extern volatile uint8_t controllerSetupStage;
extern volatile uint8_t controllerSetupProgress;
// The ODrive's error from the controller: OM_ODRIVE_ERROR_NONE or its bit number (OM_STATE_ODRIVE_FAULT)
extern volatile uint8_t controllerOdriveError;

// Set to 1 to log the motor loop rate and link timing (polls, reply delay, send results) once a second.
#define LOOP_RATE_DEBUG 0

// Puts the radio back the way the link needs it (channel, protocols incl. Long Range, no power
// save, TX power). Changing the Wi-Fi mode or joining a network can reset these, after which the
// controller's Long Range polls no longer get through; the firmware update calls this after each.
void linkRadioRestore();

void linkTask(void *parameters);
