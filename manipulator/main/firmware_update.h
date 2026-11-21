// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Firmware updates from the menu: the Wi-Fi setup page, and "Update firmware", which fetches
// controller.bin and manipulator.bin from CONFIG_OM_UPDATE_URL (a GitHub release's download URL).
//
// An update first asks the controller for its version (omachine_update_protocol.h, which works
// across protocol versions). If controller.bin is a different version, it's downloaded into this
// board's spare app slot and relayed to the controller over ESP-NOW; once the controller is back
// on the new version, manipulator.bin is installed here and the board restarts into it. Either
// board's new image then boots on trial (firmware_valid.c). The controller goes first, so that if
// anything fails the two boards are left able to talk and the update can simply be run again.
#pragma once

#include <stddef.h>
#include <stdint.h>

// From link_task: after Wi-Fi and ESP-NOW are set up, and for every update frame received
void firmwareUpdateInit();
void firmwareUpdateOnRecv(const uint8_t *mac, const uint8_t *data, int len);

// The home Wi-Fi network the update joins, kept in NVS ("wifi")
bool wifiStoredSsid(char *ssid, size_t len);

// Wi-Fi setup: this board opens its own network, WIFI_SETUP_SSID, with a page at WIFI_SETUP_ADDRESS
// to type in the home network's name and password
#define WIFI_SETUP_SSID    "omachine-setup"
#define WIFI_SETUP_ADDRESS "192.168.4.1"
typedef enum { WIFI_SETUP_OFF, WIFI_SETUP_RUNNING, WIFI_SETUP_SAVED } WifiSetupState;
extern volatile WifiSetupState wifiSetupState;
void wifiSetupStart();
void wifiSetupStop();

typedef enum { UPDATE_IDLE, UPDATE_RUNNING, UPDATE_DONE, UPDATE_FAILED } UpdateState;
extern volatile UpdateState updateState;
extern volatile bool updateInProgress;   // the motor stays off (the radio is away from the link)
extern volatile int updateProgress;      // 0..100 for the current step, or -1
extern volatile uint32_t updateStepCount; // changes whenever updateStep() does
const char *updateStep();                // what it's doing now
const char *updateResultTitle();         // once DONE or FAILED
const char *updateResultLine();
void firmwareUpdateStart();
