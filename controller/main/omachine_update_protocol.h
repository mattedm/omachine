// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Firmware update over ESP-NOW: the manipulator relays the controller's new image to it.
 *
 * KEEP THIS FILE IDENTICAL in both projects, and NEVER CHANGE THE LAYOUT: unlike the frames in
 * omachine_protocol.h (which change with OM_PROTOCOL_VERSION), these must work between any two
 * firmware versions, so that either board can always be brought up to date.
 *
 * Every frame starts with om_update_header_t; frames are told apart from the normal link frames by
 * their first byte, OM_UPDATE_MAGIC (never a protocol version).
 *
 *   manipulator                         controller
 *   HELLO (broadcast)            ->
 *                                <-     HELLO_ACK  + om_update_hello_t (project name, version)
 *   BEGIN  length = image size   ->     stops the ODrive (IDLE), starts writing its spare slot
 *                                <-     ACK        offset 0, status
 *   DATA   offset, length + data ->     written if offset is the next byte expected
 *                                <-     ACK        offset = next byte expected, status
 *   ...                                 (the manipulator always carries on from the ACK's offset)
 *   END    length = image size   ->     checks the image, makes it the boot image
 *                                <-     ACK        status, then restarts into it
 *   ABORT                        ->     drops the update and restarts on the old image
 *
 * Each step is stop-and-wait: the manipulator resends until it gets the ACK.
 * Multi-byte fields are little-endian (both chips are).
 */

#ifndef OMACHINE_UPDATE_PROTOCOL_H
#define OMACHINE_UPDATE_PROTOCOL_H

#include <stdint.h>

#define OM_UPDATE_MAGIC   0xA5   // first byte of every update frame
#define OM_UPDATE_CHUNK   1024   // image bytes per DATA frame (ESP-NOW v2 carries up to 1470)

enum {
    OM_UPD_HELLO = 1,
    OM_UPD_HELLO_ACK,
    OM_UPD_BEGIN,
    OM_UPD_DATA,
    OM_UPD_END,
    OM_UPD_ABORT,
    OM_UPD_ACK,
};

// ACK status
#define OM_UPD_OK         0
#define OM_UPD_ERR_STATE  1   // DATA or END without a BEGIN
#define OM_UPD_ERR_FLASH  2   // couldn't start or write the update
#define OM_UPD_ERR_IMAGE  3   // the image didn't check out (size, checksum, or not a controller image)

typedef struct __attribute__((packed)) {
    uint8_t  magic;      // OM_UPDATE_MAGIC
    uint8_t  type;       // OM_UPD_*
    uint8_t  status;     // ACK: OM_UPD_OK or OM_UPD_ERR_*
    uint8_t  reserved;
    uint32_t offset;     // DATA: where the data goes; ACK: next byte expected
    uint32_t length;     // BEGIN/END: image size; DATA: bytes of data that follow
} om_update_header_t;

typedef struct __attribute__((packed)) {
    char project[32];    // esp_app_desc_t project_name, e.g. "omachine-controller"
    char version[32];    // esp_app_desc_t version (git describe)
} om_update_hello_t;

#ifdef __cplusplus
static_assert(sizeof(om_update_header_t) == 12, "update header layout");
static_assert(sizeof(om_update_hello_t) == 64, "update hello layout");
#else
_Static_assert(sizeof(om_update_header_t) == 12, "update header layout");
_Static_assert(sizeof(om_update_hello_t) == 64, "update hello layout");
#endif

#endif // OMACHINE_UPDATE_PROTOCOL_H
