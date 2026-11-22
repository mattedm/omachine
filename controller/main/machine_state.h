// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "omachine_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTON_ROTARY = 0,
    BUTTON_ROTARY_BUTTON,
    BUTTON_POWER,               // Power clicks from the manipulator's menu (rotary_delta)
    BUTTON_SAVE_SETTINGS,       // the manipulator's menu: store Power, Speed, Length and Stroke in NVS
    BUTTON_ODRIVE_SETUP,        // the manipulator's menu: "Set up ODrive" (see odrive_setup.h)
    BUTTON_ODRIVE_SETUP_CANCEL, // ... and stop it
    BUTTON_AUTO_BUTTON,
    BUTTON_CONTROLLER_DISCONNECTED,
    BUTTON_CONTROLLER_CONNECTED,
    // Requests from the ODrive control loop. Only machine_state_task changes the state;
    // it applies these only if the machine is still in the state the request is for.
    BUTTON_HOMING_STARTED,      // INITIAL -> HOMING
    BUTTON_HOMING_DONE,         // HOMING -> SYNCING
    BUTTON_ODRIVE_SYNCED,       // PREAUTO -> AUTO: ODrive has reached the handle
    BUTTON_HANDLE_SYNCED,       // SYNCING -> MANUAL: handle has reached the held ODrive
    BUTTON_ODRIVE_FAULT,        // any state -> ODRIVE_FAULT: the ODrive reports an error or left closed loop
    BUTTON_ODRIVE_RECOVERED,    // ODRIVE_FAULT -> SYNCING (or INITIAL, if it failed while starting up)
} button_type_t;

typedef struct {
    button_type_t type;
    int8_t rotary_delta; // Only used when type is BUTTON_ROTARY or BUTTON_POWER
} button_event_t;

typedef enum {
    // Values are sent to the manipulator, so they come from omachine_protocol.h
    MACHINE_INITIAL      = OM_STATE_INITIAL,
    MACHINE_HOMING       = OM_STATE_HOMING,
    MACHINE_SYNCING      = OM_STATE_SYNCING,    // after homing, stopping AUTO, a cancelled PREAUTO or a link loss: handle returning to the held ODrive
    MACHINE_MANUAL       = OM_STATE_MANUAL,
    MACHINE_AUTO         = OM_STATE_AUTO,
    MACHINE_PREAUTO      = OM_STATE_PREAUTO,
    MACHINE_DISCONNECTED = OM_STATE_DISCONNECTED,
    MACHINE_ODRIVE_SETUP = OM_STATE_ODRIVE_SETUP,  // odrive_setup.c has the ODrive; the controller restarts after
    MACHINE_ODRIVE_FAULT = OM_STATE_ODRIVE_FAULT,  // no setpoints; twai_odrive_control.c tries to bring it back
} machine_state_t;

typedef enum {
    ELEMENT_POWER = 0,
    ELEMENT_SPEED,
    ELEMENT_LENGTH,
    ELEMENT_STROKE_INDEX,
    ELEMENT_ACTIVE_INDEX,
    ELEMENT_STATE_INDEX,
    ELEMENT_VBUS,
    ELEMENT_COUNT
} machine_element_key_t;

typedef struct {
    const char *name;
    bool screen_element;
    int32_t current_value;
    int32_t max_value;
    bool rollaround;
    bool controller_update_needed;
    bool odrive_update_needed;
} machine_element_t;

typedef struct {
    machine_element_key_t key;
    int32_t value;
} element_update_t;

typedef enum {
    STROKE_JACK = 0,
    STROKE_PULSE,
    STROKE_PUMP,
    STROKE_CLASSIC,
    STROKE_TEASE,
    STROKE_CRESCENDO,
    STROKE_SLOW_IN,
    STROKE_HEARTBEAT,
    STROKE_BREATHING,
    STROKE_WANDER,
    STROKE_PENDULUM,
    STROKE_BOUNCE,
    STROKE_SPRING,
    STROKE_LORENZ,
    STROKE_GALLOP,
    STROKE_SWING,
    STROKE_RAMP_UP,
    STROKE_FLUTTER,
    STROKE_WAVES,
    STROKE_COUNT
} stroke_id_t;

typedef struct {
    QueueHandle_t buttons_queue;
    volatile machine_state_t state;  // written only by machine_state_task, read by other tasks
} machine_state_ctx_t;

extern machine_element_t g_machine_elements[ELEMENT_COUNT];

void machine_state_init_defaults(machine_state_ctx_t *ctx);
void machine_state_set(machine_state_ctx_t *ctx, machine_state_t new_state);  // machine_state_task only
machine_state_t machine_state_get(const machine_state_ctx_t *ctx);
int32_t machine_element_value(machine_element_key_t key);  // safe from any task
bool machine_apply_button(machine_state_ctx_t *ctx, const button_event_t *ev);
void machine_state_task(void *arg);

float adjusted_pot_pos(float actual_pos);
float stroke_eval(stroke_id_t stroke, uint32_t millis, float length);
float inter_point(float xa, float xb, float ya, float yb, float xc);

// Power, Speed, Length and Stroke kept in NVS ("settings"), so they're the defaults at start-up.
// Load before the tasks start (it initialises NVS); a save bumps machine_settings_saved_count, which
// goes to the manipulator in every frame so its menu can confirm the save.
void machine_settings_load(void);

// twai_odrive_control.c: the main button in ODRIVE_FAULT asks for another try (errors cleared, closed loop)
void odrive_fault_retry(void);
// twai_odrive_control.c: the fault for the manipulator: the ODrive's active errors and disarm reason
// (both 0 when there's none), its bus voltage in 0.1 V, and OM_FAULT_* flags
void odrive_fault_report(uint32_t *errors, uint32_t *disarm, uint16_t *vbus_dv, uint8_t *flags);
extern volatile uint8_t machine_settings_saved_count;

#ifdef __cplusplus
}
#endif

#endif // MACHINE_STATE_H
