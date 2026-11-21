// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "machine_state.h"
#include "odrive_setup.h"
#include "stroke_pendulum_table.h"

#include <math.h>
#include <string.h>

#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#define NUMBER_OF_DISPLAY_ELEMENTS 4
#define FIRST_KNOB_ELEMENT ELEMENT_SPEED   // the rotary button cycles Speed..Stroke; Power has its own menu page
#define POT_MAX 32767.0f

static const char *TAG = "machine-state";

typedef struct {
    float adjust_pos;
    float redirect_pos;
} adjust_position_t;

static adjust_position_t s_linpot_adjust = {0.0f, 0.0f};

static float stroke_classic(uint32_t millis, float length)
{
    return length * ((cosf((millis / 1000.0f) * (float)M_PI) - 1.0f) / -2.0f);
}

static float stroke_pulse(uint32_t millis, float length)
{
    float distance = (float)(millis % 2000u);
    if (distance >= 1000.0f) {
        distance = 2000.0f - distance;
    }
    distance = distance / 1000.0f;
    distance = (0.7f * distance) + (0.3f * ((cosf((millis / 107.0f) * (float)M_PI) - 1.0f) / -2.0f));
    return length * distance;
}

static float stroke_pump(uint32_t millis, float length)
{
    float distance = (float)(millis % 3000u);
    if (distance < 2000.0f) {
        distance = distance / 2000.0f;
    } else {
        const float factor = 160.0f;
        distance = (((-(powf(2.0f, (distance - 2000.0f) / factor)) + 1.0f) /
                    (powf(2.0f, 1000.0f / factor) - 1.0f)) + 1.0f);
    }
    return length * distance;
}

static float stroke_jack(uint32_t millis, float length)
{
    float distance = (float)(millis % 2000u);
    if (distance >= 1000.0f) {
        distance = 2000.0f - distance;
    }
    distance = distance / 1000.0f;
    return length * distance;
}

// One cosine stroke out to depth (0..1) and back, phase 0..1
static float cosine_stroke(float phase, float depth)
{
    return depth * (1.0f - cosf(phase * 2.0f * (float)M_PI)) / 2.0f;
}

// Three shallow strokes, then one full one
static float stroke_tease(uint32_t millis, float length)
{
    uint32_t cycle = millis % 4000u;
    float depth = (cycle < 3000u) ? 0.3f : 1.0f;
    return length * cosine_stroke((cycle % 1000u) / 1000.0f, depth);
}

// Each stroke deeper than the last, 20% to 100% over 8 strokes, then shallow again
static float stroke_crescendo(uint32_t millis, float length)
{
    uint32_t n = (millis / 1000u) % 8u;
    float depth = 0.2f + 0.8f * n / 7.0f;
    return length * cosine_stroke((millis % 1000u) / 1000.0f, depth);
}

// Slow out to full (1.2 s), quick return (0.3 s)
static float stroke_slow_in(uint32_t millis, float length)
{
    uint32_t t = millis % 1500u;
    float d = (t < 1200u) ? (1.0f - cosf((float)M_PI * t / 1200.0f)) / 2.0f
                          : (1.0f + cosf((float)M_PI * (t - 1200u) / 300.0f)) / 2.0f;
    return length * d;
}

// A strong beat, a softer one, then a rest
static float stroke_heartbeat(uint32_t millis, float length)
{
    uint32_t t = millis % 1600u;
    if (t < 350u) {
        return length * cosine_stroke(t / 350.0f, 1.0f);
    }
    if (t < 700u) {
        return length * cosine_stroke((t - 350u) / 350.0f, 0.6f);
    }
    return 0.0f;
}

// 1 s strokes whose depth swells from 30% to 100% and back over 10 s
static float stroke_breathing(uint32_t millis, float length)
{
    float depth = 0.3f + 0.7f * (1.0f - cosf(2.0f * (float)M_PI * millis / 10000.0f)) / 2.0f;
    return length * depth * (1.0f - cosf((float)M_PI * millis / 500.0f)) / 2.0f;
}

// Three smooth waves with unrelated periods, so the pattern never quite repeats. Each term runs
// 0..weight (weights sum to 1) and starts at 0, so there is no jump when AUTO starts.
static float stroke_wander(uint32_t millis, float length)
{
    float t = millis / 1000.0f;
    float d = 0.6f * (1.0f - cosf(2.0f * (float)M_PI * t / 1.3f)) / 2.0f
            + 0.3f * (1.0f - cosf(2.0f * (float)M_PI * t / 2.9f)) / 2.0f
            + 0.1f * (1.0f - cosf(2.0f * (float)M_PI * t / 0.7f)) / 2.0f;
    return length * d;
}

// A double pendulum's lower bob, swinging side to side: irregular swings and flicks, as a seamless
// 20 s loop from tools/gen_pendulum_stroke.py, smoothly interpolated (Catmull-Rom) between samples
static float stroke_pendulum(uint32_t millis, float length)
{
    const uint32_t loop_ms = PENDULUM_SAMPLES * 1000u / PENDULUM_RATE_HZ;
    float pos = (float)(millis % loop_ms) * PENDULUM_RATE_HZ / 1000.0f;
    int i = (int)pos;
    float f = pos - (float)i;
    float p0 = pendulum_samples[(i + PENDULUM_SAMPLES - 1) % PENDULUM_SAMPLES];
    float p1 = pendulum_samples[i % PENDULUM_SAMPLES];
    float p2 = pendulum_samples[(i + 1) % PENDULUM_SAMPLES];
    float p3 = pendulum_samples[(i + 2) % PENDULUM_SAMPLES];
    float d = 0.5f * (2.0f * p1 + (p2 - p0) * f + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * f * f +
                      (3.0f * (p1 - p2) + p3 - p0) * f * f * f) / 32767.0f;
    return length * fminf(fmaxf(d, 0.0f), 1.0f);   // the curve can overshoot a sample a little
}

typedef float (*stroke_fn_t)(uint32_t millis, float length);

typedef struct {
    const char *name;
    stroke_fn_t fn;
} stroke_entry_t;

static const stroke_entry_t s_strokes[STROKE_COUNT] = {
    [STROKE_JACK] = {"Jack", stroke_jack},
    [STROKE_PULSE] = {"Pulse", stroke_pulse},
    [STROKE_PUMP] = {"Pump", stroke_pump},
    [STROKE_CLASSIC] = {"Classic", stroke_classic},
    [STROKE_TEASE] = {"Tease", stroke_tease},
    [STROKE_CRESCENDO] = {"Crescendo", stroke_crescendo},
    [STROKE_SLOW_IN] = {"Slow In", stroke_slow_in},
    [STROKE_HEARTBEAT] = {"Heartbeat", stroke_heartbeat},
    [STROKE_BREATHING] = {"Breathing", stroke_breathing},
    [STROKE_WANDER] = {"Wander", stroke_wander},
    [STROKE_PENDULUM] = {"Pendulum", stroke_pendulum},
};

machine_element_t g_machine_elements[ELEMENT_COUNT] = {
    [ELEMENT_POWER] = {"Power", true, 40, 100, false, false, true},
    [ELEMENT_SPEED] = {"Speed", true, 20, 100, false, false, true},
    [ELEMENT_LENGTH] = {"Length", true, 20, 100, false, false, true},
    // max_value is inclusive; roll-around uses range of max_value + 1
    [ELEMENT_STROKE_INDEX] = {"StrokeIndex", true, 3, STROKE_COUNT - 1, true, false, true},
    [ELEMENT_ACTIVE_INDEX] = {"ActiveElementIndex", false, 1, NUMBER_OF_DISPLAY_ELEMENTS - 1, false, false, true},
    [ELEMENT_STATE_INDEX] = {"MachineStateIndex", false, MACHINE_INITIAL, MACHINE_ODRIVE_FAULT, false, false, true},
    [ELEMENT_VBUS] = {"VBus", false, OM_BATTERY_UNKNOWN, 100, false, true, true},  // battery percent (6S), from VBus; unknown until the ODrive reports it
};

float inter_point(float xa, float xb, float ya, float yb, float xc)
{
    float dx = xb - xa;
    if (dx == 0.0f) {
        return yb;
    }
    float m = (yb - ya) / dx;
    return ((xc - xa) * m) + ya;
}

float adjusted_pot_pos(float actual_pos)
{
    float return_val = actual_pos;
    if (actual_pos >= s_linpot_adjust.adjust_pos) {
        if (POT_MAX == s_linpot_adjust.adjust_pos) {
            return_val = actual_pos;
        } else {
            float numerator = (actual_pos - s_linpot_adjust.adjust_pos);
            float denominator = POT_MAX - s_linpot_adjust.adjust_pos;
            float span = POT_MAX - s_linpot_adjust.redirect_pos;
            return_val = (numerator / denominator) * span + s_linpot_adjust.redirect_pos;
        }
    } else {
        if (s_linpot_adjust.adjust_pos == 0.0f) {
            return_val = actual_pos;
        } else {
            return_val = (actual_pos / s_linpot_adjust.adjust_pos) * s_linpot_adjust.redirect_pos;
        }
    }
    s_linpot_adjust.adjust_pos = actual_pos;
    s_linpot_adjust.redirect_pos = return_val;
    return return_val;
}

float stroke_eval(stroke_id_t stroke, uint32_t millis, float length)
{
    if (stroke >= STROKE_COUNT) {
        return 0.0f;
    }
    return s_strokes[stroke].fn(millis, length);
}

void machine_state_set(machine_state_ctx_t *ctx, machine_state_t new_state)
{
    if (ctx == NULL) {
        return;
    }
    ctx->state = new_state;
    g_machine_elements[ELEMENT_STATE_INDEX].current_value = (int32_t)new_state;  // sent with every frame
    ESP_LOGI(TAG, "Machine state -> %d", new_state);
}

machine_state_t machine_state_get(const machine_state_ctx_t *ctx)
{
    return ctx->state;
}

int32_t machine_element_value(machine_element_key_t key)
{
    // Aligned 32-bit reads are atomic; volatile so a polling loop always sees the latest value
    return *(volatile int32_t *)&g_machine_elements[key].current_value;
}

void machine_state_init_defaults(machine_state_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->state = MACHINE_INITIAL;
    g_machine_elements[ELEMENT_STATE_INDEX].current_value = MACHINE_INITIAL;
}

#define SETTINGS_NAMESPACE "settings"
static const struct {
    machine_element_key_t key;
    const char *nvs_key;
} s_saved_settings[] = {
    {ELEMENT_POWER, "power"},
    {ELEMENT_SPEED, "speed"},
    {ELEMENT_LENGTH, "length"},
    {ELEMENT_STROKE_INDEX, "stroke"},
};
volatile uint8_t machine_settings_saved_count = 0;
static machine_state_t s_state_before_fault = MACHINE_INITIAL;

void machine_settings_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t h;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "No saved settings: using the built-in defaults");
        return;
    }
    for (size_t i = 0; i < sizeof(s_saved_settings) / sizeof(s_saved_settings[0]); i++) {
        machine_element_t *e = &g_machine_elements[s_saved_settings[i].key];
        uint8_t v;
        if (nvs_get_u8(h, s_saved_settings[i].nvs_key, &v) == ESP_OK && v <= e->max_value) {
            e->current_value = v;
            ESP_LOGI(TAG, "Saved %s: %u", e->name, v);
        }
    }
    nvs_close(h);
}

static bool machine_settings_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        for (size_t i = 0; err == ESP_OK && i < sizeof(s_saved_settings) / sizeof(s_saved_settings[0]); i++) {
            err = nvs_set_u8(h, s_saved_settings[i].nvs_key, (uint8_t)machine_element_value(s_saved_settings[i].key));
        }
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Saving the settings failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Settings saved: power %ld, speed %ld, length %ld, stroke %ld",
             (long)machine_element_value(ELEMENT_POWER), (long)machine_element_value(ELEMENT_SPEED),
             (long)machine_element_value(ELEMENT_LENGTH), (long)machine_element_value(ELEMENT_STROKE_INDEX));
    return true;
}

bool machine_apply_button(machine_state_ctx_t *ctx, const button_event_t *ev)
{
    if ((ctx == NULL) || (ev == NULL)) {
        return false;
    }

    switch (ev->type) {
    case BUTTON_ROTARY:
    case BUTTON_POWER: {
        // The knob changes the active element; Power is set only from the manipulator's menu
        int32_t active_key = g_machine_elements[ELEMENT_ACTIVE_INDEX].current_value;
        if (ev->type == BUTTON_POWER) {
            active_key = ELEMENT_POWER;
        } else if (active_key < FIRST_KNOB_ELEMENT || active_key >= NUMBER_OF_DISPLAY_ELEMENTS) {
            active_key = FIRST_KNOB_ELEMENT;
        }
        machine_element_t *target = &g_machine_elements[active_key];
        int32_t capped = target->current_value + ev->rotary_delta;

        if (target->rollaround) {
            int32_t span = target->max_value + 1;
            if (span <= 0) {
                span = 1;
            }
            capped %= span;
            if (capped < 0) {
                capped += span;
            }
        } else {
            if (capped > target->max_value) {
                capped = target->max_value;
            } else if (capped < 0) {
                capped = 0;
            }
        }

        target->current_value = capped;  // sent to the manipulator with every frame
        ESP_LOGI(TAG, "%s:%ld", target->name, (long)target->current_value);
        break;
    }
    case BUTTON_ROTARY_BUTTON: {
        // Speed, Stroke, Length, then round again: clockwise on the manipulator's screen (Speed on
        // the left, Stroke below the middle, Length on the right). Power is left out: see BUTTON_POWER.
        int32_t next_active = g_machine_elements[ELEMENT_ACTIVE_INDEX].current_value - 1;
        if (next_active < FIRST_KNOB_ELEMENT || next_active >= NUMBER_OF_DISPLAY_ELEMENTS) {
            next_active = NUMBER_OF_DISPLAY_ELEMENTS - 1;
        }
        g_machine_elements[ELEMENT_ACTIVE_INDEX].current_value = next_active;
        ESP_LOGI(TAG, "Active Element:%s", g_machine_elements[next_active].name);
        break;
    }
    case BUTTON_SAVE_SETTINGS:
        if (machine_settings_save()) {
            machine_settings_saved_count = machine_settings_saved_count + 1;   // the manipulator shows "Settings saved"
        }
        break;
    case BUTTON_ODRIVE_SETUP:
        // From any state: the setup takes the ODrive over, and the controller restarts after it
        if (ctx->state != MACHINE_ODRIVE_SETUP && odrive_setup_request()) {
            machine_state_set(ctx, MACHINE_ODRIVE_SETUP);
        }
        break;
    case BUTTON_ODRIVE_SETUP_CANCEL:
        if (ctx->state == MACHINE_ODRIVE_SETUP) {
            odrive_setup_cancel();
        }
        break;
    case BUTTON_AUTO_BUTTON:
        // PREAUTO -> AUTO and SYNCING -> MANUAL are requested by the ODrive control loop
        if (ctx->state == MACHINE_ODRIVE_FAULT) {
            odrive_fault_retry();
        } else if (ctx->state == MACHINE_MANUAL) {
            machine_state_set(ctx, MACHINE_PREAUTO);
        } else if (ctx->state == MACHINE_PREAUTO) {
            // cancel before strokes start: hold the ODrive where it got to and bring the handle over
            machine_state_set(ctx, MACHINE_SYNCING);
        } else if (ctx->state == MACHINE_AUTO) {
            // stop: hold the ODrive where it is and bring the handle over
            machine_state_set(ctx, MACHINE_SYNCING);
        }
        ESP_LOGI(TAG, "Autobutton");
        break;
    case BUTTON_CONTROLLER_DISCONNECTED:
        // Ignored until homing is done (the homing sequence requests MANUAL itself), during the
        // ODrive setup, which carries on without the link and restarts the controller after, and
        // in a fault, which has to be seen and cleared first
        if (ctx->state != MACHINE_INITIAL && ctx->state != MACHINE_HOMING && ctx->state != MACHINE_ODRIVE_SETUP &&
            ctx->state != MACHINE_ODRIVE_FAULT) {
            ESP_LOGI(TAG, "Disconnected, put machinestate");
            machine_state_set(ctx, MACHINE_DISCONNECTED);
        }
        break;
    case BUTTON_CONTROLLER_CONNECTED:
        // Link back after a loss: the handle may have moved meanwhile, so first bring it back to
        // the held ODrive (SYNCING), then resume in MANUAL (never straight back into AUTO).
        // In any other state (homing, manual, auto) keep going as is.
        if (ctx->state == MACHINE_DISCONNECTED) {
            ESP_LOGI(TAG, "Connected, put machinestate");
            machine_state_set(ctx, MACHINE_SYNCING);
        }
        break;
    case BUTTON_HOMING_STARTED:
        if (ctx->state == MACHINE_INITIAL) {
            machine_state_set(ctx, MACHINE_HOMING);
        }
        break;
    case BUTTON_HOMING_DONE:
        // Bring the handle over to the homed ODrive first (SYNCING), which then hands over to MANUAL
        if (ctx->state == MACHINE_HOMING) {
            machine_state_set(ctx, MACHINE_SYNCING);
        }
        break;
    case BUTTON_ODRIVE_SYNCED:
        if (ctx->state == MACHINE_PREAUTO) {
            machine_state_set(ctx, MACHINE_AUTO);
        }
        break;
    case BUTTON_HANDLE_SYNCED:
        if (ctx->state == MACHINE_SYNCING) {
            machine_state_set(ctx, MACHINE_MANUAL);
        }
        break;
    case BUTTON_ODRIVE_FAULT:
        if (ctx->state != MACHINE_ODRIVE_FAULT && ctx->state != MACHINE_ODRIVE_SETUP) {
            s_state_before_fault = ctx->state;
            machine_state_set(ctx, MACHINE_ODRIVE_FAULT);
        }
        break;
    case BUTTON_ODRIVE_RECOVERED:
        // Running again: hold where it is and bring the handle over (SYNCING), or, if it never got
        // going, carry on starting up
        if (ctx->state == MACHINE_ODRIVE_FAULT) {
            bool starting = s_state_before_fault == MACHINE_INITIAL || s_state_before_fault == MACHINE_HOMING;
            machine_state_set(ctx, starting ? MACHINE_INITIAL : MACHINE_SYNCING);
        }
        break;
    default:
        return false;
    }

    return true;
}

void machine_state_task(void *arg)
{
    machine_state_ctx_t *ctx = (machine_state_ctx_t *)arg;
    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    button_event_t ev = {0};
    for (;;) {
        if (xQueueReceive(ctx->buttons_queue, &ev, portMAX_DELAY) == pdTRUE) {
            machine_apply_button(ctx, &ev);
        }
    }
}
