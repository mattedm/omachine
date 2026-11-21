// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * "Set up ODrive" over CAN (CANSimple, firmware 0.6.11): see odrive_setup.h.
 *
 * Settings are written by endpoint ID with RxSdo and read back with TxSdo to check each one.
 * Calibrations are started with Set_Axis_State and followed with the heartbeat (state, axis
 * error, procedure result), sped up to every 100 ms meanwhile. The anticogging calibration is
 * also watched for the motor stalling: it otherwise ends "successfully" with a useless map.
 */

#include "odrive_setup.h"
#include "odrive_setup_table.h"
#include "omachine_protocol.h"

#include <math.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "odrive-setup";

// From twai_odrive_control.c
extern volatile float EncoderEstimate;           // pos_estimate [turns], every 5 ms
extern volatile uint8_t AxisStatus;              // heartbeat: axis state
extern volatile uint32_t AxisError;              // heartbeat: active errors
extern volatile uint8_t AxisProcedureResult;     // heartbeat: procedure result
extern volatile uint32_t HeartbeatCount;         // heartbeats received
bool odrive_can_send(uint32_t cmd, const uint8_t *data, uint8_t len);

// CANSimple command IDs
#define CMD_GET_VERSION      0x00
#define CMD_RX_SDO           0x04
#define CMD_TX_SDO           0x05
#define CMD_SET_AXIS_STATE   0x07
#define CMD_REBOOT           0x16
#define CMD_CLEAR_ERRORS     0x18
#define SDO_READ             0
#define SDO_WRITE            1
#define REBOOT_NO_SAVE       0
#define REBOOT_SAVE          1

#define AXIS_IDLE                       1
#define AXIS_FULL_CALIBRATION_SEQUENCE  3
#define AXIS_ANTICOGGING_CALIBRATION    14
#define PROCEDURE_SUCCESS               0

// What the controller's CAN side is set up for (twai_odrive_control.c)
#define EXPECTED_CAN_BAUD    1000000
#define EXPECTED_NODE_ID     0

// Stiff gains for the anticogging calibration only; the operating gains come back from the table.
// A softer loop lets this coggy motor stall in a detent (found on the bench, Nov 2026).
#define CAL_POS_GAIN            250.0f
#define CAL_VEL_GAIN            1.3f
#define CAL_VEL_INTEGRATOR_GAIN 320.0f

#define SETUP_HEARTBEAT_MS      100      // heartbeat rate while calibrations run
#define SDO_REPLY_TIMEOUT_MS    50
#define SDO_TRIES               3
#define LEAVE_IDLE_TIMEOUT_MS   3000     // a calibration that hasn't started by then was refused
#define CALIBRATION_EXPECTED_S  20       // for the progress bar
#define CALIBRATION_TIMEOUT_S   120
#define STALL_WINDOW_S          30       // anticogging: less than STALL_MIN_TURNS in this long is a stall
#define STALL_MIN_TURNS         0.05f
#define REBOOT_TIMEOUT_MS       10000

typedef struct {
    uint16_t id;
    uint32_t bits;
} sdo_reply_t;

static TaskHandle_t s_task = NULL;
static QueueHandle_t s_sdo_queue = NULL;
static QueueHandle_t s_version_queue = NULL;
static volatile bool s_active = false;
static volatile bool s_cancel = false;
static volatile uint8_t s_ack = 0;
static volatile uint8_t s_stage = OM_SETUP_IDLE;
static volatile uint8_t s_progress = 0;

static void set_stage(uint8_t stage)
{
    s_progress = 0;
    s_stage = stage;
    ESP_LOGW(TAG, "Stage %u", stage);
}

static void set_progress(uint32_t done, uint32_t total)
{
    uint32_t pct = total ? done * 100u / total : 0;
    s_progress = (uint8_t)(pct > 99 ? 99 : pct);   // 100 only once the stage is over
}

void odrive_setup_on_rx(uint32_t cmd, const uint8_t data[8])
{
    if (cmd == CMD_TX_SDO && s_sdo_queue) {
        sdo_reply_t r = {
            .id = (uint16_t)(data[1] | (data[2] << 8)),
            .bits = (uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24),
        };
        xQueueSend(s_sdo_queue, &r, 0);
    } else if (cmd == CMD_GET_VERSION && s_version_queue) {
        xQueueOverwrite(s_version_queue, data);
    }
}

static bool send_retry(uint32_t cmd, const uint8_t *data, uint8_t len)
{
    for (int i = 0; i < 20; i++) {
        if (odrive_can_send(cmd, data, len)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "CAN command 0x%02" PRIx32 " not sent", cmd);
    return false;
}

static bool set_axis_state(uint32_t state)
{
    uint8_t d[4] = {(uint8_t)state, (uint8_t)(state >> 8), (uint8_t)(state >> 16), (uint8_t)(state >> 24)};
    return send_retry(CMD_SET_AXIS_STATE, d, sizeof(d));
}

static bool clear_errors(void)
{
    uint8_t d[1] = {0};
    return send_retry(CMD_CLEAR_ERRORS, d, sizeof(d));
}

// One SDO request and its reply (writes are confirmed since firmware 0.6.11)
static bool sdo_transfer(uint8_t opcode, uint16_t id, uint32_t bits, uint32_t *reply_bits)
{
    xQueueReset(s_sdo_queue);
    uint8_t d[8] = {opcode, (uint8_t)id, (uint8_t)(id >> 8), 0,
                    (uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
    if (!send_retry(CMD_RX_SDO, d, sizeof(d))) {
        return false;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SDO_REPLY_TIMEOUT_MS);
    sdo_reply_t r;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0 || xQueueReceive(s_sdo_queue, &r, deadline - now) != pdTRUE) {
            return false;
        }
        if (r.id == id) {
            if (reply_bits) {
                *reply_bits = r.bits;
            }
            return true;
        }
    }
}

static bool sdo_read(uint16_t id, uint32_t *bits)
{
    for (int i = 0; i < SDO_TRIES; i++) {
        if (sdo_transfer(SDO_READ, id, 0, bits)) {
            return true;
        }
    }
    return false;
}

// A write isn't waited on: write_setting reads every value back instead, which says more than the
// confirmation 0.6.11 sends (sdo_transfer flushes that before the read)
#define SDO_WRITE_SETTLE_MS 2
static bool sdo_write(uint16_t id, uint32_t bits)
{
    uint8_t d[8] = {SDO_WRITE, (uint8_t)id, (uint8_t)(id >> 8), 0,
                    (uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
    if (!send_retry(CMD_RX_SDO, d, sizeof(d))) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(SDO_WRITE_SETTLE_MS));
    return true;
}

// A write that must have taken (not for values that change by themselves, like pos_estimate)
static bool sdo_write_checked(uint16_t id, uint32_t bits)
{
    uint32_t back = 0;
    if (!sdo_write(id, bits) || !sdo_read(id, &back) || back != bits) {
        ESP_LOGE(TAG, "Endpoint %u: wrote 0x%08" PRIx32 ", reads 0x%08" PRIx32, id, bits, back);
        return false;
    }
    return true;
}

static uint32_t float_bits(float f)
{
    uint32_t b;
    memcpy(&b, &f, sizeof(b));
    return b;
}

static float bits_float(uint32_t b)
{
    float f;
    memcpy(&f, &b, sizeof(f));
    return f;
}

static uint32_t type_mask(uint8_t type)
{
    switch (type) {
    case ODRIVE_T_BOOL: case ODRIVE_T_U8: case ODRIVE_T_I8: return 0xffu;
    case ODRIVE_T_U16: case ODRIVE_T_I16: return 0xffffu;
    default: return 0xffffffffu;
    }
}

// Write a setting and read it back
static bool write_setting(const odrive_setting_t *s)
{
    uint32_t back = 0;
    if (!sdo_write(s->id, s->bits) || !sdo_read(s->id, &back)) {
        ESP_LOGE(TAG, "%s: no reply", s->path);
        return false;
    }
    uint32_t mask = type_mask(s->type);
    bool same = (back & mask) == (s->bits & mask);
    if (!same && s->type == ODRIVE_T_FLOAT) {
        same = isnan(bits_float(back)) && isnan(bits_float(s->bits));   // any NaN will do
    }
    if (!same) {
        ESP_LOGE(TAG, "%s: wrote 0x%08" PRIx32 ", reads 0x%08" PRIx32, s->path, s->bits, back & mask);
    }
    return same;
}

static const odrive_setting_t *find_setting(const char *path)
{
    for (int i = 0; i < ODRIVE_SETTING_COUNT; i++) {
        if (strcmp(odrive_settings[i].path, path) == 0) {
            return &odrive_settings[i];
        }
    }
    return NULL;
}

static float setting_float(const char *path, float fallback)
{
    const odrive_setting_t *s = find_setting(path);
    return (s && s->type == ODRIVE_T_FLOAT) ? bits_float(s->bits) : fallback;
}

// Wait for a heartbeat newer than 'after'
static bool wait_heartbeat(uint32_t after, uint32_t timeout_ms)
{
    int64_t end_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (HeartbeatCount == after) {
        if (esp_timer_get_time() > end_us) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}

typedef enum { PROC_OK, PROC_FAILED, PROC_CANCELLED, PROC_STALLED, PROC_TIMEOUT } proc_result_t;

// Start a calibration state and follow it until the ODrive is back in IDLE
static proc_result_t run_procedure(uint32_t state, uint32_t expected_s, uint32_t timeout_s, bool watch_stall)
{
    clear_errors();
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!set_axis_state(state)) {
        return PROC_FAILED;
    }
    int64_t start_us = esp_timer_get_time();

    // The heartbeat still says IDLE until the ODrive has started
    while (AxisStatus == AXIS_IDLE) {
        if (s_cancel) {
            return PROC_CANCELLED;
        }
        if (esp_timer_get_time() - start_us > LEAVE_IDLE_TIMEOUT_MS * 1000LL) {
            ESP_LOGE(TAG, "State %" PRIu32 " didn't start: axis error 0x%" PRIx32 ", procedure result %u",
                     state, AxisError, AxisProcedureResult);
            return PROC_FAILED;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    float window[STALL_WINDOW_S] = {0};
    int samples = 0;
    int64_t next_sample_us = start_us;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int64_t now_us = esp_timer_get_time();
        uint32_t elapsed_s = (uint32_t)((now_us - start_us) / 1000000);
        if (s_cancel) {
            set_axis_state(AXIS_IDLE);
            return PROC_CANCELLED;
        }
        if (AxisStatus == AXIS_IDLE) {
            if (AxisProcedureResult == PROCEDURE_SUCCESS && AxisError == 0) {
                return PROC_OK;
            }
            ESP_LOGE(TAG, "State %" PRIu32 " ended with axis error 0x%" PRIx32 ", procedure result %u",
                     state, AxisError, AxisProcedureResult);
            return PROC_FAILED;
        }
        if (elapsed_s >= timeout_s) {
            ESP_LOGE(TAG, "State %" PRIu32 " still running after %" PRIu32 " s", state, elapsed_s);
            set_axis_state(AXIS_IDLE);
            return PROC_TIMEOUT;
        }
        set_progress(elapsed_s, expected_s);

        if (watch_stall && now_us >= next_sample_us) {
            // One position a second; the oldest is STALL_WINDOW_S seconds back
            next_sample_us += 1000000;
            window[samples % STALL_WINDOW_S] = EncoderEstimate;
            samples++;
            if (samples > STALL_WINDOW_S + 5) {
                float lo = window[0], hi = window[0];
                for (int i = 1; i < STALL_WINDOW_S; i++) {
                    lo = fminf(lo, window[i]);
                    hi = fmaxf(hi, window[i]);
                }
                if (hi - lo < STALL_MIN_TURNS) {
                    ESP_LOGE(TAG, "Stalled at %.3f turns (%.3f in %d s)", EncoderEstimate, hi - lo, STALL_WINDOW_S);
                    set_axis_state(AXIS_IDLE);
                    return PROC_STALLED;
                }
            }
        }
    }
}

// Reboot the ODrive (saving or not) and wait for it to come back
static bool reboot_odrive(uint8_t action)
{
    uint8_t d[1] = {action};
    if (!send_retry(CMD_REBOOT, d, sizeof(d))) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));   // heartbeats from before the reboot don't count
    return wait_heartbeat(HeartbeatCount, REBOOT_TIMEOUT_MS);
}

static uint8_t run_setup(void)
{
    bool touched = false;   // settings changed on the ODrive: a reboot without saving undoes them
    uint8_t failure = OM_SETUP_CANCELLED;
    uint32_t raw = 0, gain = 0;
    const odrive_setting_t *pos_gain = find_setting("axis0.controller.config.pos_gain");

    // --- Check: the ODrive answers, runs the firmware the table is for, and reads back the
    // CAN settings the controller is using (so the endpoint IDs are right)
    set_stage(OM_SETUP_CHECKING);
    set_axis_state(AXIS_IDLE);
    vTaskDelay(pdMS_TO_TICKS(200));
    uint8_t ver[8];
    bool got_version = false;
    for (int i = 0; i < 3 && !got_version; i++) {
        xQueueReset(s_version_queue);
        if (send_retry(CMD_GET_VERSION, NULL, 0)) {
            got_version = xQueueReceive(s_version_queue, ver, pdMS_TO_TICKS(500)) == pdTRUE;
        }
    }
    if (!got_version) {
        ESP_LOGE(TAG, "No Get_Version reply");
        return OM_SETUP_FAILED_NO_ODRIVE;
    }
    ESP_LOGW(TAG, "ODrive hw %u.%u.%u, firmware %u.%u.%u%s", ver[1], ver[2], ver[3], ver[4], ver[5], ver[6],
             ver[7] ? " (unreleased)" : "");
    if (ver[1] != ODRIVE_SETUP_HW_PRODUCT_LINE || ver[2] != ODRIVE_SETUP_HW_VERSION || ver[3] != ODRIVE_SETUP_HW_VARIANT ||
        ver[4] != ODRIVE_SETUP_FW_MAJOR || ver[5] != ODRIVE_SETUP_FW_MINOR || ver[6] != ODRIVE_SETUP_FW_REVISION) {
        ESP_LOGE(TAG, "The settings table is for hw %d.%d.%d, firmware %d.%d.%d: regenerate it",
                 ODRIVE_SETUP_HW_PRODUCT_LINE, ODRIVE_SETUP_HW_VERSION, ODRIVE_SETUP_HW_VARIANT,
                 ODRIVE_SETUP_FW_MAJOR, ODRIVE_SETUP_FW_MINOR, ODRIVE_SETUP_FW_REVISION);
        return OM_SETUP_FAILED_VERSION;
    }
    uint32_t baud = 0, node = 0xff;
    if (!sdo_read(EP_CAN_BAUD_RATE, &baud) || !sdo_read(EP_CAN_NODE_ID, &node)) {
        ESP_LOGE(TAG, "No SDO reply");
        return OM_SETUP_FAILED_NO_ODRIVE;
    }
    if (baud != EXPECTED_CAN_BAUD || node != EXPECTED_NODE_ID) {
        ESP_LOGE(TAG, "Endpoint check: baud rate reads %" PRIu32 ", node ID %" PRIu32 ": wrong endpoint IDs?", baud, node);
        return OM_SETUP_FAILED_VERSION;
    }
    if (s_cancel) {
        return OM_SETUP_CANCELLED;
    }

    // --- Configure: every setting from the file
    set_stage(OM_SETUP_CONFIGURING);
    touched = true;
    for (int i = 0; i < ODRIVE_SETTING_COUNT; i++) {
        if (s_cancel) {
            goto undo;
        }
        if (!write_setting(&odrive_settings[i])) {
            failure = OM_SETUP_FAILED_CONFIG;
            goto undo;
        }
        set_progress(i + 1, ODRIVE_SETTING_COUNT);
    }
    ESP_LOGW(TAG, "%d settings written", ODRIVE_SETTING_COUNT);
    if (!sdo_write_checked(EP_HEARTBEAT_RATE_MS, SETUP_HEARTBEAT_MS)) {
        failure = OM_SETUP_FAILED_CONFIG;
        goto undo;
    }

    // --- Motor and encoder calibration
    set_stage(OM_SETUP_CALIBRATING);
    switch (run_procedure(AXIS_FULL_CALIBRATION_SEQUENCE, CALIBRATION_EXPECTED_S, CALIBRATION_TIMEOUT_S, false)) {
    case PROC_OK: break;
    case PROC_CANCELLED: goto undo;
    default: failure = OM_SETUP_FAILED_CALIBRATE; goto undo;
    }

    // --- Anticogging: stiff gains, the position lined up with the absolute encoder (as at every
    // start), then the calibration
    set_stage(OM_SETUP_ANTICOGGING);
    if (!sdo_write_checked(EP_POS_GAIN, float_bits(CAL_POS_GAIN)) || !sdo_write_checked(EP_VEL_GAIN, float_bits(CAL_VEL_GAIN)) ||
        !sdo_write_checked(EP_VEL_INTEGRATOR_GAIN, float_bits(CAL_VEL_INTEGRATOR_GAIN)) ||
        !sdo_read(EP_ONBOARD_ENCODER_RAW, &raw) || !sdo_write(EP_POS_ESTIMATE, raw)) {
        failure = OM_SETUP_FAILED_CONFIG;
        goto undo;
    }
    {
        float coarse = setting_float("axis0.config.anticogging.calib_coarse_tuning_duration", 120.0f);
        float fine = setting_float("axis0.config.anticogging.calib_fine_tuning_duration", 180.0f);
        const odrive_setting_t *bidir = find_setting("axis0.config.anticogging.calib_bidirectional");
        uint32_t expected_s = (uint32_t)((coarse + fine) * ((bidir && bidir->bits) ? 2.0f : 1.0f)) + 10;
        ESP_LOGW(TAG, "Anticogging: about %" PRIu32 " s", expected_s);
        switch (run_procedure(AXIS_ANTICOGGING_CALIBRATION, expected_s, expected_s * 2 + 60, true)) {
        case PROC_OK: break;
        case PROC_CANCELLED: goto undo;
        case PROC_STALLED: failure = OM_SETUP_FAILED_STALL; goto undo;
        default: failure = OM_SETUP_FAILED_ANTICOG; goto undo;
        }
    }

    // --- The operating settings back (gains, control and input mode, heartbeat rate), save
    set_stage(OM_SETUP_SAVING);
    for (int i = 0; i < ODRIVE_RESTORE_COUNT; i++) {
        if (!write_setting(&odrive_settings[odrive_restore[i]])) {
            failure = OM_SETUP_FAILED_CONFIG;
            goto undo;
        }
        set_progress(i + 1, ODRIVE_RESTORE_COUNT + 2);
    }
    if (s_cancel) {
        goto undo;
    }
    if (!reboot_odrive(REBOOT_SAVE)) {
        ESP_LOGE(TAG, "No heartbeat after saving");
        return OM_SETUP_FAILED_SAVE;
    }
    set_progress(ODRIVE_RESTORE_COUNT + 1, ODRIVE_RESTORE_COUNT + 2);
    // The saved operating gain is back after the reboot, not the calibration one
    if (!sdo_read(EP_POS_GAIN, &gain) || !pos_gain || gain != pos_gain->bits) {
        ESP_LOGE(TAG, "After saving, pos_gain reads 0x%08" PRIx32, gain);
        return OM_SETUP_FAILED_SAVE;
    }
    return OM_SETUP_DONE;

undo:
    // Nothing has been saved: a reboot without saving brings back the old settings
    set_axis_state(AXIS_IDLE);
    vTaskDelay(pdMS_TO_TICKS(200));
    if (touched && !reboot_odrive(REBOOT_NO_SAVE)) {
        ESP_LOGE(TAG, "No heartbeat after the reboot");
    }
    return failure;
}

static void setup_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t start_us = esp_timer_get_time();
        uint8_t result = run_setup();
        s_progress = 100;
        s_stage = result;
        ESP_LOGW(TAG, "Setup finished: result %u after %" PRId64 " s; restarting in %d ms", result,
                 (esp_timer_get_time() - start_us) / 1000000, OM_SETUP_RESULT_HOLD_MS);
        // Long enough for the manipulator to show the result, then start afresh
        vTaskDelay(pdMS_TO_TICKS(OM_SETUP_RESULT_HOLD_MS));
        esp_restart();
    }
}

float odrive_setup_config_float(const char *path, float fallback)
{
    return setting_float(path, fallback);
}

void odrive_setup_init(void)
{
    s_sdo_queue = xQueueCreate(8, sizeof(sdo_reply_t));
    s_version_queue = xQueueCreate(1, 8);
    assert(s_sdo_queue && s_version_queue);
    xTaskCreate(setup_task, "ODrive_Setup", 4096, NULL, 9, &s_task);
}

bool odrive_setup_request(void)
{
    if (s_active || s_task == NULL) {
        return false;
    }
    s_cancel = false;
    s_active = true;
    set_stage(OM_SETUP_CHECKING);
    xTaskNotifyGive(s_task);
    return true;
}

void odrive_setup_cancel(void)
{
    if (s_active) {
        s_cancel = true;
        ESP_LOGW(TAG, "Cancel requested");
    }
}

bool odrive_setup_active(void)
{
    return s_active;
}

void odrive_setup_ack(uint8_t setup_count)
{
    s_ack = setup_count;
}

void odrive_setup_status(uint8_t *ack, uint8_t *stage, uint8_t *progress)
{
    *ack = s_ack;
    *stage = s_stage;
    *progress = s_progress;
}
