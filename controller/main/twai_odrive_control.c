/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 MattEdm
 *
 * SPDX-License-Identifier: GPL-3.0-or-later AND Apache-2.0
 *
 * Started from Espressif's TWAI example (Apache-2.0) and since modified; the changes are GPL-3.0-or-later.
 */

#include "firmware_valid.h"
#include "firmware_receiver.h"
#include "log_async.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "shared_types.h"
#include "machine_state.h"
#include "odrive_setup.h"
#include "status_led.h"

#define TWAI_LISTENER_TX_GPIO   7
#define TWAI_LISTENER_RX_GPIO   15                
#define TWAI_BITRATE            1000000  // must match the ODrive: odrv0.can.config.baud_rate (500000 if the bus is long or noisy)

#define TWAI_FRAME_MAX_LEN 8
#define TWAI_TX_QUEUE_TIMEOUT_MS 10  // max wait for space in the TX queue
#define TWAI_TX_DONE_TIMEOUT_MS  20  // max wait for a frame to go out on the bus
#define TWAI_RX_QUEUE_LEN 64

// ODrive S1 CAN interface
#define ODRIVE_NODE_ID          0

//  ODrive Axis States
#define AXIS_STATE_IDLE         1
#define CLOSED_LOOP_CONTROL     8


//define command ids
#define GET_VERSION             0   // reply goes to odrive_setup.c
#define ODRIVE_HEARTBEAT        1
#define ESTOP                   2
#define GET_MOTOR_ERROR         3/*IMPLEMENTED IN REQUEST_MSG*/
#define GET_ENCODER_ERROR       4/*IMPLEMENTED IN REQUEST_MSG*/
#define SET_AXIS_NODE_ID        6
#define SET_AXIS_REQUESTED_STATE  7
#define GET_ENCODER_ESTIMATE    9 /*IMPLEMENTED IN REQUEST_MSG*/
#define GET_ENCODER_COUNTS      10 /*IMPLEMENTED IN REQUEST_MSG*/
#define MOVE_TO_POS             11
#define SET_POS_SETPOINT        12
#define SET_VEL_SETPOINT        13
#define SET_CUR_SETPOINT        14
#define SET_LIMITS              15  // Set_Limits: velocity limit [rev/s], current limit [A]
#define START_ANTI_COGGING      16
#define SET_TRAJ_VEL_LIMIT      17
#define SET_TRAJ_ACCEL_LIMIT    18
#define SET_TRAJ_A_PER_CSS      19
#define GET_IQ_VALUES           20 /*IMPLEMENTED IN REQUEST_MSG*/
#define REBOOT_ODRIVE           22
#define CLEAR_ERRORS            0x18
#define GET_VBUS_VOLTAGE        23 /*IMPLEMENTED IN REQUEST_MSG*/
#define SET_VEL_PI_GAIN         24
#define TX_SDO                  5   // SDO replies, for odrive_setup.c
#define SET_POS_GAIN            0x1A
#define SET_VEL_GAINS           0x1B

// Buffer for burst data handling
#define POLL_DEPTH              400
void bytes2Float(uint8_t * bytes_temp, float* float_variable);

static const char *TAG = "odrive-control";

// Auto-mode transitions: PREAUTO -> AUTO once the ODrive has reached the handle,
// SYNCING -> MANUAL once the handle has reached the (held) ODrive
#define SYNC_TOLERANCE  OM_SYNC_TOLERANCE   // "matched" = within 2% of the stroke span (shared with the manipulator)
#define STROKE_CHANGE_SPEED 0.5f  // move to a new stroke's start at this many stroke spans per second
// ODrive limits, sent with Set_Limits whenever the manipulator's Power setting changes
#define VEL_LIMIT           5.0f  // rev/s, as axis0.controller.config.vel_limit
#define MAX_CURRENT_A       20.0f // Power 100% = this many amps
#define MIN_CURRENT_A       2.0f  // floor, so low Power never leaves the ODrive limp
#define MANUAL_EASE_SPEED   0.2f  // on entering MANUAL, move the target from the hold position at this many spans per second
// MANUAL: the target goes this share of the way from the ODrive to the handle on each pass. Below 1
// it's soft, but the lag is the ODrive's own (about 20 ms) divided by it: 0.5 measured 38 ms.
#define MANUAL_HANDLE_WEIGHT 0.75f

// MANUAL compliance: the carriage is soft to push by hand while the handle is free to follow it,
// and stiff (the gains in odrive-s1-config.json) while the handle leads, or holds still against a push.
// Stiffness runs 0 (soft gains) to 1 (stiff), log-scaled between the two.
#define COMPLIANCE_ENABLE    1
#define SOFT_POS_GAIN        5.0f    // (turns/s) per turn of error
#define SOFT_VEL_GAIN        0.15f   // Nm per turns/s
#define HANDLE_VEL_FILTER_S  0.02f   // smoothing of the handle's speed
#define HANDLE_MOVING_TPS    0.05f   // the handle slower than this (turns/s) counts as still
#define HELD_GAP_TURNS       0.01f   // the carriage this far from a still handle: the handle is held
#define HELD_DELAY_S         0.5f    // held this long, the ODrive stiffens...
#define HELD_STIFFEN_S       0.5f    // ...over this long, bringing the carriage back to the handle
#define HELD_KEEP_S          1.0f    // and stays stiff this long after, so a steady push doesn't pump
#define LEAD_STIFFEN_S       0.05f   // the handle leading: stiff this quickly
#define SOFTEN_S             0.3f    // otherwise back to soft over this long
#define GAIN_SEND_STEP       0.01f   // send new gains once the stiffness has moved this much

// Set to 1 to log setpoint timing once a second (diagnosing jerky ODrive motion)
#define TIMING_DEBUG    0
// Set to 1 to log the handle and ODrive positions every TRACK_DEBUG_MS, for measuring how closely
// the ODrive follows the handle. Lines: "TRK <ms> <handle> <target sent> <ODrive> <state> <stiffness>", positions
// in 1e-5 turns (handle = where the handle asks for, mapped onto the ODrive's travel), then the
// MANUAL compliance stiffness sent (0..100)
#define TRACK_DEBUG     0
#define TRACK_DEBUG_MS  10
#if TIMING_DEBUG
static volatile uint32_t s_pos_skipped = 0;   // set_position() skipped: previous frame still pending
static volatile uint32_t s_pos_late = 0;      // set_position() frame not on the bus within TWAI_TX_DONE_TIMEOUT_MS
#endif

static machine_state_ctx_t s_machine_ctx;

typedef struct {
    machine_state_t machine_state;
    int32_t speed_pct;
    int32_t length_pct;
    int32_t stroke_index;
} odrive_control_state_t;


QueueHandle_t odrive_encoder_queue;
QueueHandle_t manipulator_encoder_queue;

volatile float EncoderEstimate = 0.0;
SemaphoreHandle_t EncoderEstimate_semaphore;
volatile float EncoderExtended = 0.0;
volatile float EncoderContracted = 0.0;
volatile bool EncoderExtensionsDone = false;  // set after Extended/Contracted (volatile keeps that order)

float VBusVoltage = 0.0;
SemaphoreHandle_t VBusVoltage_semaphore;

volatile uint8_t AxisStatus = 0;
volatile uint32_t AxisError = 0;
volatile uint8_t AxisProcedureResult = 0;   // heartbeat byte 5: result of the last calibration etc.
volatile uint32_t HeartbeatCount = 0;       // heartbeats received, so odrive_setup.c can tell a new one
SemaphoreHandle_t AxisStatus_semaphore;


typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_listener_data_t;

typedef struct {
    twai_node_handle_t node_hdl;
    twai_listener_data_t *rx_pool;
    SemaphoreHandle_t free_pool_semaphore;
    SemaphoreHandle_t rx_result_semaphore;
    int write_idx;
    int read_idx;
} twai_ctx_t;

typedef struct { 
    uint32_t id; 
    uint8_t dlc; 
    uint8_t data[TWAI_FRAME_MAX_LEN]; 
} twai_rx_msg_t;
static QueueHandle_t twai_rx_queue;

//architecture dependent byte to float converter (Endian-ness)
void bytes2Float(uint8_t * bytes_temp, float* float_variable){ 
  union {
    float a;
    uint8_t bytes[4];
  } thing;
  //swap around for different endlian
  thing.bytes[0] = bytes_temp[0];
  thing.bytes[1] = bytes_temp[1];
  thing.bytes[2] = bytes_temp[2];
  thing.bytes[3] = bytes_temp[3];
  *(float_variable) = thing.a;
  //printf("b2f float = %f, bytes_temp: %x %x %x %x \n",thing.a, bytes_temp[0],bytes_temp[1],bytes_temp[2],bytes_temp[3]);
  //printf("b2f *float = %f, thing.bytes: %x %x %x %x \n",*(float_variable), thing.bytes[0],thing.bytes[1],thing.bytes[2],thing.bytes[3]);
		
}

// Bus errors are counted in the ISR and reported from task context by report_bus_errors().
// While the ODrive is still booting every retry of an un-ACKed frame raises an error, thousands
// per second; logging each one from the ISR starved core 0 and tripped the interrupt watchdog.
static volatile uint32_t s_bus_error_count = 0;
static volatile uint32_t s_bus_error_flags = 0;
// The same for the other things that used to be logged as they happened (from an ISR, or with
// ESP_EARLY_LOG, which writes to the console directly and waits while the USB console is connected)
static volatile uint32_t s_tx_fail_count = 0;
static volatile uint32_t s_tx_fail_id = 0;
static volatile uint32_t s_state_changes = 0;
static volatile int s_bus_state = 0;
static volatile uint32_t s_bad_length_count = 0;
static volatile uint32_t s_unknown_count = 0;
static volatile uint32_t s_unknown_id = 0;

// Error callback
static bool IRAM_ATTR twai_listener_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    s_bus_error_count++;
    s_bus_error_flags = edata->err_flags.val;
    return false;
}

static void report_bus_errors(void)
{
    static const char *state_names[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    static uint32_t last_count = 0, last_tx_fail = 0, last_state_changes = 0, last_bad_length = 0, last_unknown = 0;
    uint32_t count = s_bus_error_count;
    if (count != last_count) {
        ESP_LOGW(TAG, "%" PRIu32 " CAN bus errors (last flags 0x%" PRIx32 ")", count - last_count, s_bus_error_flags);
        last_count = count;
    }
    if (s_state_changes != last_state_changes) {
        last_state_changes = s_state_changes;
        int st = s_bus_state;
        ESP_LOGW(TAG, "CAN bus state now %s", (st >= 0 && st < 4) ? state_names[st] : "?");
    }
    if (s_tx_fail_count != last_tx_fail) {
        ESP_LOGW(TAG, "%" PRIu32 " CAN frames not sent (last ID 0x%" PRIx32 ")", s_tx_fail_count - last_tx_fail, s_tx_fail_id);
        last_tx_fail = s_tx_fail_count;
    }
    if (s_bad_length_count != last_bad_length) {
        ESP_LOGW(TAG, "%" PRIu32 " CAN frames with the wrong length", s_bad_length_count - last_bad_length);
        last_bad_length = s_bad_length_count;
    }
    if (s_unknown_count != last_unknown) {
        ESP_LOGW(TAG, "%" PRIu32 " unexpected CAN frames (last ID 0x%" PRIx32 ")", s_unknown_count - last_unknown, s_unknown_id);
        last_unknown = s_unknown_count;
    }
}

// Node state
static bool IRAM_ATTR twai_listener_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    s_bus_state = edata->new_sta;   // reported by report_bus_errors()
    s_state_changes++;
    return false;
}

// TWAI receive callback - store data and signal
static bool IRAM_ATTR twai_listener_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;

    uint8_t buffer[8];
    twai_frame_t frame = {
        .buffer = &buffer[0],
        .buffer_len = 8
    };

    


    twai_node_receive_from_isr(handle, &frame);
    twai_rx_msg_t msg;

    // Any length: SDO replies are only as long as their value (a bool's is 5 bytes). The task
    // checks the length each message needs; the rest of data[] is zeros.
    if (frame.header.dlc <= TWAI_FRAME_MAX_LEN) {
        msg.id = frame.header.id;
        msg.dlc = frame.header.dlc;
        memset(msg.data, 0, sizeof(msg.data));
        memcpy(msg.data, frame.buffer, frame.header.dlc);
        xQueueSendFromISR(twai_rx_queue, &msg, &woken);
    }

    return woken == pdTRUE;  // only request a context switch if a task was woken

}

// TX frames. The driver queues a *pointer* to the frame and reads it later from its ISR, so each
// frame and its payload must stay valid until its tx_done callback. They live in static storage,
// and a new one is only queued once the previous one of the same kind has gone out. (Stack frames
// left queued while the ODrive was not ACKing yet crashed the driver once the bus came up.)
typedef struct {
    float position;
    int16_t Vel_FF;     // 0.001 rev/s per unit
    int16_t Torque_FF;  // 0.001 Nm per unit
} set_input_pos_payload_t;
_Static_assert(sizeof(set_input_pos_payload_t) == 8, "Set_Input_Pos payload must be 8 bytes");

static uint32_t s_axis_state_payload;
static twai_frame_t s_axis_state_frame = {
    .header.id = SET_AXIS_REQUESTED_STATE,
    .header.rtr = 0, // Remote Transmission Request
    .buffer = (uint8_t *) &s_axis_state_payload,
    .buffer_len = sizeof(s_axis_state_payload),
};
static volatile bool s_axis_state_in_flight = false;

static set_input_pos_payload_t s_position_payload;
static twai_frame_t s_position_frame = {
    .header.id = SET_POS_SETPOINT,
    .header.rtr = 0, // Remote Transmission Request
    .buffer = (uint8_t *) &s_position_payload,
    .buffer_len = sizeof(s_position_payload),
};
static volatile bool s_position_in_flight = false;

typedef struct {
    float vel_limit;      // rev/s
    float current_limit;  // A
} set_limits_payload_t;
_Static_assert(sizeof(set_limits_payload_t) == 8, "Set_Limits payload must be 8 bytes");

static set_limits_payload_t s_limits_payload;
static twai_frame_t s_limits_frame = {
    .header.id = SET_LIMITS,
    .header.rtr = 0, // Remote Transmission Request
    .buffer = (uint8_t *) &s_limits_payload,
    .buffer_len = sizeof(s_limits_payload),
};
static volatile bool s_limits_in_flight = false;

// Any other command, for odrive_setup.c (one at a time)
static uint8_t s_generic_payload[TWAI_FRAME_MAX_LEN];
static twai_frame_t s_generic_frame = {
    .header.rtr = 0,
    .buffer = s_generic_payload,
};
static volatile bool s_generic_in_flight = false;

// Transmission completion callback
static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    if (edata->done_tx_frame == &s_axis_state_frame) {
        s_axis_state_in_flight = false;
    } else if (edata->done_tx_frame == &s_position_frame) {
        s_position_in_flight = false;
    } else if (edata->done_tx_frame == &s_limits_frame) {
        s_limits_in_flight = false;
    } else if (edata->done_tx_frame == &s_generic_frame) {
        s_generic_in_flight = false;
    }
    if (!edata->is_tx_success) {
        s_tx_fail_id = edata->done_tx_frame->header.id;   // reported by report_bus_errors()
        s_tx_fail_count++;
    }
    return false; // No task wake required
}


uint16_t C_OdriveEncoder_ManipulatorEncoder (float odrive_encoder_value) {
    uint16_t manipulator_encoder_value;
    // Map ODrive encoder value (0.0 to 1.0) to manipulator encoder value (0 to 32767)
    if (EncoderExtensionsDone) { 
        if (odrive_encoder_value < EncoderContracted) {
            odrive_encoder_value = EncoderContracted;
        }
        if (odrive_encoder_value > EncoderExtended) {
            odrive_encoder_value = EncoderExtended;
        }
    manipulator_encoder_value = (uint16_t)(((EncoderContracted-odrive_encoder_value)/(-EncoderExtended+EncoderContracted))*32767); 
    }
    else {
        manipulator_encoder_value = 0; // Default to 0 until extensions set
    }
    return manipulator_encoder_value;
}  


float C_ManipulatorEncoder_OdriveEncoder (uint16_t manipulator_encoder_value) {
    float odrive_encoder_value;
    // Map manipulator encoder value (0 to 32767) to ODrive encoder value (0.0 to 1.0) 
    odrive_encoder_value = ((((float)(manipulator_encoder_value))/32767)*(EncoderExtended-EncoderContracted)) + EncoderContracted; 
    
    return odrive_encoder_value;
}


// Listener task function
void twai_process_task(void *pvParameters) {
    twai_rx_msg_t msg;
    
    while (xQueueReceive(twai_rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
        // SDO replies (TxSdo) carry a 4-byte header and then the value; everything else is 8 bytes
        uint8_t needed = (msg.id == TX_SDO) ? 4 : 8;
        if (msg.dlc < needed) {
            s_bad_length_count++;   // reported by report_bus_errors()
            continue;
        }
        switch (msg.id)
        {
        case GET_ENCODER_ESTIMATE: {
            float encoder_estimate;
            memcpy(&encoder_estimate, &msg.data[0], sizeof(encoder_estimate));  // data[] is unaligned
            EncoderEstimate = encoder_estimate;
            encoder_estimate_msg_t enc_msg = { .value = C_OdriveEncoder_ManipulatorEncoder(EncoderEstimate) };
            xQueueOverwrite(odrive_encoder_queue, &enc_msg);  // Always keep latest

            ESP_LOGI(TAG, "RX Encoder Position: %f", \
                EncoderEstimate);
            xSemaphoreGive(EncoderEstimate_semaphore);
            break;
        }
        
        case ODRIVE_HEARTBEAT: {
            uint32_t axis_error;
            memcpy(&axis_error, &msg.data[0], sizeof(axis_error));
            AxisError = axis_error;
            AxisStatus = msg.data[4];
            AxisProcedureResult = msg.data[5];
            HeartbeatCount = HeartbeatCount + 1;
            ESP_LOGI(TAG, "RX Axis State: %x %x", \
                AxisStatus, AxisError);
            xSemaphoreGive(AxisStatus_semaphore);
            break;
        }
        case GET_VERSION:
        case TX_SDO:
            odrive_setup_on_rx(msg.id, msg.data);
            break;
        case GET_VBUS_VOLTAGE:
            memcpy(&VBusVoltage, &msg.data[0], sizeof(VBusVoltage));
            ESP_LOGI(TAG, "RX VBus Voltage: %f", \
                VBusVoltage);
            xSemaphoreGive(VBusVoltage_semaphore);
            break;

        default:
            s_unknown_id = msg.id;   // reported by report_bus_errors()
            s_unknown_count++;
            break;
        }
    }
}

static twai_node_handle_t s_odrive_node = NULL;   // set once the TWAI node is up

// ODrive faults: an error, or the ODrive out of closed loop while the machine runs. The errors are
// cleared and closed loop asked for FAULT_AUTO_TRIES times, FAULT_RETRY_MS apart; after that the
// main button (odrive_fault_retry) allows that many more.
#define FAULT_AUTO_TRIES    3
#define FAULT_RETRY_MS      2000
volatile uint8_t g_odrive_fault_error = OM_ODRIVE_ERROR_NONE;   // sent to the manipulator as odrive_error
static volatile bool s_fault_retry = false;

void odrive_fault_retry(void)
{
    s_fault_retry = true;
}

// The highest set bit of the ODrive's errors, as the manipulator names them
static uint8_t odrive_error_bit(uint32_t errors)
{
    return errors ? (uint8_t)(31 - __builtin_clz(errors)) : OM_ODRIVE_ERROR_NONE;
}

// Send one CANSimple command to the ODrive. False if the previous one hasn't gone out yet (the
// ODrive isn't ACKing, e.g. while it reboots) or the TX queue is full: try again.
bool odrive_can_send(uint32_t cmd, const uint8_t *data, uint8_t len)
{
    if (s_odrive_node == NULL || s_generic_in_flight || len > TWAI_FRAME_MAX_LEN) {
        return false;
    }
    if (len > 0) {
        memcpy(s_generic_payload, data, len);
    }
    s_generic_frame.header.id = (ODRIVE_NODE_ID << 5) | cmd;
    s_generic_frame.buffer_len = len;
    s_generic_in_flight = true;
    if (twai_node_transmit(s_odrive_node, &s_generic_frame, TWAI_TX_QUEUE_TIMEOUT_MS) != ESP_OK) {
        s_generic_in_flight = false;
        return false;
    }
    twai_node_transmit_wait_all_done(s_odrive_node, TWAI_TX_DONE_TIMEOUT_MS);
    return true;
}

// ODrive interface functions
void set_axis_requested_state (twai_node_handle_t sender_node, uint32_t Axis_Requested_State) {

        if (s_axis_state_in_flight) {
            ESP_LOGW(TAG, "TX Axis Requested State: previous frame still pending, skipped");
            return;
        }
        s_axis_state_payload = Axis_Requested_State;
        s_axis_state_in_flight = true;
        if (twai_node_transmit(sender_node, &s_axis_state_frame, TWAI_TX_QUEUE_TIMEOUT_MS) != ESP_OK) {
            s_axis_state_in_flight = false;
            ESP_LOGW(TAG, "TX Axis Requested State: queue full, frame dropped");
            return;
        }
        ESP_LOGI(TAG, "TX Axis Requested State: %x [%d]", \
                            s_axis_state_frame.header.id, Axis_Requested_State);
        if (twai_node_transmit_wait_all_done(sender_node, TWAI_TX_DONE_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "TX Axis Requested State: not sent in time");
        }

}

// Returns false if the frame couldn't be queued, so the caller can try again
static bool set_limits(twai_node_handle_t sender_node, float vel_limit, float current_limit) {

    if (s_limits_in_flight) {
        return false;
    }
    s_limits_payload.vel_limit = vel_limit;
    s_limits_payload.current_limit = current_limit;
    s_limits_in_flight = true;
    if (twai_node_transmit(sender_node, &s_limits_frame, TWAI_TX_QUEUE_TIMEOUT_MS) != ESP_OK) {
        s_limits_in_flight = false;
        ESP_LOGW(TAG, "TX Limits: queue full, frame dropped");
        return false;
    }
    ESP_LOGI(TAG, "TX Limits: vel %.2f rev/s, current %.1f A", vel_limit, current_limit);
    return true;
}

void set_position (twai_node_handle_t sender_node, float position_setpoint) {

    if (s_position_in_flight) {
#if TIMING_DEBUG
        s_pos_skipped++;
#endif
        ESP_LOGW(TAG, "TX Position Setpoint: previous frame still pending, skipped");
        return;
    }
    // Set_Input_Pos is 8 bytes: position, velocity feed-forward, torque feed-forward
    s_position_payload.position = position_setpoint;
    s_position_payload.Vel_FF = 0;
    s_position_payload.Torque_FF = 0;
    s_position_in_flight = true;
    if (twai_node_transmit(sender_node, &s_position_frame, TWAI_TX_QUEUE_TIMEOUT_MS) != ESP_OK) {
        s_position_in_flight = false;
        ESP_LOGW(TAG, "TX Position Setpoint: queue full, frame dropped");
        return;
    }
    ESP_LOGI(TAG, "TX Position Setpoint: %x [%f]", \
                        s_position_frame.header.id, position_setpoint);
    if (twai_node_transmit_wait_all_done(sender_node, TWAI_TX_DONE_TIMEOUT_MS) != ESP_OK) {
#if TIMING_DEBUG
        s_pos_late++;
#endif
        ESP_LOGW(TAG, "TX Position Setpoint: not sent in time");
    }

}


// Battery gauge for the manipulator display: 6S LiPo, resting voltage per cell -> percent
#define BATTERY_CELLS        6
#define VBUS_FILTER_ALPHA    0.3f    // smoothing of the published voltage (motor current makes VBus sag)

static int32_t battery_percent(float vbus)
{
    static const struct { float volts; float pct; } curve[] = {
        {3.27f, 0}, {3.61f, 5}, {3.69f, 10}, {3.71f, 15}, {3.73f, 20}, {3.75f, 25},
        {3.77f, 30}, {3.79f, 35}, {3.80f, 40}, {3.82f, 45}, {3.84f, 50}, {3.85f, 55},
        {3.87f, 60}, {3.91f, 65}, {3.95f, 70}, {3.98f, 75}, {4.02f, 80}, {4.08f, 85},
        {4.11f, 90}, {4.15f, 95}, {4.20f, 100},
    };
    const int n = sizeof(curve) / sizeof(curve[0]);
    float cell = vbus / BATTERY_CELLS;
    if (cell <= curve[0].volts) {
        return 0;
    }
    if (cell >= curve[n - 1].volts) {
        return 100;
    }
    for (int i = 1; i < n; i++) {
        if (cell < curve[i].volts) {
            float t = (cell - curve[i - 1].volts) / (curve[i].volts - curve[i - 1].volts);
            return (int32_t)lroundf(curve[i - 1].pct + t * (curve[i].pct - curve[i - 1].pct));
        }
    }
    return 100;
}

// Ask machine_state_task for a state change; it is the only task that changes the state
static void request_state_event(button_type_t type)
{
    button_event_t ev = { .type = type, .rotary_delta = 0 };
    if (xQueueSend(s_machine_ctx.buttons_queue, &ev, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "State request %d dropped, buttons queue full", type);
    }
}

#if COMPLIANCE_ENABLE
static float s_stiff_pos_gain, s_stiff_vel_gain, s_vel_integrator_gain;   // from odrive-s1-config.json

// Send the gains for a stiffness of 0..1. False if they didn't both go: try again next pass.
static bool send_gains(float stiffness)
{
    float pos = SOFT_POS_GAIN * powf(s_stiff_pos_gain / SOFT_POS_GAIN, stiffness);
    float vel = SOFT_VEL_GAIN * powf(s_stiff_vel_gain / SOFT_VEL_GAIN, stiffness);
    uint8_t d[8];
    memcpy(d, &pos, sizeof(pos));
    if (!odrive_can_send(SET_POS_GAIN, d, 4)) {
        return false;
    }
    memcpy(d, &vel, sizeof(vel));
    memcpy(d + 4, &s_vel_integrator_gain, sizeof(s_vel_integrator_gain));
    return odrive_can_send(SET_VEL_GAINS, d, 8);
}
#endif

// Sender task function
void odrive_task(void *pvParameters) {
    twai_node_handle_t node = (twai_node_handle_t)pvParameters;
    float initial_position = 0.0;
    // Snapshot of the machine state and settings, refreshed every loop pass from
    // s_machine_ctx / g_machine_elements (machine_state_task owns both)
    odrive_control_state_t control = {0};
    
    // Send closed loop command until axis is in closed loop. Not while "Set up ODrive" runs, which
    // can be started from here too (e.g. an ODrive that won't calibrate); the controller restarts after it.
    // An ODrive reporting an error has it cleared, a few times; if it won't clear, the manipulator shows
    // the fault and its main button allows more tries.
    int start_clears = 0;
    TickType_t last_clear = 0;
    while(AxisStatus != CLOSED_LOOP_CONTROL) {
        if (odrive_setup_active()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (s_fault_retry) {
            s_fault_retry = false;
            start_clears = 0;
        }
        if (AxisError != 0) {
            g_odrive_fault_error = odrive_error_bit(AxisError);
            TickType_t now = xTaskGetTickCount();
            if (start_clears < FAULT_AUTO_TRIES) {
                if (start_clears == 0 || now - last_clear >= pdMS_TO_TICKS(FAULT_RETRY_MS)) {
                    uint8_t d[1] = {0};
                    odrive_can_send(CLEAR_ERRORS, d, sizeof(d));
                    ESP_LOGW(TAG, "ODrive error 0x%" PRIx32 " at start-up: cleared (%d)", AxisError, start_clears + 1);
                    start_clears++;
                    last_clear = now;
                }
            } else if (machine_state_get(&s_machine_ctx) != MACHINE_ODRIVE_FAULT) {
                ESP_LOGE(TAG, "ODrive error 0x%" PRIx32 " won't clear: fault", AxisError);
                request_state_event(BUTTON_ODRIVE_FAULT);
            }
        }
        set_axis_requested_state(node, CLOSED_LOOP_CONTROL);
        ESP_LOGI(TAG, "Waiting for Axis to enter CLOSED_LOOP_CONTROL state. Current State: %d", AxisStatus);
        report_bus_errors();
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100 ms between checks
    }

    g_odrive_fault_error = OM_ODRIVE_ERROR_NONE;
    request_state_event(BUTTON_ODRIVE_RECOVERED);   // back to INITIAL, if the start-up had a fault shown
    request_state_event(BUTTON_HOMING_STARTED);

    // Send position setpoints in a loop
    if (xSemaphoreTake(EncoderEstimate_semaphore, -1) == pdTRUE) {
        initial_position = EncoderEstimate;
        // Temporary hack to set extended and contracted positions
        EncoderExtended = initial_position + 0.5; // 0.5 rad extension
        EncoderContracted = initial_position - 0.5; // 0.5 rad contraction
        EncoderExtensionsDone = true;
        ESP_LOGI(TAG, "Initial Encoder Position: %f", initial_position);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second to simulate homing delay

    request_state_event(BUTTON_HOMING_DONE);

    TickType_t last_vbus_tick = xTaskGetTickCount();
    encoder_estimate_msg_t enc_msg;

    // Auto stroke timing: stroke time advances at a speed-scaled rate from 0 at AUTO entry,
    // so strokes start at zero offset and speed changes don't jump the phase
    machine_state_t prev_state = MACHINE_INITIAL;
    bool sync_requested = false;  // PREAUTO/SYNCING sync request sent for this state entry
    TickType_t last_tick = xTaskGetTickCount();
    float stroke_ms = 0.0f;
    float hold_position = 0.0f;
    // Changing stroke in AUTO: glide at STROKE_CHANGE_SPEED from the last target to the new
    // stroke's start, then start it from zero, instead of jumping into the middle of it
    int32_t prev_stroke_index = -1;
    bool stroke_changing = false;
    float last_target = EncoderEstimate;   // the last target sent; starts where the ODrive is
    // Entering MANUAL from SYNCING, the ODrive sits a little off hold_position (the error it holds
    // the load with), and MANUAL's target lands on where it actually is. Ease the target over at
    // MANUAL_EASE_SPEED so the holding force fades instead of dropping in one frame.
    bool manual_easing = false;
    int32_t sent_power_pct = -1;   // Power the current limit was last sent for; -1: not sent yet
    int fault_tries = 0;
    TickType_t fault_last_try = 0;
#if COMPLIANCE_ENABLE
    s_stiff_pos_gain = odrive_setup_config_float("axis0.controller.config.pos_gain", 80.0f);
    s_stiff_vel_gain = odrive_setup_config_float("axis0.controller.config.vel_gain", 1.4f);
    s_vel_integrator_gain = odrive_setup_config_float("axis0.controller.config.vel_integrator_gain", 0.0f);
    float stiffness = 1.0f;          // the ODrive starts with its saved (stiff) gains
    float stiffness_sent = -1.0f;    // -1: none sent yet
    float handle_vel = 0.0f;         // turns/s, smoothed
    float prev_handle = NAN;
    float held_s = 0.0f, keep_s = 0.0f;
#endif

    // Block for next manipulator position
    while (xQueueReceive(manipulator_encoder_queue, &enc_msg, -1) == pdTRUE) {
#if TIMING_DEBUG
        int64_t loop_start_us = esp_timer_get_time();
#endif
        if (odrive_setup_active()) {
            continue;   // odrive_setup.c has the ODrive: no setpoints or limits
        }
        float controller_target = C_ManipulatorEncoder_OdriveEncoder(enc_msg.value);
        //ESP_LOGI(TAG, "Recieved manipulator encoder: %f", controller_target);
        control.machine_state = machine_state_get(&s_machine_ctx);
        control.speed_pct = machine_element_value(ELEMENT_SPEED);
        control.length_pct = machine_element_value(ELEMENT_LENGTH);
        control.stroke_index = machine_element_value(ELEMENT_STROKE_INDEX);

        TickType_t now_tick_loop = xTaskGetTickCount();
        uint32_t dt_ms = (uint32_t)((now_tick_loop - last_tick) * portTICK_PERIOD_MS);
        last_tick = now_tick_loop;

        // The ODrive out of closed loop while the machine runs (an error disarmed it): stop sending
        // setpoints and show the fault. Not during a firmware update, which puts it in IDLE itself.
        if (control.machine_state == MACHINE_ODRIVE_FAULT) {
            if (s_fault_retry) {
                s_fault_retry = false;
                fault_tries = 0;
                fault_last_try = now_tick_loop - pdMS_TO_TICKS(FAULT_RETRY_MS);   // try straight away
            }
            if (AxisStatus == CLOSED_LOOP_CONTROL) {
                ESP_LOGW(TAG, "ODrive running again");
                g_odrive_fault_error = OM_ODRIVE_ERROR_NONE;
                request_state_event(BUTTON_ODRIVE_RECOVERED);   // SYNCING: hold where it is, the handle comes over
            } else {
                if (AxisError != 0) {
                    g_odrive_fault_error = odrive_error_bit(AxisError);
                }
                if (fault_tries < FAULT_AUTO_TRIES && now_tick_loop - fault_last_try >= pdMS_TO_TICKS(FAULT_RETRY_MS)) {
                    // Hold where it is now (it may have been moved while off), then clear and arm
                    uint8_t d[1] = {0};
                    odrive_can_send(CLEAR_ERRORS, d, sizeof(d));
                    set_position(node, EncoderEstimate);
                    set_axis_requested_state(node, CLOSED_LOOP_CONTROL);
                    fault_tries++;
                    fault_last_try = now_tick_loop;
                    ESP_LOGW(TAG, "ODrive fault: errors cleared, closed loop asked for (try %d)", fault_tries);
                }
            }
            prev_state = MACHINE_ODRIVE_FAULT;   // so SYNCING afterwards takes the hold position afresh
            last_target = EncoderEstimate;
            continue;
        }
        bool running = control.machine_state == MACHINE_SYNCING || control.machine_state == MACHINE_MANUAL ||
                       control.machine_state == MACHINE_PREAUTO || control.machine_state == MACHINE_AUTO ||
                       control.machine_state == MACHINE_DISCONNECTED;
        if (running && AxisStatus != CLOSED_LOOP_CONTROL && !g_firmware_update_active) {
            g_odrive_fault_error = odrive_error_bit(AxisError);
            ESP_LOGE(TAG, "ODrive left closed loop (state %d, errors 0x%" PRIx32 ") in machine state %d",
                     AxisStatus, AxisError, control.machine_state);
            request_state_event(BUTTON_ODRIVE_FAULT);
            fault_tries = 0;
            fault_last_try = now_tick_loop - pdMS_TO_TICKS(FAULT_RETRY_MS) + pdMS_TO_TICKS(1000);   // first try in 1 s
            continue;
        }

        if (control.machine_state != prev_state) {
            sync_requested = false;
            if (control.machine_state == MACHINE_AUTO) {
                stroke_ms = 0.0f;
                prev_stroke_index = control.stroke_index;
                stroke_changing = false;
            } else if (control.machine_state == MACHINE_SYNCING) {
                hold_position = EncoderEstimate;  // stop where we are
            } else if (control.machine_state == MACHINE_MANUAL) {
                manual_easing = true;
            }
            prev_state = control.machine_state;
        }

        float span = EncoderExtended - EncoderContracted;
        float target = controller_target;
        float offset = 0.0f;

        switch (control.machine_state) {
        case MACHINE_MANUAL:
            target = EncoderEstimate + MANUAL_HANDLE_WEIGHT * (controller_target - EncoderEstimate);
            if (manual_easing) {
                float step = MANUAL_EASE_SPEED * fabsf(span) * (float)dt_ms / 1000.0f;
                if (fabsf(target - last_target) <= step) {
                    manual_easing = false;   // caught up: follow the handle normally from now on
                } else {
                    target = last_target + (target > last_target ? step : -step);
                }
            }
            break;
        case MACHINE_PREAUTO:
            // Bring the ODrive onto the handle, then start stroking from there
            target = controller_target;
            if (!sync_requested && fabsf(EncoderEstimate - controller_target) < SYNC_TOLERANCE * fabsf(span)) {
                request_state_event(BUTTON_ODRIVE_SYNCED);
                sync_requested = true;
            }
            break;
        case MACHINE_AUTO: {
            // The stroke runs from the handle position toward Extended; shorten it so it ends at
            // Extended at most, rather than being flattened against the clamp below
            float stroke_len = span * (control.length_pct / 100.0f);
            stroke_len = fmaxf(fminf(stroke_len, EncoderExtended - controller_target), 0.0f);
            if (control.stroke_index != prev_stroke_index) {
                prev_stroke_index = control.stroke_index;
                stroke_changing = true;
            }
            if (stroke_changing) {
                // Every stroke starts at zero offset, i.e. at the handle position
                float start = controller_target;
                float step = STROKE_CHANGE_SPEED * fabsf(span) * (float)dt_ms / 1000.0f;
                if (fabsf(start - last_target) <= step) {
                    target = start;
                    stroke_changing = false;
                    stroke_ms = 0.0f;
                } else {
                    target = last_target + (start > last_target ? step : -step);
                }
                break;
            }
            stroke_ms += (float)dt_ms * (float)(control.speed_pct + 1) / 20.0f;
            offset = stroke_eval((stroke_id_t)control.stroke_index, (uint32_t)stroke_ms, stroke_len);
            target = controller_target + offset;
            break;
        }
        case MACHINE_SYNCING: {
            // After homing, stopping AUTO, a cancelled PREAUTO or a link loss: hold the ODrive still while the manipulator
            // drives the handle over to it (the manipulator follows the ODrive position it is sent). The handle can only reach
            // the stroke span, so compare against the hold position clamped to that range.
            target = hold_position;
            float lo = fminf(EncoderContracted, EncoderExtended);
            float hi = fmaxf(EncoderContracted, EncoderExtended);
            float reachable = fminf(fmaxf(hold_position, lo), hi);
            if (!sync_requested && fabsf(controller_target - reachable) < SYNC_TOLERANCE * fabsf(span)) {
                request_state_event(BUTTON_HANDLE_SYNCED);
                sync_requested = true;
            }
            break;
        }
        case MACHINE_DISCONNECTED:
        default:
            target = EncoderEstimate;
            break;
        }
        // Safety net: never command the ODrive outside the Contracted..Extended span
        if (target < EncoderContracted) {
            target = EncoderContracted;
        } else if (target > EncoderExtended) {
            target = EncoderExtended;
        }
        last_target = target;

#if COMPLIANCE_ENABLE
        {
            float dt_s = (float)dt_ms / 1000.0f;
            if (dt_s > 0.0f) {
                if (!isnan(prev_handle)) {
                    float raw = (controller_target - prev_handle) / dt_s;
                    handle_vel += (dt_s / (HANDLE_VEL_FILTER_S + dt_s)) * (raw - handle_vel);
                }
                prev_handle = controller_target;
            }
            if (control.machine_state == MACHINE_MANUAL && !manual_easing) {
                float gap = controller_target - EncoderEstimate;   // handle minus ODrive
                bool moving = fabsf(handle_vel) > HANDLE_MOVING_TPS;
                if (moving && handle_vel * gap > 0.0f) {
                    // the handle moving away from the carriage: it's leading, so follow it closely
                    stiffness += dt_s / LEAD_STIFFEN_S;
                    held_s = 0.0f;
                    keep_s = 0.0f;
                } else if (!moving && fabsf(gap) > HELD_GAP_TURNS) {
                    // the carriage pushed away from a handle that isn't following: it's held
                    held_s += dt_s;
                    if (held_s >= HELD_DELAY_S) {
                        stiffness += dt_s / HELD_STIFFEN_S;
                        keep_s = HELD_KEEP_S;
                    }
                } else {
                    // the handle following the carriage, or everything at rest
                    held_s = 0.0f;
                    if (keep_s > 0.0f) {
                        keep_s -= dt_s;
                    } else {
                        stiffness -= dt_s / SOFTEN_S;
                    }
                }
            } else {
                stiffness = 1.0f;   // every other state (and easing into MANUAL) stiff, as saved
                held_s = 0.0f;
                keep_s = 0.0f;
            }
            stiffness = fminf(fmaxf(stiffness, 0.0f), 1.0f);
            bool at_end = (stiffness == 0.0f || stiffness == 1.0f) && stiffness != stiffness_sent;
            if ((fabsf(stiffness - stiffness_sent) >= GAIN_SEND_STEP || at_end) && send_gains(stiffness)) {
                stiffness_sent = stiffness;
            }
        }
#endif

        // Power sets the ODrive's current limit: sent on the first pass and whenever it changes
        int32_t power_pct = machine_element_value(ELEMENT_POWER);
        if (power_pct != sent_power_pct) {
            float current = fmaxf(MAX_CURRENT_A * (float)power_pct / 100.0f, MIN_CURRENT_A);
            if (set_limits(node, VEL_LIMIT, current)) {
                sent_power_pct = power_pct;
            }
        }

        set_position(node, target);
        ESP_LOGI(TAG, "Sending Odrive Target: %f (%f)", controller_target, target);
#if TRACK_DEBUG
        {
            static int64_t last_track_us = 0;
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_track_us >= TRACK_DEBUG_MS * 1000) {
                last_track_us = now_us;
#if COMPLIANCE_ENABLE
                int stiff_pct = (int)lroundf(stiffness_sent * 100.0f);
#else
                int stiff_pct = 100;
#endif
                ESP_LOGW(TAG, "TRK %ld %ld %ld %ld %d %d", (long)(now_us / 1000), lroundf(controller_target * 1e5f),
                         lroundf(target * 1e5f), lroundf(EncoderEstimate * 1e5f), (int)control.machine_state, stiff_pct);
            }
        }
#endif
#if TIMING_DEBUG
        {
            static int64_t window_start_us = 0, last_set_us = 0;
            static uint32_t sets = 0, max_gap_us = 0, max_set_us = 0;
            static float min_target = 1e9f, max_target = -1e9f;
            int64_t now_us = esp_timer_get_time();
            if (last_set_us != 0 && (uint32_t)(now_us - last_set_us) > max_gap_us) {
                max_gap_us = (uint32_t)(now_us - last_set_us);
            }
            uint32_t set_us = (uint32_t)(now_us - loop_start_us);
            if (set_us > max_set_us) {
                max_set_us = set_us;
            }
            last_set_us = now_us;
            sets++;
            min_target = fminf(min_target, target);
            max_target = fmaxf(max_target, target);
            if (now_us - window_start_us >= 1000000) {
                ESP_LOGW(TAG, "[timing] state %d: %" PRIu32 " setpoints/s, max gap %" PRIu32 " ms, "
                         "max loop work %" PRIu32 " ms, skipped %" PRIu32 ", late %" PRIu32 ", target %.3f..%.3f, est %.3f",
                         control.machine_state, sets, max_gap_us / 1000, max_set_us / 1000,
                         s_pos_skipped, s_pos_late, min_target, max_target, EncoderEstimate);
                window_start_us = now_us;
                sets = 0; max_gap_us = 0; max_set_us = 0;
                s_pos_skipped = 0; s_pos_late = 0;
                min_target = 1e9f; max_target = -1e9f;
            }
        }
#endif
        // Periodic VBus publish to the manipulator's display. The battery stays OM_BATTERY_UNKNOWN
        // (not shown) until the ODrive has reported its bus voltage, then shows as soon as it has.
        TickType_t now_tick = xTaskGetTickCount();
        static float vbus_filtered = 0.0f;
        bool first_vbus = vbus_filtered == 0.0f && VBusVoltage > 0.0f;
        if (first_vbus || (now_tick - last_vbus_tick) > pdMS_TO_TICKS(2000)) {
            report_bus_errors();
            if (VBusVoltage > 0.0f) {
                vbus_filtered = (vbus_filtered == 0.0f) ? VBusVoltage
                              : vbus_filtered + VBUS_FILTER_ALPHA * (VBusVoltage - vbus_filtered);
                int32_t pct = battery_percent(vbus_filtered);
                g_machine_elements[ELEMENT_VBUS].current_value = pct;  // sent to the manipulator with every frame
            }
            last_vbus_tick = now_tick;
        }
        // No delay here: the loop is paced by manipulator replies (POLL_PERIOD_MS in espnow_manipulator.cpp)
    }
}

// A firmware update is about to write flash: put the ODrive in IDLE (motor off) first, so it isn't
// left holding position while the flash writes hold up this board. The controller restarts after
// the update, and the ODrive goes back to closed loop as at any start.
void odrive_idle_for_update(void)
{
    if (s_odrive_node == NULL) {
        return;
    }
    for (int i = 0; i < 3 && AxisStatus != AXIS_STATE_IDLE; i++) {
        set_axis_requested_state(s_odrive_node, AXIS_STATE_IDLE);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGW(TAG, "ODrive set to IDLE for the firmware update (state %d)", (int)AxisStatus);
}

void app_main(void)
{
    log_async_start();        // first: from here on, logging never holds up the task that logs
    firmware_valid_start();   // a new image from an update is on trial until the link comes up
    // Queues and context for machine state flow
    // - buttons_queue: input events from manipulator
    // Element values (g_machine_elements) go to the manipulator in every ESP-NOW frame
    // The ODrive control loop reads the state and element values directly (see odrive_task)
    s_machine_ctx = (machine_state_ctx_t){0};
    machine_settings_load();   // saved Power, Speed, Length and Stroke, before anything reads them
    machine_state_init_defaults(&s_machine_ctx);
    s_machine_ctx.buttons_queue = xQueueCreate(16, sizeof(button_event_t));
    assert(s_machine_ctx.buttons_queue != NULL);

    // The ESP-NOW task uses both straight away, so they exist before it starts
    odrive_encoder_queue = xQueueCreate(1, sizeof(encoder_estimate_msg_t));
    assert(odrive_encoder_queue != NULL);
    manipulator_encoder_queue = xQueueCreate(1, sizeof(encoder_estimate_msg_t));
    assert(manipulator_encoder_queue != NULL);

    // Start machine-state processing first so button events are consumed
    xTaskCreate(machine_state_task, "Machine_State", 4096, &s_machine_ctx, 10, NULL);
    status_led_start(&s_machine_ctx);   // the dev board's RGB LED shows the state

    //printf("===================Serial Starting...===================\n");
    // Create ESP-NOW manipulator task with machine context
    xTaskCreate(espnow_manipulator_task, "ESPNOW_Manipulator", 8192, &s_machine_ctx, 10, NULL);


    printf("===================TWAI Starting...===================\n");
    // Per-message INFO logs block the control loop on the UART console; show warnings and errors only
    esp_log_level_set(TAG, ESP_LOG_WARN);
    // Create semaphore for receive notification
    twai_ctx_t twai_ctx = {0};
    twai_ctx.free_pool_semaphore = xSemaphoreCreateCounting(POLL_DEPTH, POLL_DEPTH);
    twai_ctx.rx_result_semaphore = xSemaphoreCreateCounting(POLL_DEPTH, 0);
    EncoderEstimate_semaphore = xSemaphoreCreateBinary();
    VBusVoltage_semaphore = xSemaphoreCreateBinary();
    AxisStatus_semaphore = xSemaphoreCreateBinary();

    assert(twai_ctx.free_pool_semaphore != NULL);
    assert(twai_ctx.rx_result_semaphore != NULL);
    assert(EncoderEstimate_semaphore != NULL);
    assert(VBusVoltage_semaphore != NULL);
    assert(AxisStatus_semaphore != NULL);
    twai_rx_queue = xQueueCreate(TWAI_RX_QUEUE_LEN, sizeof(twai_rx_msg_t));
    assert(twai_rx_queue != NULL);

    twai_ctx.rx_pool = calloc(POLL_DEPTH, sizeof(twai_listener_data_t));
    assert(twai_ctx.rx_pool != NULL);
    for (int i = 0; i < POLL_DEPTH; i++) {
        twai_ctx.rx_pool[i].frame.buffer = twai_ctx.rx_pool[i].data;
        twai_ctx.rx_pool[i].frame.buffer_len = sizeof(twai_ctx.rx_pool[i].data);
    }
    ESP_LOGI(TAG, "Buffer initialized: %d slots for burst data", POLL_DEPTH);

    // Configure TWAI node
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_LISTENER_TX_GPIO,
            .rx = TWAI_LISTENER_RX_GPIO,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .tx_queue_depth = 32,
        .bit_timing.bitrate = TWAI_BITRATE,
        .flags.enable_listen_only = false,
        .fail_retry_cnt = -1,
    };

    // Create TWAI node
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &twai_ctx.node_hdl));
    ESP_LOGI(TAG, "TWAI node created");


    // Register callbacks
    twai_event_callbacks_t callbacks = {
        .on_tx_done = twai_sender_tx_done_callback,
        .on_rx_done = twai_listener_rx_callback,
        .on_error = twai_listener_on_error_callback,
        .on_state_change = twai_listener_on_state_change_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_ctx.node_hdl, &callbacks, &twai_ctx));

    // Enable TWAI node
    ESP_ERROR_CHECK(twai_node_enable(twai_ctx.node_hdl));
    ESP_LOGI(TAG, "TWAI start listening...");

    s_odrive_node = twai_ctx.node_hdl;
    odrive_setup_init();

    // Create twai sender task
    xTaskCreate(odrive_task, "TWAI_Sender", 4096, twai_ctx.node_hdl, 10, NULL);

    xTaskCreate(twai_process_task, "TWAI_Process", 4096, NULL, 10, NULL);

    // Main task can now suspend or handle other things
    vTaskDelete(NULL);

    // Cleanup (unreachable in this case)
    vSemaphoreDelete(twai_ctx.rx_result_semaphore);
    vSemaphoreDelete(twai_ctx.free_pool_semaphore);
    vSemaphoreDelete(EncoderEstimate_semaphore);
    vSemaphoreDelete(VBusVoltage_semaphore);
    vSemaphoreDelete(AxisStatus_semaphore);

    free(twai_ctx.rx_pool);
    ESP_ERROR_CHECK(twai_node_disable(twai_ctx.node_hdl));
    ESP_ERROR_CHECK(twai_node_delete(twai_ctx.node_hdl));
}
