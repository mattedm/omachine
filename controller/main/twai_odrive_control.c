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
#define GET_ERROR               3   // asked for by remote frame: active errors, disarm reason
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
#define VEL_LIMIT           15.0f // rev/s, as axis0.controller.config.vel_limit
#define MAX_CURRENT_A       40.0f // Power 100% = this many amps
#define MIN_CURRENT_A       2.0f  // floor, so low Power never leaves the ODrive limp
// Homing: drive slowly into each end of the rail at a low current; an end is reached once the
// position stops moving (as the manipulator finds its end stops). The ends, less HOMING_BACKOFF,
// become the stroke's span.
#define HOMING_SPEED        1.0f   // turns/s
#define HOMING_CURRENT_A    8.0f   // moves the carriage up a vertical rail (about 1.6 A holds it there); 4 A stalled on the way
#define HOMING_LEAD         0.2f   // the target runs at most this far (turns) ahead of the carriage
#define HOMING_CHECK_MS     300    // how often movement is checked
#define HOMING_STALL_TURNS  0.01f  // moved less than this in HOMING_CHECK_MS: at the end
#define HOMING_SETTLE_MS    500    // let it get going before checking for a stall
#define HOMING_MAX_TURNS    40.0f  // no end within this: the carriage isn't on the rail (or the belt is off)
#define HOMING_MIN_TRAVEL   1.0f   // ends closer than this (turns): it never really moved
#define HOMING_BACKOFF      0.1f   // keep this far (turns) inside each end
#define HOMING_FALLBACK     0.5f   // homing failed: a span this far (turns) either side of where it is
// Auto: the stroke's target passes through a limiter, so no stroke at any Speed asks for more than
// the motor can do. Without it, Heartbeat at high Speed asked for over 15 turns/s and braked so hard
// from there that the ODrive tripped its spinout check. Within these limits it's followed exactly.
#define AUTO_VEL_MAX        12.0f  // turns/s, under VEL_LIMIT
#define AUTO_ACCEL_MAX      300.0f // turns/s^2 (the default-Speed strokes stay under it)
#define AUTO_TRACK_S        0.02f  // how quickly the limited target closes a gap to the stroke
#define LOOP_PERIOD_MS      5      // a setpoint at least this often, handle reading or not (the link polls at the same rate)
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
// The stiffness follows the gap between the handle and the carriage: soft while they're close
// (the handle following the carriage freely), rising smoothly as they part (the handle held while
// the carriage is pushed, or the handle leading), so the carriage is drawn back like a spring that
// firms up the further it's stretched.
#define GAP_SOFT_TURNS       0.02f   // up to this gap (turns): fully soft (the handle's normal lag)
#define GAP_STIFF_TURNS      0.15f   // from this gap: fully stiff; an S-curve between the two
#define STIFFEN_S            0.03f   // the stiffness rises towards the gap's with this time constant
#define SOFTEN_S             0.25f   // and falls with this one, slower, so it doesn't pump as the gap closes
// Settling: once the carriage stops being pushed away from the handle, the stiffness builds over
// time until the carriage reaches the handle, so a small offset (or gravity's sag on an upright
// rail) doesn't stay springy. Pushing it away again lets go at once; once there, it relaxes.
#define SETTLE_GAP_TURNS     0.005f  // a gap above this gets settled
#define SETTLE_DONE_TURNS    0.002f  // within this, it has arrived: the settling relaxes again
#define SETTLE_DELAY_S       0.5f    // not pushed away for this long before settling starts
#define SETTLE_S             1.0f    // then up to fully stiff over this long
#define SETTLE_RELEASE_S     0.05f   // pushed away from the handle: the settling lets go this quickly
#define CARRIAGE_MOVING_TPS  0.05f   // the carriage faster than this (turns/s) counts as moving
#define CARRIAGE_VEL_FILTER_S 0.03f  // smoothing of the carriage's speed
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
// The fault the manipulator shows: the ODrive's active errors and disarm reason, from its Get_Error
// reply (or the heartbeat's errors until that comes), kept until it runs again
static volatile uint32_t s_fault_errors = 0;
static volatile uint32_t s_fault_disarm = 0;
static volatile bool s_fault_retrying = false;   // automatic tries left
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
        case GET_ERROR: {
            uint32_t active, disarm;
            memcpy(&active, &msg.data[0], sizeof(active));
            memcpy(&disarm, &msg.data[4], sizeof(disarm));
            if (active | disarm) {   // keep the last cause; an empty reply (errors cleared) doesn't erase it
                s_fault_errors = active;
                s_fault_disarm = disarm;
            }
            break;
        }
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
#define FAULT_QUERY_MS      300    // how often Get_Error is asked for while there's a fault
static volatile bool s_fault_retry = false;

void odrive_fault_retry(void)
{
    s_fault_retry = true;
}

static bool odrive_can_request(uint32_t cmd);

// A fault is showing: ask the ODrive for its errors now and then (the reply lands in s_fault_*),
// with the heartbeat's errors meanwhile; retrying says whether automatic tries are left. Logs a change.
static void fault_note(bool retrying)
{
    static TickType_t last_query = 0;
    static uint32_t logged_errors = 0, logged_disarm = 0;
    TickType_t now = xTaskGetTickCount();
    if (now - last_query >= pdMS_TO_TICKS(FAULT_QUERY_MS)) {
        odrive_can_request(GET_ERROR);
        last_query = now;
    }
    if (s_fault_errors == 0 && s_fault_disarm == 0 && AxisError != 0) {
        s_fault_errors = AxisError;
    }
    s_fault_retrying = retrying;
    if (s_fault_errors != logged_errors || s_fault_disarm != logged_disarm) {
        ESP_LOGE(TAG, "ODrive fault: active errors 0x%" PRIx32 ", disarm reason 0x%" PRIx32 ", bus %.1f V",
                 s_fault_errors, s_fault_disarm, VBusVoltage);
        logged_errors = s_fault_errors;
        logged_disarm = s_fault_disarm;
    }
}

// Running again: nothing to show
static void fault_clear(void)
{
    s_fault_errors = 0;
    s_fault_disarm = 0;
    s_fault_retrying = false;
}

void odrive_fault_report(uint32_t *errors, uint32_t *disarm, uint16_t *vbus_dv, uint8_t *flags)
{
    *errors = s_fault_errors;
    *disarm = s_fault_disarm;
    float v = VBusVoltage * 10.0f + 0.5f;
    *vbus_dv = (v <= 0.0f) ? 0 : (v >= 65535.0f ? 65535 : (uint16_t)v);
    *flags = s_fault_retrying ? OM_FAULT_RETRYING : 0;
}

// Send one CANSimple command to the ODrive. False if the previous one hasn't gone out yet (the
// ODrive isn't ACKing, e.g. while it reboots) or the TX queue is full: try again.
// A data frame, or (rtr) a remote frame asking for an 8-byte reply
static bool odrive_can_frame(uint32_t cmd, const uint8_t *data, uint8_t len, bool rtr)
{
    if (s_odrive_node == NULL || s_generic_in_flight || len > TWAI_FRAME_MAX_LEN) {
        return false;
    }
    if (len > 0) {
        memcpy(s_generic_payload, data, len);
    }
    s_generic_frame.header.id = (ODRIVE_NODE_ID << 5) | cmd;
    s_generic_frame.header.rtr = rtr;
    s_generic_frame.header.dlc = rtr ? 8 : 0;
    s_generic_frame.buffer_len = len;
    s_generic_in_flight = true;
    if (twai_node_transmit(s_odrive_node, &s_generic_frame, TWAI_TX_QUEUE_TIMEOUT_MS) != ESP_OK) {
        s_generic_in_flight = false;
        return false;
    }
    twai_node_transmit_wait_all_done(s_odrive_node, TWAI_TX_DONE_TIMEOUT_MS);
    return true;
}

bool odrive_can_send(uint32_t cmd, const uint8_t *data, uint8_t len)
{
    return odrive_can_frame(cmd, data, len, false);
}

static bool odrive_can_request(uint32_t cmd)
{
    return odrive_can_frame(cmd, NULL, 0, true);
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

// Drive towards one end (dir +1 or -1) until the carriage stops; *end gets where it stopped.
// False if it ran HOMING_MAX_TURNS without stopping, or the ODrive left closed loop.
static bool homing_find_end(twai_node_handle_t node, float dir, float *end)
{
    const float start = EncoderEstimate;
    float target = start;
    float last_pos = start;
    TickType_t begun = xTaskGetTickCount(), last_check = begun;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10));
        float pos = EncoderEstimate;
        if (AxisStatus != CLOSED_LOOP_CONTROL) {
            ESP_LOGE(TAG, "Homing: the ODrive left closed loop (errors 0x%" PRIx32 ")", AxisError);
            return false;
        }
        if (fabsf(pos - start) > HOMING_MAX_TURNS) {
            set_position(node, pos);
            ESP_LOGE(TAG, "Homing: no end after %.0f turns: is the carriage on the rail?", HOMING_MAX_TURNS);
            return false;
        }
        target += dir * HOMING_SPEED * 0.01f;
        if (dir * (target - pos) > HOMING_LEAD) {
            target = pos + dir * HOMING_LEAD;   // pushing against the end: don't wind up far ahead
        }
        set_position(node, target);
        TickType_t now = xTaskGetTickCount();
        if (now - last_check >= pdMS_TO_TICKS(HOMING_CHECK_MS)) {
            bool settled = now - begun >= pdMS_TO_TICKS(HOMING_SETTLE_MS);
            if (settled && fabsf(pos - last_pos) < HOMING_STALL_TURNS) {
                set_position(node, pos);   // stop pushing
                *end = pos;
                return true;
            }
            last_pos = pos;
            last_check = now;
        }
    }
}

// Find both ends of the rail and set the stroke's span (EncoderContracted..EncoderExtended) inside
// them: the contracted end first, then the extended end, where it stays (fully retracted on the
// machine). If that fails, a small span around where it is.
static void odrive_home(twai_node_handle_t node)
{
    while (!set_limits(node, HOMING_SPEED * 2.0f, HOMING_CURRENT_A)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float high = 0.0f, low = 0.0f;
    bool ok = homing_find_end(node, -1.0f, &low);
    if (ok) {
        ESP_LOGW(TAG, "Homing: contracted end at %.3f", low);
        vTaskDelay(pdMS_TO_TICKS(200));
        ok = homing_find_end(node, +1.0f, &high);
    }
    if (ok) {
        ESP_LOGW(TAG, "Homing: extended end at %.3f", high);
        if (high - low < HOMING_MIN_TRAVEL) {
            ESP_LOGE(TAG, "Homing: ends only %.3f turns apart: it didn't move (stuck, or too little current)", high - low);
            ok = false;
        }
    }
    if (ok) {
        EncoderContracted = low + HOMING_BACKOFF;
        EncoderExtended = high - HOMING_BACKOFF;
        set_position(node, EncoderExtended);   // off the end stop
        ESP_LOGW(TAG, "Homing: rail %.3f turns, stroke span %.3f..%.3f", high - low, EncoderContracted, EncoderExtended);
    } else {
        float here = EncoderEstimate;
        EncoderContracted = here - HOMING_FALLBACK;
        EncoderExtended = here + HOMING_FALLBACK;
        set_position(node, here);
        ESP_LOGE(TAG, "Homing failed: using %.3f..%.3f around where it is", EncoderContracted, EncoderExtended);
    }
    EncoderExtensionsDone = true;
    vTaskDelay(pdMS_TO_TICKS(300));   // let it settle off the stop
}

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
            fault_note(start_clears < FAULT_AUTO_TRIES);
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

    fault_clear();
    request_state_event(BUTTON_ODRIVE_RECOVERED);   // back to INITIAL, if the start-up had a fault shown
    request_state_event(BUTTON_HOMING_STARTED);

    // Wait for a position, then find the ends of the rail
    if (xSemaphoreTake(EncoderEstimate_semaphore, -1) == pdTRUE) {
        initial_position = EncoderEstimate;
        ESP_LOGI(TAG, "Initial Encoder Position: %f", initial_position);
    }
    odrive_home(node);

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
    bool auto_limiting = false;            // Auto's speed and acceleration limiter (AUTO_VEL_MAX...)
    float lim_pos = 0.0f, lim_vel = 0.0f, lim_prev_desired = 0.0f;
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
    float settle = 0.0f;             // the settling stiffness, 0..1
    float settle_wait_s = 0.0f;      // how long the carriage hasn't been pushed away
    float carriage_vel = 0.0f;       // turns/s, smoothed
    float prev_carriage = NAN;
#endif

    // One pass for every handle reading, as soon as it arrives, and at least every LOOP_PERIOD_MS
    // without one: Auto's strokes come from the controller's own clock, so a slow radio link (a
    // busy Wi-Fi channel delays the handle's replies by tens of ms) doesn't stutter them. A pass
    // without a fresh reading uses the last one.
    bool have_reading = false;
    TickType_t last_pass = xTaskGetTickCount();
    for (;;) {
        TickType_t since = xTaskGetTickCount() - last_pass;
        TickType_t wait = since >= pdMS_TO_TICKS(LOOP_PERIOD_MS) ? 0 : pdMS_TO_TICKS(LOOP_PERIOD_MS) - since;
        bool fresh = xQueueReceive(manipulator_encoder_queue, &enc_msg, wait) == pdTRUE;
        if (fresh) {
            have_reading = true;
        }
        if (!have_reading || g_firmware_update_active) {
            // nothing from the handle yet, or an update is being written: leave the ODrive be. Restart
            // the wait, or the next pass wouldn't block and this task (above the update receiver's
            // priority) would spin and starve it.
            last_pass = xTaskGetTickCount();
            continue;
        }
        last_pass = xTaskGetTickCount();
#if TIMING_DEBUG
        int64_t loop_start_us = esp_timer_get_time();
#endif
        if (odrive_setup_active()) {
            continue;   // odrive_setup.c has the ODrive: no setpoints or limits
        }
        float controller_target = C_ManipulatorEncoder_OdriveEncoder(enc_msg.value);
        if (!fresh && machine_state_get(&s_machine_ctx) == MACHINE_DISCONNECTED) {
            continue;   // link lost: the ODrive holds its last target (resending its own position would let it sag)
        }
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
                fault_clear();
                request_state_event(BUTTON_ODRIVE_RECOVERED);   // SYNCING: hold where it is, the handle comes over
            } else {
                fault_note(fault_tries < FAULT_AUTO_TRIES);
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
            fault_clear();
            fault_note(true);
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
        // Auto: limit the target's speed and acceleration (AUTO_VEL_MAX, AUTO_ACCEL_MAX). It follows the
        // stroke's own speed, plus a correction that closes any gap but can always stop in time.
        if (control.machine_state == MACHINE_AUTO) {
            if (!auto_limiting) {
                auto_limiting = true;
                lim_pos = last_target;
                lim_vel = 0.0f;
                lim_prev_desired = target;
            }
            if (dt_ms > 0) {
                float dt = (float)dt_ms / 1000.0f;
                float stroke_vel = (target - lim_prev_desired) / dt;
                lim_prev_desired = target;
                float err = target - lim_pos;
                float closing = fminf(fabsf(err) / AUTO_TRACK_S, sqrtf(2.0f * AUTO_ACCEL_MAX * fabsf(err)));
                float want = stroke_vel + copysignf(closing, err);
                want = fminf(fmaxf(want, -AUTO_VEL_MAX), AUTO_VEL_MAX);
                float dv_max = AUTO_ACCEL_MAX * dt;
                lim_vel += fminf(fmaxf(want - lim_vel, -dv_max), dv_max);
                lim_pos += lim_vel * dt;
            }
            target = lim_pos;
        } else {
            auto_limiting = false;
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
            if (control.machine_state == MACHINE_MANUAL && !manual_easing) {
                // the stiffness the gap asks for: 0 up to GAP_SOFT_TURNS, an S-curve up to 1 at GAP_STIFF_TURNS
                float gap = fabsf(controller_target - EncoderEstimate);   // handle to carriage, turns
                float x = (gap - GAP_SOFT_TURNS) / (GAP_STIFF_TURNS - GAP_SOFT_TURNS);
                x = fminf(fmaxf(x, 0.0f), 1.0f);
                float want = x * x * (3.0f - 2.0f * x);

                // settling: the carriage's speed, and whether it's being pushed away from the handle
                float carriage = EncoderEstimate;
                if (!isnan(prev_carriage) && dt_s > 0.0f) {
                    float raw = (carriage - prev_carriage) / dt_s;
                    carriage_vel += (dt_s / (CARRIAGE_VEL_FILTER_S + dt_s)) * (raw - carriage_vel);
                }
                prev_carriage = carriage;
                float signed_gap = controller_target - carriage;   // handle minus carriage
                bool pushed_away = fabsf(carriage_vel) > CARRIAGE_MOVING_TPS && carriage_vel * signed_gap < 0.0f;
                if (pushed_away) {
                    settle_wait_s = 0.0f;
                    settle += (0.0f - settle) * (dt_s / (SETTLE_RELEASE_S + dt_s));
                } else if (gap > SETTLE_GAP_TURNS) {
                    settle_wait_s += dt_s;
                    if (settle_wait_s >= SETTLE_DELAY_S) {
                        settle = fminf(settle + dt_s / SETTLE_S, 1.0f);
                    }
                } else if (gap < SETTLE_DONE_TURNS) {
                    settle_wait_s = 0.0f;
                    settle += (0.0f - settle) * (dt_s / (SOFTEN_S + dt_s));
                }
                want = fmaxf(want, settle);

                // rising: quickly; falling: slowly, but at once when pushed away (so a push after settling starts soft)
                float tau = (want > stiffness) ? STIFFEN_S : (pushed_away ? SETTLE_RELEASE_S : SOFTEN_S);
                stiffness += (want - stiffness) * (dt_s / (tau + dt_s));
                if (fabsf(want - stiffness) < 0.002f) {
                    stiffness = want;   // settle exactly, so fully soft or stiff is reached and sent
                }
            } else {
                stiffness = 1.0f;   // every other state (and easing into MANUAL) stiff, as saved
                settle = 0.0f;
                settle_wait_s = 0.0f;
                prev_carriage = NAN;
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
