// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * ESP-NOW link to the manipulator (haptic handle / HMI) board.
 *
 * Frame layouts are in omachine_protocol.h (shared with omachine-manipulator).
 * The controller polls every POLL_PERIOD_MS with the ODrive position and every display
 * element; the manipulator answers each poll with the handle position and running
 * counters for the rotary knob and buttons, which are turned into events here.
 *
 * Pairing is automatic: until a manipulator answers, frames go to the broadcast
 * address. The first valid reply pairs that MAC, and from then on frames are
 * unicast (MAC-level ACK + retries). If the manipulator stays silent for
 * UNPAIR_TIMEOUT_MS we fall back to broadcast so a replacement board can pair.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "firmware_valid.h"
#include "firmware_receiver.h"
#include "omachine_update_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_types.h"
#include "machine_state.h"
#include "odrive_setup.h"
#include "omachine_protocol.h"

static_assert(OM_ELEMENT_COUNT == ELEMENT_COUNT, "omachine_protocol.h and machine_state.h disagree on the element list");

// Radio settings: must match omachine-manipulator (link_task.cpp)
#define ESPNOW_CHANNEL       (1)     // 1..13; move off 1 if a nearby Wi-Fi network uses it
#define ESPNOW_LONG_RANGE    (1)     // 1: Espressif Long Range PHY (better reception, lower bit rate)
#define ESPNOW_LR_RATE       WIFI_PHY_RATE_LORA_500K   // or WIFI_PHY_RATE_LORA_250K: more range, slower
#define ESPNOW_TX_POWER      (80)    // units of 0.25 dBm: 80 = 20 dBm (maximum)

#define POLL_PERIOD_MS       (5)     // poll the manipulator every 5 ms (200 Hz) with the latest ODrive position
#define SEND_DONE_TIMEOUT_MS (50)
#define LINK_TIMEOUT_MS      (500)   // no reply for this long -> BUTTON_CONTROLLER_DISCONNECTED
#define UNPAIR_TIMEOUT_MS    (2000)  // no reply for this long -> forget peer, go back to broadcast
#define LINK_STATS_DEBUG     (0)     // 1: log sent/failed/replies, lost replies and max reply gap once a second

namespace {
static const char *TAG = "ESP-NOW Manipulator Task";
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static machine_state_ctx_t *s_machine_ctx = nullptr;
static TaskHandle_t s_tx_task = nullptr;

// Peer state. Written by the TX task, except s_candidate_* which the RX callback hands over.
static portMUX_TYPE s_peer_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static bool s_paired = false;
static uint8_t s_candidate_mac[ESP_NOW_ETH_ALEN];
static bool s_candidate_pending = false;

static volatile int64_t s_last_rx_us = 0;
// Reply timing, logged once a second by the TX task
static volatile uint32_t s_rx_count = 0;
static volatile uint32_t s_rx_lost = 0;       // gaps in the manipulator's sequence numbers
static volatile uint32_t s_rx_max_gap_us = 0;
static uint32_t s_tx_count = 0;
static uint32_t s_tx_fail = 0;
static bool s_link_up = false;
// Cleared when the link drops (the manipulator may have rebooted and restarted its counters);
// the next reply then only sets the counter baselines instead of generating events.
static volatile bool s_rx_baseline_valid = false;

static void enqueue_button_event(button_type_t type, int8_t delta)
{
    if (!s_machine_ctx || !s_machine_ctx->buttons_queue) {
        return;
    }
    button_event_t ev = {
        .type = type,
        .rotary_delta = delta,
    };
    (void)xQueueSend(s_machine_ctx->buttons_queue, &ev, 0);
}

/**
 * @brief Handle one reply from the manipulator (Wi-Fi task context)
 */
static void handle_frame(const om_manipulator_frame_t *f)
{
    static uint8_t last_seq, last_rotary, last_auto, last_rotary_btn, last_power, last_save, last_setup, last_setup_cancel;

    // Publish latest manipulator encoder to the ODrive control loop
    encoder_estimate_msg_t enc_msg = { .value = f->encoder_pos };
    xQueueOverwrite(manipulator_encoder_queue, &enc_msg);  // Always keep latest

    if (!s_rx_baseline_valid) {
        last_seq = f->seq;
        last_rotary = f->rotary_count;
        last_auto = f->auto_presses;
        last_rotary_btn = f->rotary_presses;
        last_power = f->power_count;
        last_save = f->save_count;
        last_setup = f->setup_count;
        last_setup_cancel = f->setup_cancel_count;
        s_rx_baseline_valid = true;
        return;
    }

    s_rx_lost = s_rx_lost + (uint8_t)(f->seq - last_seq - 1);
    last_seq = f->seq;

    // Counters wrap at 256; the difference since the last reply is what happened in between
    int8_t clicks = (int8_t)(f->rotary_count - last_rotary);
    last_rotary = f->rotary_count;
    if (clicks != 0) {
        enqueue_button_event(BUTTON_ROTARY, clicks);
    }

    int8_t power_clicks = (int8_t)(f->power_count - last_power);
    last_power = f->power_count;
    if (power_clicks != 0) {
        enqueue_button_event(BUTTON_POWER, power_clicks);
    }

    if (f->save_count != last_save) {
        enqueue_button_event(BUTTON_SAVE_SETTINGS, 0);   // several requests at once need only one save
    }
    last_save = f->save_count;

    // "Set up ODrive": setup_ack tells the menu its request has arrived, whether or not a setup
    // could start (one already running counts as that one)
    if (f->setup_count != last_setup) {
        enqueue_button_event(BUTTON_ODRIVE_SETUP, 0);
        odrive_setup_ack(f->setup_count);
    }
    last_setup = f->setup_count;
    if (f->setup_cancel_count != last_setup_cancel) {
        enqueue_button_event(BUTTON_ODRIVE_SETUP_CANCEL, 0);
    }
    last_setup_cancel = f->setup_cancel_count;

    for (uint8_t n = (uint8_t)(f->auto_presses - last_auto); n > 0; n--) {
        enqueue_button_event(BUTTON_AUTO_BUTTON, 0);
        ESP_LOGI(TAG, "Button Pressed!");
    }
    last_auto = f->auto_presses;

    for (uint8_t n = (uint8_t)(f->rotary_presses - last_rotary_btn); n > 0; n--) {
        enqueue_button_event(BUTTON_ROTARY_BUTTON, 0);
    }
    last_rotary_btn = f->rotary_presses;
}

/**
 * @brief ESP-NOW receive callback (runs in the Wi-Fi task, keep it short)
 */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len > 0 && data[0] == OM_UPDATE_MAGIC) {
        // A firmware update. Only from the paired manipulator; unpaired (e.g. its protocol version
        // differs, which an update is there to fix), from any
        bool allowed;
        portENTER_CRITICAL(&s_peer_lock);
        allowed = !s_paired || memcmp(info->src_addr, s_peer_mac, ESP_NOW_ETH_ALEN) == 0;
        portEXIT_CRITICAL(&s_peer_lock);
        if (allowed) {
            firmware_receiver_on_recv(info->src_addr, data, len);
        }
        return;
    }
    if (len != (int)sizeof(om_manipulator_frame_t)) {
        return;  // not ours
    }
    if (data[0] != OM_PROTOCOL_VERSION) {
        static int64_t last_warn_us = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_warn_us > 5000000) {
            ESP_LOGW(TAG, "Ignoring manipulator frame with protocol version %d (expected %d): reflash both boards",
                           data[0], OM_PROTOCOL_VERSION);
            last_warn_us = now_us;
        }
        return;
    }

    bool accept;
    portENTER_CRITICAL(&s_peer_lock);
    if (s_paired) {
        accept = memcmp(info->src_addr, s_peer_mac, ESP_NOW_ETH_ALEN) == 0;
    } else {
        // First manipulator to answer gets paired by the TX task
        if (!s_candidate_pending) {
            memcpy(s_candidate_mac, info->src_addr, ESP_NOW_ETH_ALEN);
            s_candidate_pending = true;
        }
        accept = memcmp(info->src_addr, s_candidate_mac, ESP_NOW_ETH_ALEN) == 0;
    }
    portEXIT_CRITICAL(&s_peer_lock);

    if (!accept) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    uint32_t gap_us = (uint32_t)(now_us - s_last_rx_us);
    if (s_last_rx_us != 0 && gap_us > s_rx_max_gap_us) {
        s_rx_max_gap_us = gap_us;
    }
    s_rx_count = s_rx_count + 1;
    s_last_rx_us = now_us;
    om_manipulator_frame_t frame;
    memcpy(&frame, data, sizeof(frame));
    handle_frame(&frame);
}

/**
 * @brief ESP-NOW send callback: report delivery status back to the TX task
 */
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (s_tx_task) {
        xTaskNotify(s_tx_task, (status == ESP_NOW_SEND_SUCCESS) ? 1 : 2, eSetValueWithOverwrite);
    }
}

static void add_peer(const uint8_t *mac)
{
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
        return;
    }
#if ESPNOW_LONG_RANGE
    esp_now_rate_config_t rate = {};
    rate.phymode = WIFI_PHY_MODE_LR;
    rate.rate = ESPNOW_LR_RATE;
    err = esp_now_set_peer_rate_config(mac, &rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_set_peer_rate_config failed: %s", esp_err_to_name(err));
    }
#endif
}

static void wifi_espnow_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {  // INVALID_STATE: already created
        ESP_ERROR_CHECK(err);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // no modem sleep, lowest latency
#if ESPNOW_LONG_RANGE
    // LR alongside the normal protocols (as in Espressif's ESP-NOW example); peers are sent LR in add_peer()
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESPNOW_TX_POWER));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    add_peer(BROADCAST_MAC);

    uint8_t mac[ESP_NOW_ETH_ALEN];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "ESP-NOW ready on channel %d%s, controller MAC " MACSTR, ESPNOW_CHANNEL,
             ESPNOW_LONG_RANGE ? " (long range)" : "", MAC2STR(mac));
}

/**
 * @brief Pair a newly heard manipulator, and watch for link loss
 */
static void supervise_link()
{
    uint8_t candidate[ESP_NOW_ETH_ALEN];
    bool have_candidate = false;
    portENTER_CRITICAL(&s_peer_lock);
    if (!s_paired && s_candidate_pending) {
        memcpy(candidate, s_candidate_mac, ESP_NOW_ETH_ALEN);
        have_candidate = true;
    }
    portEXIT_CRITICAL(&s_peer_lock);

    if (have_candidate) {
        add_peer(candidate);
        portENTER_CRITICAL(&s_peer_lock);
        memcpy(s_peer_mac, candidate, ESP_NOW_ETH_ALEN);
        s_paired = true;
        s_candidate_pending = false;
        portEXIT_CRITICAL(&s_peer_lock);
        ESP_LOGI(TAG, "Paired with manipulator " MACSTR, MAC2STR(candidate));
    }

    int64_t silent_ms = (esp_timer_get_time() - s_last_rx_us) / 1000;

    if (!s_link_up && s_paired && silent_ms < LINK_TIMEOUT_MS) {
        firmware_mark_valid("link up");
        s_link_up = true;
        ESP_LOGI(TAG, "Link up");
        enqueue_button_event(BUTTON_CONTROLLER_CONNECTED, 0);
    } else if (s_link_up && silent_ms >= LINK_TIMEOUT_MS) {
        s_link_up = false;
        s_rx_baseline_valid = false;
        ESP_LOGW(TAG, "Link lost (%" PRId64 " ms without reply)", silent_ms);
        enqueue_button_event(BUTTON_CONTROLLER_DISCONNECTED, 0);
    }

    if (s_paired && silent_ms >= UNPAIR_TIMEOUT_MS) {
        uint8_t old[ESP_NOW_ETH_ALEN];
        portENTER_CRITICAL(&s_peer_lock);
        memcpy(old, s_peer_mac, ESP_NOW_ETH_ALEN);
        s_paired = false;
        s_candidate_pending = false;
        portEXIT_CRITICAL(&s_peer_lock);
        esp_now_del_peer(old);
        ESP_LOGW(TAG, "Unpaired " MACSTR ", broadcasting for a manipulator", MAC2STR(old));
    }
}
}

/**
 * @brief Manipulator link task
 *
 * Sends one frame every POLL_PERIOD_MS with the latest ODrive position and every display
 * element. The manipulator answers every frame it receives.
 */
extern "C" void espnow_manipulator_task(void *pvParameters)
{
    s_machine_ctx = reinterpret_cast<machine_state_ctx_t *>(pvParameters);
    s_tx_task = xTaskGetCurrentTaskHandle();

    wifi_espnow_init();
    firmware_receiver_init();

    uint16_t odrive_position = 0;
    encoder_estimate_msg_t enc_msg;
    om_controller_frame_t frame = {};
    frame.version = OM_PROTOCOL_VERSION;

    TickType_t next_poll = xTaskGetTickCount();
    while (true) {
        // Fixed-rate polling; odrive_encoder_queue holds just the latest ODrive position
        xTaskDelayUntil(&next_poll, pdMS_TO_TICKS(POLL_PERIOD_MS));
        if (xQueueReceive(odrive_encoder_queue, &enc_msg, 0) == pdTRUE) {
            odrive_position = enc_msg.value;
        }

        supervise_link();
        if (g_firmware_update_active) {
            continue;   // receiving an update: the manipulator is busy sending it, don't poll
        }

        frame.seq++;
        frame.odrive_pos = odrive_position;
        for (int i = 0; i < OM_ELEMENT_COUNT; i++) {
            int32_t v = machine_element_value((machine_element_key_t)i);
            frame.elements[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
        }
        frame.saved_count = machine_settings_saved_count;
        odrive_setup_status(&frame.setup_ack, &frame.setup_stage, &frame.setup_progress);
        frame.odrive_error = g_odrive_fault_error;

        uint8_t dest[ESP_NOW_ETH_ALEN];
        portENTER_CRITICAL(&s_peer_lock);
        memcpy(dest, s_paired ? s_peer_mac : BROADCAST_MAC, ESP_NOW_ETH_ALEN);
        portEXIT_CRITICAL(&s_peer_lock);

        xTaskNotifyStateClear(NULL);
        esp_err_t err = esp_now_send(dest, (const uint8_t *)&frame, sizeof(frame));
        uint32_t result = 0;
        if (err == ESP_OK) {
            xTaskNotifyWait(0, UINT32_MAX, &result, pdMS_TO_TICKS(SEND_DONE_TIMEOUT_MS));
        } else {
            // One line a second at most: a line per failed poll would be up to 200 a second (it
            // happens when the link is poor, e.g. the antenna blocked)
            static int64_t s_last_fail_log_us = 0;
            static uint32_t s_send_errors = 0;
            s_send_errors++;
            int64_t now_us = esp_timer_get_time();
            if (now_us - s_last_fail_log_us >= 1000000) {
                ESP_LOGW(TAG, "esp_now_send failed %" PRIu32 " times in the last second (last: %s)",
                         s_send_errors, esp_err_to_name(err));
                s_send_errors = 0;
                s_last_fail_log_us = now_us;
            }
        }

        s_tx_count++;
        if (result != 1) {
            s_tx_fail++;
        }
#if LINK_STATS_DEBUG
        {
            static int64_t window_start_us = 0;
            int64_t now_us = esp_timer_get_time();
            if (now_us - window_start_us >= 1000000) {
                ESP_LOGI(TAG, "[timing] sent %" PRIu32 " (failed %" PRIu32 "), replies %" PRIu32 " (lost %" PRIu32 "), max reply gap %" PRIu32 " ms",
                         s_tx_count, s_tx_fail, s_rx_count, s_rx_lost, s_rx_max_gap_us / 1000);
                window_start_us = now_us;
                s_tx_count = 0; s_tx_fail = 0; s_rx_count = 0; s_rx_lost = 0; s_rx_max_gap_us = 0;
            }
        }
#endif
    }
}
