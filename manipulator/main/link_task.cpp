// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "link_task.h"
#include "firmware_valid.h"
#include "firmware_update.h"
#include "omachine_update_protocol.h"
#include "inputs.h"
#include "motor_task.h"
#include "motor_calibration.h"
#include "power.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "link";

// Radio settings: must match the controller (espnow_manipulator.cpp)
#define ESPNOW_CHANNEL 1                          // 1..13
#define ESPNOW_LONG_RANGE 1                       // 1: Espressif Long Range PHY
#define ESPNOW_LR_RATE WIFI_PHY_RATE_LORA_500K    // or WIFI_PHY_RATE_LORA_250K
#define ESPNOW_TX_POWER 80                        // units of 0.25 dBm: 80 = 20 dBm (maximum)
// Send replies in Long Range too. LR replies only get ~130 frames/s out on this build, below the
// 200 Hz poll rate, and default-rate replies keep up, so they are the better fit
#define ESPNOW_REPLY_LONG_RANGE 0
// Replies allowed in ESP-NOW's queue at once: 2 keeps the radio busy (each send takes several ms
// to complete) while no reply waits behind more than one other
#define MAX_REPLIES_IN_FLIGHT 2
// Give up waiting for send callbacks that never came
#define REPLY_IN_FLIGHT_TIMEOUT_US 50000
#define LINK_TIMEOUT_MS 500
#define RSSI_WINDOW_MS 1000                       // linkRssi is the weakest poll in each window
#define LINK_IDLE_OFF_MS 120000                   // no polls for this long (or none since starting): turn off

uint8_t elements[OM_ELEMENT_COUNT] = {0,0,0,0,0,0,0};

typedef struct {
  uint8_t mac[ESP_NOW_ETH_ALEN];
  om_controller_frame_t frame;
  int8_t rssi;   // dBm
  int64_t rx_us;  // when the Wi-Fi task received it
} rx_frame_t;

static QueueHandle_t rx_frame_queue;
static uint8_t controllerMac[ESP_NOW_ETH_ALEN];
static bool controllerPaired = false;
volatile bool linkUp = false;
volatile int8_t linkRssi = 0;
volatile uint8_t powerCount = 0;
volatile uint8_t saveCount = 0;
volatile uint8_t controllerSavedCount = 0;
volatile uint8_t setupCount = 0;
volatile uint8_t setupCancelCount = 0;
volatile uint8_t controllerSetupAck = 0;
volatile uint8_t controllerSetupStage = OM_SETUP_IDLE;
volatile uint8_t controllerSetupProgress = 0;
static portMUX_TYPE s_fault_lock = portMUX_INITIALIZER_UNLOCKED;
static OdriveFault s_fault = {};

OdriveFault controllerOdriveFault()
{
  portENTER_CRITICAL(&s_fault_lock);
  OdriveFault f = s_fault;
  portEXIT_CRITICAL(&s_fault_lock);
  return f;
}

// ESP-NOW can't drop frames once queued, so a reply queued behind many others would carry a stale
// handle position. Instead a poll that arrives while MAX_REPLIES_IN_FLIGHT replies are still being
// sent gets no reply, and the next poll's reply carries the latest position.
static volatile int s_replies_in_flight = 0;
static volatile int64_t s_send_us = 0;
#if LOOP_RATE_DEBUG
static volatile uint32_t s_tx_ok = 0, s_tx_fail = 0, s_tx_max_ack_us = 0;
#endif

// ESP-NOW send callback (runs in the Wi-Fi task): the reply is delivered or given up on
static void onEspNowSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
#if LOOP_RATE_DEBUG
  uint32_t ack_us = (uint32_t)(esp_timer_get_time() - s_send_us);
  if (status == ESP_NOW_SEND_SUCCESS) s_tx_ok = s_tx_ok + 1; else s_tx_fail = s_tx_fail + 1;
  if (ack_us > s_tx_max_ack_us) s_tx_max_ack_us = ack_us;
#endif
  if (s_replies_in_flight > 0) s_replies_in_flight = s_replies_in_flight - 1;
}

// ESP-NOW receive callback (runs in the Wi-Fi task): hand the frame to linkTask
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (len > 0 && data[0] == OM_UPDATE_MAGIC) {
    firmwareUpdateOnRecv(info->src_addr, data, len);   // the controller answering a firmware update
    return;
  }
  if (len != (int)sizeof(om_controller_frame_t) || data[0] != OM_PROTOCOL_VERSION) {
    return;  // not ours, or a controller running a different protocol version
  }
  rx_frame_t frame;
  memcpy(frame.mac, info->src_addr, ESP_NOW_ETH_ALEN);
  memcpy(&frame.frame, data, sizeof(frame.frame));
  frame.rssi = info->rx_ctrl->rssi;
  frame.rx_us = esp_timer_get_time();
  xQueueSend(rx_frame_queue, &frame, 0);
}

void linkRadioRestore()
{
  uint8_t before = 0;
  esp_wifi_get_protocol(WIFI_IF_STA, &before);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // no modem sleep, lowest latency
#if ESPNOW_LONG_RANGE
  // LR alongside the normal protocols; replies to the controller use ESPNOW_REPLY_LONG_RANGE
  ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESPNOW_TX_POWER));
  ESP_LOGI(TAG, "Radio set for the link (STA protocols were 0x%02x)", before);
}

static void setupEspNow()
{
  esp_err_t err = nvs_flash_init();   // Wi-Fi keeps calibration data in NVS
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  // Network interfaces for firmware updates (joining the home Wi-Fi) and the Wi-Fi setup page;
  // ESP-NOW itself doesn't use them
  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  linkRadioRestore();

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(onEspNowRecv));
  ESP_ERROR_CHECK(esp_now_register_send_cb(onEspNowSent));
  firmwareUpdateInit();

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  ESP_LOGI(TAG, "ESP-NOW ready on channel %d%s, manipulator MAC " MACSTR, ESPNOW_CHANNEL,
           ESPNOW_LONG_RANGE ? " (long range)" : "", MAC2STR(mac));
}

// Reply to whichever controller polled us; switch peers if a different one shows up
static void setControllerPeer(const uint8_t *mac)
{
  if (controllerPaired && memcmp(mac, controllerMac, ESP_NOW_ETH_ALEN) == 0) {
    return;
  }
  if (controllerPaired) {
    esp_now_del_peer(controllerMac);
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) {
#if ESPNOW_LONG_RANGE && ESPNOW_REPLY_LONG_RANGE
    esp_now_rate_config_t rate = {};
    rate.phymode = WIFI_PHY_MODE_LR;
    rate.rate = ESPNOW_LR_RATE;
    if (esp_now_set_peer_rate_config(mac, &rate) != ESP_OK) {
      ESP_LOGE(TAG, "esp_now_set_peer_rate_config failed");
    }
#endif
    memcpy(controllerMac, mac, ESP_NOW_ETH_ALEN);
    controllerPaired = true;
    ESP_LOGI(TAG, "Paired with controller " MACSTR, MAC2STR(mac));
  } else {
    controllerPaired = false;
  }
}

// SYNCING ends once the handle is within OM_SYNC_TOLERANCE of the held ODrive, not exactly on it.
// So that neither side jumps, the link maps the handle onto the ODrive with a bent line through
// (0,0), (syncHandle,syncOdrive) and (1,1), positions normalised 0..1 across the stroke: matched
// at the sync point, easing to 1:1 at both ends. The handle reports its position through the map,
// and the ODrive position it is sent goes through the inverse, so the handle feels no offset.
// The kink follows the handle: each time the handle has moved SYNC_MAP_STEP, the kink moves to the
// handle's current mapped point. Continuous, but the offset only ever shrinks, by the share of the
// remaining distance to the end the handle moved toward, and is gone once it reaches either end.
static bool syncMapActive = false;
static float syncHandle = 0.0f;
static float syncOdrive = 0.0f;
static uint8_t prevState = 0;
#define SYNC_MAP_STEP 0.005f        // re-anchor after this much handle travel, so jitter doesn't erode the offset
#define SYNC_MAP_DONE 0.0005f       // offset small enough to drop the map

static float handleNorm()
{
  return (motor.shaft_angle - startlimit) / (endlimit - startlimit);
}

// handle -> ODrive
static float mapHandleToOdrive(float h)
{
  if (!syncMapActive) return h;
  if (h <= syncHandle) return h * syncOdrive / syncHandle;
  return syncOdrive + (h - syncHandle) * (1.0f - syncOdrive) / (1.0f - syncHandle);
}

// ODrive -> handle (inverse of mapHandleToOdrive)
static float mapOdriveToHandle(float o)
{
  if (!syncMapActive) return o;
  if (o <= syncOdrive) return o * syncHandle / syncOdrive;
  return syncHandle + (o - syncOdrive) * (1.0f - syncHandle) / (1.0f - syncOdrive);
}

static void processFrame(const om_controller_frame_t &f)
{
  float odrive = float(f.odrive_pos) / (float)OM_POS_MAX;
  uint8_t state = f.elements[OM_ELEMENT_STATE_INDEX];

  // The map stays on in every state, so that leaving MANUAL (e.g. into PREAUTO) doesn't jump the
  // reported position by whatever offset is left; it's re-anchored at each sync and dropped only
  // before homing is done or once the offset has worn away
  if (state == OM_STATE_INITIAL || state == OM_STATE_HOMING) {
    syncMapActive = false;
  } else if (state == OM_STATE_MANUAL && prevState == OM_STATE_SYNCING) {
    // just synced: bend the map through the current handle and ODrive positions
    float h = handleNorm();
    syncMapActive = (h > 0.0f && h < 1.0f && odrive > 0.0f && odrive < 1.0f);
    syncHandle = h;
    syncOdrive = odrive;
  }
  prevState = state;

  if (syncMapActive) {
    float h = handleNorm();
    if (fabsf(h - syncHandle) >= SYNC_MAP_STEP) {
      float o = mapHandleToOdrive(h);
      if (h <= 0.0f || h >= 1.0f || o <= 0.0f || o >= 1.0f || fabsf(o - h) < SYNC_MAP_DONE) {
        syncMapActive = false;   // at an end, or close enough: 1:1 from here on
      } else {
        syncHandle = h;
        syncOdrive = o;
      }
    }
  }

  odriveposition = (mapOdriveToHandle(odrive) * (endlimit - startlimit)) + startlimit;

  // every frame carries the full set of elements; the display redraws if they changed
  memcpy(elements, f.elements, sizeof(elements));
  controllerSavedCount = f.saved_count;
  controllerSetupAck = f.setup_ack;
  controllerSetupStage = f.setup_stage;
  controllerSetupProgress = f.setup_progress;
  OdriveFault fault = {f.odrive_errors, f.odrive_disarm, f.odrive_vbus_dv, f.fault_flags};
  portENTER_CRITICAL(&s_fault_lock);
  s_fault = fault;
  portEXIT_CRITICAL(&s_fault_lock);
}

void linkTask(void *parameters)
{
  uint16_t encoder_angle = 0;
  float flt_encoder_angle;
  rx_frame_t frame;
  om_manipulator_frame_t reply = {};
  reply.version = OM_PROTOCOL_VERSION;
  uint32_t lastRxTime = 0;
  uint32_t lastHeardMs = millis();   // last poll, or when this task started; for LINK_IDLE_OFF_MS
  bool idleOffRequested = false;
  int8_t rssiMin = INT8_MAX;
  uint32_t rssiWindowStart = 0;
  bool rssiShown = false;   // linkRssi holds a reading from the current connection

  rx_frame_queue = xQueueCreate(8, sizeof(rx_frame_t));
  assert(rx_frame_queue);
  setupEspNow();

#if LOOP_RATE_DEBUG
  uint32_t lastRateTime = millis();
  uint32_t lastLoopCount = motorLoopCount;
  uint32_t polls = 0, maxPollGapUs = 0, maxReplyDelayUs = 0, sendErrors = 0, skipped = 0;
  esp_err_t lastSendErr = ESP_OK;
  int64_t lastPollUs = 0;
#endif

  for (;;) {

#if LOOP_RATE_DEBUG
    if (millis() - lastRateTime >= 1000) {
      uint32_t now = millis();
      uint32_t count = motorLoopCount;
      ESP_LOGI(TAG, "Motor loop: %lu Hz, polls %lu, max poll gap %lu us, max reply delay %lu us, tx ok %lu fail %lu err %lu skipped %lu, max ack %lu us",
               (unsigned long)((count - lastLoopCount) * 1000UL / (now - lastRateTime)),
               (unsigned long)polls, (unsigned long)maxPollGapUs, (unsigned long)maxReplyDelayUs,
               (unsigned long)s_tx_ok, (unsigned long)s_tx_fail, (unsigned long)sendErrors, (unsigned long)skipped,
               (unsigned long)s_tx_max_ack_us);
      if (lastSendErr != ESP_OK) ESP_LOGW(TAG, "esp_now_send: %s", esp_err_to_name(lastSendErr));
      s_tx_ok = 0; s_tx_fail = 0; s_tx_max_ack_us = 0; sendErrors = 0; skipped = 0; lastSendErr = ESP_OK;
      lastLoopCount = count;
      lastRateTime = now;
      polls = 0; maxPollGapUs = 0; maxReplyDelayUs = 0;
    }
#endif

    if (xQueueReceive(rx_frame_queue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
      setControllerPeer(frame.mac);
      processFrame(frame.frame);
      lastRxTime = millis();
      lastHeardMs = lastRxTime;
      if (!linkUp) firmware_mark_valid("link up");
      linkUp = true;

      if (frame.rssi < rssiMin) {
        rssiMin = frame.rssi;
      }
      if (!rssiShown) {
        // first poll after (re)connecting: show it straight away, then once per window
        linkRssi = frame.rssi;
        rssiShown = true;
        rssiMin = INT8_MAX;
        rssiWindowStart = lastRxTime;
      } else if (lastRxTime - rssiWindowStart >= RSSI_WINDOW_MS) {
        linkRssi = rssiMin;
        rssiMin = INT8_MAX;
        rssiWindowStart = lastRxTime;
      }

      // While the motor re-aligns from the menu the handle moves to the middle and back on its own:
      // keep reporting where it was, so the machine doesn't follow
      if (motorCalState != MOTOR_CAL_RUNNING && motorCalState != MOTOR_CAL_REQUESTED) {
        flt_encoder_angle = mapHandleToOdrive(handleNorm()) * 32767.0f;
        if (flt_encoder_angle < 0 ) flt_encoder_angle = 0;
        if (flt_encoder_angle > 32767.0 ) flt_encoder_angle = 32767;
        encoder_angle = flt_encoder_angle;
      }

      reply.encoder_pos = encoder_angle;
      reply.rotary_count = rotaryCount;
      reply.power_count = powerCount;
      reply.save_count = saveCount;
      reply.setup_count = setupCount;
      reply.setup_cancel_count = setupCancelCount;
      reply.auto_presses = autoButton.presses;
      reply.rotary_presses = rotaryButton.presses;
      reply.flags = (autoButton.pressed ? OM_FLAG_AUTO_HELD : 0) | (rotaryButton.pressed ? OM_FLAG_ROTARY_HELD : 0);

      // A lost or skipped reply needs no retry: the next one carries the same running counters
      int64_t send_us = esp_timer_get_time();
      if (s_replies_in_flight >= MAX_REPLIES_IN_FLIGHT && send_us - s_send_us > REPLY_IN_FLIGHT_TIMEOUT_US) {
        s_replies_in_flight = 0;
      }
      // (not while a firmware update has the radio on another channel: the send would only fail)
      if (controllerPaired && s_replies_in_flight < MAX_REPLIES_IN_FLIGHT && !updateInProgress) {
        reply.seq++;   // counts sent replies, so the controller's lost-reply count stays meaningful
        s_send_us = send_us;
        s_replies_in_flight = s_replies_in_flight + 1;
        esp_err_t send_err = esp_now_send(controllerMac, (const uint8_t *)&reply, sizeof(reply));
        if (send_err != ESP_OK) {
          s_replies_in_flight = s_replies_in_flight - 1;
#if LOOP_RATE_DEBUG
          sendErrors++;
          lastSendErr = send_err;
#endif
        }
      }
#if LOOP_RATE_DEBUG
      else if (controllerPaired) {
        skipped++;
      }
#endif
#if LOOP_RATE_DEBUG
      int64_t now_us = esp_timer_get_time();
      polls++;
      if (lastPollUs && (uint32_t)(frame.rx_us - lastPollUs) > maxPollGapUs) maxPollGapUs = (uint32_t)(frame.rx_us - lastPollUs);
      lastPollUs = frame.rx_us;
      if ((uint32_t)(now_us - frame.rx_us) > maxReplyDelayUs) maxReplyDelayUs = (uint32_t)(now_us - frame.rx_us);
#endif
    }
    else if (linkUp && (millis() - lastRxTime > LINK_TIMEOUT_MS)) {
      // controller went quiet: keep holding the last position, show it on the display
      linkUp = false;
      rssiShown = false;
    }

    // Nothing from the controller for LINK_IDLE_OFF_MS: turn off rather than run the battery down.
    // Not during a firmware update, which takes the radio off the link's channel on purpose, nor
    // on USB, where there's no battery to save (unplugging starts the full wait afresh).
    if (updateInProgress || power_on_usb()) {
      lastHeardMs = millis();
    } else if (!idleOffRequested && millis() - lastHeardMs >= LINK_IDLE_OFF_MS) {
      ESP_LOGW(TAG, "No link for %d s: turning off", LINK_IDLE_OFF_MS / 1000);
      power_request_off(SCREEN_NO_LINK_OFF);
      idleOffRequested = true;
    }

    // elements keep their last values when the link drops, so don't show a stale mode
    ButtonLightColor light = BUTTON_LIGHT_OFF;
    if (linkUp) {
      switch (elements[OM_ELEMENT_STATE_INDEX]) {
        case OM_STATE_INITIAL:
        case OM_STATE_HOMING:
        case OM_STATE_ODRIVE_SETUP:
          light = BUTTON_LIGHT_BLUE;
          break;
        case OM_STATE_SYNCING:
        case OM_STATE_PREAUTO:
          light = BUTTON_LIGHT_YELLOW;
          break;
        case OM_STATE_MANUAL:
          light = BUTTON_LIGHT_GREEN;
          break;
        case OM_STATE_AUTO:
          light = BUTTON_LIGHT_RED;
          break;
        case OM_STATE_ODRIVE_FAULT:
          light = (millis() / 500) % 2 ? BUTTON_LIGHT_OFF : BUTTON_LIGHT_RED;   // blinking: press to retry
          break;
        default:  // OM_STATE_DISCONNECTED
          break;
      }
    }
    setButtonLight(light);
  }
}
