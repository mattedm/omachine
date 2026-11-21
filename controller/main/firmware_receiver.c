// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "firmware_receiver.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "omachine_update_protocol.h"

static const char *TAG = "fw-update";

#define IDLE_TIMEOUT_US (60 * 1000000LL)   // no frames for this long mid-update: give up and restart

typedef struct {
    uint8_t mac[6];
    uint16_t len;
    uint8_t data[sizeof(om_update_header_t) + OM_UPDATE_CHUNK];
} rx_item_t;

volatile bool g_firmware_update_active = false;

static QueueHandle_t s_queue;
static esp_ota_handle_t s_ota;
static const esp_partition_t *s_target;
static uint32_t s_expected;   // next image byte expected
static uint32_t s_length;     // image size from BEGIN
static uint8_t s_mac[6];      // the manipulator doing the update
static int64_t s_last_frame_us;

void firmware_receiver_on_recv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (!s_queue || len < (int)sizeof(om_update_header_t) || len > (int)sizeof(((rx_item_t *)0)->data)) {
        return;
    }
    rx_item_t item;
    memcpy(item.mac, mac, 6);
    item.len = (uint16_t)len;
    memcpy(item.data, data, len);
    (void)xQueueSend(s_queue, &item, 0);   // a dropped frame is resent by the manipulator
}

static void send_frame(const uint8_t *mac, const void *frame, size_t len)
{
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;   // the current channel
    peer.ifidx = WIFI_IF_STA;
    if (!esp_now_is_peer_exist(mac)) {
        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_add_peer: %s", esp_err_to_name(err));
            return;
        }
    }
    esp_now_send(mac, (const uint8_t *)frame, len);
}

static void send_ack(const uint8_t *mac, uint8_t status)
{
    om_update_header_t ack = {
        .magic = OM_UPDATE_MAGIC,
        .type = OM_UPD_ACK,
        .status = status,
        .offset = s_expected,
        .length = s_length,
    };
    send_frame(mac, &ack, sizeof(ack));
}

static void restart_soon(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));   // let the last ACK go out
    esp_restart();
}

static void handle(const rx_item_t *item)
{
    om_update_header_t h;
    memcpy(&h, item->data, sizeof(h));
    const uint8_t *payload = item->data + sizeof(h);
    size_t payload_len = item->len - sizeof(h);

    if (g_firmware_update_active && h.type != OM_UPD_HELLO && memcmp(item->mac, s_mac, 6) != 0) {
        return;   // another manipulator: only one update at a time
    }

    switch (h.type) {
    case OM_UPD_HELLO: {
        uint8_t frame[sizeof(om_update_header_t) + sizeof(om_update_hello_t)] = {0};
        om_update_header_t ack = {.magic = OM_UPDATE_MAGIC, .type = OM_UPD_HELLO_ACK};
        om_update_hello_t hello = {0};
        const esp_app_desc_t *app = esp_app_get_description();
        strncpy(hello.project, app->project_name, sizeof(hello.project) - 1);
        strncpy(hello.version, app->version, sizeof(hello.version) - 1);
        memcpy(frame, &ack, sizeof(ack));
        memcpy(frame + sizeof(ack), &hello, sizeof(hello));
        send_frame(item->mac, frame, sizeof(frame));
        break;
    }
    case OM_UPD_BEGIN:
        if (g_firmware_update_active) {
            send_ack(item->mac, s_expected == 0 ? OM_UPD_OK : OM_UPD_ERR_STATE);   // a repeated BEGIN
            break;
        }
        s_target = esp_ota_get_next_update_partition(NULL);
        if (!s_target || h.length == 0 || h.length > s_target->size) {
            send_ack(item->mac, OM_UPD_ERR_IMAGE);
            break;
        }
        ESP_LOGW(TAG, "Firmware update from " MACSTR ": %lu bytes into %s", MAC2STR(item->mac),
                 (unsigned long)h.length, s_target->label);
        g_firmware_update_active = true;   // the link stops polling
        odrive_idle_for_update();
        memcpy(s_mac, item->mac, 6);
        s_expected = 0;
        s_length = h.length;
        // Sequential writes erase sector by sector as the data arrives, so BEGIN answers at once
        if (esp_ota_begin(s_target, OTA_WITH_SEQUENTIAL_WRITES, &s_ota) != ESP_OK) {
            send_ack(item->mac, OM_UPD_ERR_FLASH);
            restart_soon();
        }
        send_ack(item->mac, OM_UPD_OK);
        break;
    case OM_UPD_DATA:
        if (!g_firmware_update_active) {
            send_ack(item->mac, OM_UPD_ERR_STATE);
            break;
        }
        if (h.offset == s_expected && h.length == payload_len && s_expected + payload_len <= s_length) {
            if (esp_ota_write(s_ota, payload, payload_len) != ESP_OK) {
                ESP_LOGE(TAG, "Writing at %lu failed", (unsigned long)s_expected);
                send_ack(item->mac, OM_UPD_ERR_FLASH);
                esp_ota_abort(s_ota);
                restart_soon();
            }
            s_expected += payload_len;
        }
        send_ack(item->mac, OM_UPD_OK);   // a repeat or out-of-order frame: the ACK says where to go on from
        break;
    case OM_UPD_END: {
        if (!g_firmware_update_active) {
            send_ack(item->mac, OM_UPD_ERR_STATE);
            break;
        }
        uint8_t status = OM_UPD_OK;
        esp_app_desc_t desc;
        if (s_expected != s_length || h.length != s_length || esp_ota_end(s_ota) != ESP_OK) {
            status = OM_UPD_ERR_IMAGE;
        } else if (esp_ota_get_partition_description(s_target, &desc) != ESP_OK ||
                   strcmp(desc.project_name, esp_app_get_description()->project_name) != 0) {
            status = OM_UPD_ERR_IMAGE;   // a valid image, but not for this board
        } else if (esp_ota_set_boot_partition(s_target) != ESP_OK) {
            status = OM_UPD_ERR_FLASH;
        }
        if (status == OM_UPD_OK) {
            ESP_LOGW(TAG, "Update to %s received: restarting into it", desc.version);
        } else {
            ESP_LOGE(TAG, "Update rejected (%d): restarting on the old firmware", status);
        }
        send_ack(item->mac, status);
        restart_soon();
        break;
    }
    case OM_UPD_ABORT:
        if (g_firmware_update_active) {
            ESP_LOGW(TAG, "Update aborted: restarting on the old firmware");
            esp_ota_abort(s_ota);
            send_ack(item->mac, OM_UPD_OK);
            restart_soon();
        }
        break;
    default:
        break;
    }
}

static void receiver_task(void *arg)
{
    static rx_item_t item;   // too big for the stack
    for (;;) {
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(1000)) == pdTRUE) {
            s_last_frame_us = esp_timer_get_time();
            handle(&item);
        } else if (g_firmware_update_active && esp_timer_get_time() - s_last_frame_us > IDLE_TIMEOUT_US) {
            ESP_LOGE(TAG, "Update stalled: restarting on the old firmware");
            esp_ota_abort(s_ota);
            esp_restart();
        }
    }
}

void firmware_receiver_init(void)
{
    s_queue = xQueueCreate(4, sizeof(rx_item_t));
    assert(s_queue);
    xTaskCreate(receiver_task, "FW_Receiver", 4096, NULL, 9, NULL);
}
