// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "firmware_valid.h"

#include <stdbool.h>
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_log.h"

#define FIRMWARE_VALID_AFTER_MS 60000

static const char *TAG = "firmware";
static volatile bool s_pending = false;

static void fallback_cb(void *arg)
{
    firmware_mark_valid("ran for a minute");
}

void firmware_valid_start(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(running, &state);   // fails for a USB-flashed image: no state, not pending
    ESP_LOGI(TAG, "Running %s from %s", esp_app_get_description()->version, running->label);
    if (state != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }
    s_pending = true;
    ESP_LOGW(TAG, "New firmware on trial: it is kept once the link comes up (or after %d s)", FIRMWARE_VALID_AFTER_MS / 1000);
    const esp_timer_create_args_t args = {.callback = fallback_cb, .name = "fw_valid"};
    esp_timer_handle_t timer;
    if (esp_timer_create(&args, &timer) == ESP_OK) {
        esp_timer_start_once(timer, (uint64_t)FIRMWARE_VALID_AFTER_MS * 1000);
    }
}

void firmware_mark_valid(const char *reason)
{
    if (!s_pending) {
        return;
    }
    s_pending = false;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "New firmware kept (%s)", reason);
    } else {
        ESP_LOGE(TAG, "Couldn't mark the new firmware good: %s", esp_err_to_name(err));
    }
}
