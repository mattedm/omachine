// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * The dev board's addressable RGB LED, in the colours of the manipulator's button light (link_task.cpp
 * there): blue initializing, homing and ODrive setup; yellow syncing and starting; green manual; red
 * auto; off with the link lost; red blinking for an ODrive fault. Left alone, the LED shows whatever
 * it last latched (bright white after a reset glitch), so it's set at start-up and kept set.
 *
 * WS2812-type timing, sent with the RMT: 1.25 us bits, GRB order, most significant bit first.
 */

#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

// The LED's data pin: printed beside it on the board ("RGB@IO48" on DevKitC-1 v1.0 and most
// clones, IO38 on DevKitC-1 v1.1)
#define STATUS_LED_GPIO       48
#define STATUS_LED_LEVEL      16      // brightness of each colour, 0..255 (the LED is very bright)
#define STATUS_LED_PERIOD_MS  100     // how often the state is checked
#define STATUS_LED_REFRESH_MS 2000    // resend even if unchanged, in case a glitch changed the LED

#define RMT_RESOLUTION_HZ 10000000    // 0.1 us ticks

static const char *TAG = "status-led";

static rmt_channel_handle_t s_channel;
static rmt_encoder_handle_t s_encoder;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    static uint8_t grb[3];   // read by the RMT while it sends: not on the stack
    rmt_tx_wait_all_done(s_channel, 10);
    grb[0] = g;
    grb[1] = r;
    grb[2] = b;
    rmt_transmit_config_t tx = { .loop_count = 0 };
    if (rmt_transmit(s_channel, s_encoder, grb, sizeof(grb), &tx) != ESP_OK) {
        ESP_LOGW(TAG, "LED not updated");
    }
}

static void state_color(machine_state_t state, uint8_t *r, uint8_t *g, uint8_t *b)
{
    const uint8_t on = STATUS_LED_LEVEL;
    *r = *g = *b = 0;
    switch (state) {
    case MACHINE_INITIAL:
    case MACHINE_HOMING:
    case MACHINE_ODRIVE_SETUP:
        *b = on;
        break;
    case MACHINE_SYNCING:
    case MACHINE_PREAUTO:
        *r = on;
        *g = on;
        break;
    case MACHINE_MANUAL:
        *g = on;
        break;
    case MACHINE_AUTO:
        *r = on;
        break;
    case MACHINE_ODRIVE_FAULT:
        *r = on;   // blinked by the task
        break;
    default:   // MACHINE_DISCONNECTED: off
        break;
    }
}

static void status_led_task(void *arg)
{
    const machine_state_ctx_t *ctx = (const machine_state_ctx_t *)arg;
    int shown = -1;
    bool shown_blink_off = false;
    TickType_t last_send = 0;
    for (;;) {
        machine_state_t state = machine_state_get(ctx);
        TickType_t now = xTaskGetTickCount();
        bool blink_off = state == MACHINE_ODRIVE_FAULT && (now / pdMS_TO_TICKS(500)) % 2;
        if ((int)state != shown || blink_off != shown_blink_off || now - last_send >= pdMS_TO_TICKS(STATUS_LED_REFRESH_MS)) {
            uint8_t r, g, b;
            state_color(state, &r, &g, &b);
            if (blink_off) {
                r = g = b = 0;
            }
            led_set(r, g, b);
            shown = (int)state;
            shown_blink_off = blink_off;
            last_send = now;
        }
        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_PERIOD_MS));
    }
}

void status_led_start(const machine_state_ctx_t *ctx)
{
    rmt_tx_channel_config_t ch = {
        .gpio_num = STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
    };
    // Bit timings in 0.1 us ticks: 0 = 0.3 high + 0.9 low, 1 = 0.9 high + 0.3 low
    rmt_bytes_encoder_config_t enc = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    if (rmt_new_tx_channel(&ch, &s_channel) != ESP_OK || rmt_new_bytes_encoder(&enc, &s_encoder) != ESP_OK ||
        rmt_enable(s_channel) != ESP_OK) {
        ESP_LOGE(TAG, "RMT setup failed: no status LED");
        return;
    }
    // The line idles low between updates, which is also the LED's "latch" gap
    xTaskCreate(status_led_task, "Status_LED", 2048, (void *)ctx, 2, NULL);
}
