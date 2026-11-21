/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 MattEdm
 *
 * SPDX-License-Identifier: GPL-3.0-or-later AND CC0-1.0
 *
 * Started from Espressif's USB host CDC-ACM example (CC0-1.0) and since modified; the changes are GPL-3.0-or-later.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp.hpp"
#include "usb/usb_host.h"

#include "shared_types.h"
#include "machine_state.h"

using namespace esp_usb;

// Change these values to match your needs
#define EXAMPLE_BAUDRATE     (115200)
//#define EXAMPLE_BAUDRATE     (460800)
#define EXAMPLE_STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS    (8)

#define RX_STREAM_BUFFER_SIZE 4096

namespace {
static const char *TAG = "Serial Manipulator Task";
static SemaphoreHandle_t device_disconnected_sem;
static StreamBufferHandle_t rx_stream_buffer;
static machine_state_ctx_t *s_machine_ctx = nullptr;
static bool s_first_run = true;
static int8_t s_last_sent_item_index = -1;

static uint64_t now_ms()
{
    // Use FreeRTOS tick count to derive milliseconds
    return (uint64_t)xTaskGetTickCount() * (uint64_t)portTICK_PERIOD_MS;
}

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

static void enqueue_all_elements()
{
    // Push all current element values to the human-interface (manipulator) queue
    // so the TX loop can send them over USB.
    if (!s_machine_ctx || !s_machine_ctx->manipulator_queue) {
        return;
    }
    for (int i = 0; i < ELEMENT_COUNT; ++i) {
        element_update_t upd = {
            .key = (machine_element_key_t)i,
            .value = g_machine_elements[i].current_value,
        };
        (void)xQueueSend(s_machine_ctx->manipulator_queue, &upd, pdMS_TO_TICKS(10));
    }
}


/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device suddenly disconnected");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default: break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);  // blocks until an event
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}
}



/**
 * @brief State machine parser for incoming byte
 * 
 * @param[in] byte The byte to parse
 */
static void parse_byte(uint8_t byte)
{
    // State machine variables (static to persist across calls)
    typedef enum {
        PARSE_STATE_IDLE,
        PARSE_STATE_FIRST_MARKER,
        PARSE_STATE_SECOND_MARKER,
        PARSE_STATE_FIRST_ENCODER_BYTE,
        PARSE_STATE_SECOND_ENCODER_BYTE,
        PARSE_STATE_ROTARY_COUNTER_BYTE,
    } parse_state_t;
    
    static parse_state_t parse_state = PARSE_STATE_IDLE;
    static uint8_t encoder_bytes[2];
    static int8_t rotary_counter;
    static uint8_t bit_status;
    static bool last_auto_btn = false;
    static bool last_rotary_btn = false;
    static uint64_t last_auto_change_ms = 0;
    static uint64_t last_rotary_change_ms = 0;
    
    switch (parse_state) {
    case PARSE_STATE_IDLE:
        if (byte == 0xFF) {  // First Start marker
            parse_state = PARSE_STATE_FIRST_MARKER;
        }
        break;
    case PARSE_STATE_FIRST_MARKER:
        if (byte == 0xFF) {  // Second Start marker
            parse_state = PARSE_STATE_SECOND_MARKER;
        } else {
            parse_state = PARSE_STATE_IDLE;  // Not a frame start, resync
        }
        break;
    case PARSE_STATE_SECOND_MARKER:
        encoder_bytes[0] = byte;  // First encoder byte
        parse_state = PARSE_STATE_FIRST_ENCODER_BYTE;
        break;
    case PARSE_STATE_FIRST_ENCODER_BYTE:
        encoder_bytes[1] = byte;  // Second encoder byte
        parse_state = PARSE_STATE_SECOND_ENCODER_BYTE;
        break;
    case PARSE_STATE_SECOND_ENCODER_BYTE:
        rotary_counter = byte;    // Rotary counter byte
        parse_state = PARSE_STATE_ROTARY_COUNTER_BYTE;
        break;
    case PARSE_STATE_ROTARY_COUNTER_BYTE:
        bit_status = byte;        // Bit status byte
        // Full message received, process it
        
        // Convert encoder bytes to unint16 value
        {
            uint16_t encoder_value = (uint16_t)(encoder_bytes[0] | (encoder_bytes[1] << 8));
            // Send to manipulator encoder queue
            encoder_estimate_msg_t enc_msg = { .value = encoder_value };
            // Publish latest manipulator encoder to the ODrive control loop
            xQueueOverwrite(manipulator_encoder_queue, &enc_msg);  // Always keep latest

            // Rotary delta
            if (rotary_counter != 0) {
                enqueue_button_event(BUTTON_ROTARY, rotary_counter);
            }

            // Button bits
            bool autobutton = (bit_status & 0x01) != 0;
            bool rotarybutton = (bit_status & 0x02) != 0;
            bool restartdata = (bit_status & 0x04) != 0;

            uint64_t now = now_ms();

            if (autobutton != last_auto_btn && (now - last_auto_change_ms) > 80) {
                if (!autobutton && last_auto_btn) {
                    enqueue_button_event(BUTTON_AUTO_BUTTON, 0);
                    ESP_LOGI(TAG, "Button Pressed!");
                }
                last_auto_btn = autobutton;
                last_auto_change_ms = now;
            }

            if (rotarybutton != last_rotary_btn && (now - last_rotary_change_ms) > 80) {
                if (!rotarybutton && last_rotary_btn) {
                    enqueue_button_event(BUTTON_ROTARY_BUTTON, 0);
                }
                last_rotary_btn = rotarybutton;
                last_rotary_change_ms = now;
            }

            if ((restartdata || s_first_run) && (s_last_sent_item_index == -1)) {
                enqueue_all_elements();
                enqueue_button_event(BUTTON_CONTROLLER_CONNECTED, 0);
                s_first_run = false;
            }


        }
        parse_state = PARSE_STATE_IDLE;
        break;

    default:
        parse_state = PARSE_STATE_IDLE;
        break;
    }
}

/**
 * @brief Data received callback
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    // Parse directly without buffering
    for (size_t i = 0; i < data_len; i++) {
        parse_byte(data[i]);
    }
    return true;
}


/**
 * @brief TX data processor function
 * 
 * Sends initial configuration and continuously transmits manipulator data
 * 
 * @param vcp Pointer to the VCP device
 * @return true if disconnected gracefully, false on error
 */
static bool tx_processor(CdcAcmDevice *vcp, machine_state_ctx_t *machine_ctx)
{
    // Send some initial data
    ESP_LOGI(TAG, "Sending data through CdcAcmDevice");
    uint8_t initialdata[] = "\xFF\xFF\x00\x20\x00\x01"
                            "\xFF\xFF\x00\x20\x01\x01"
                            "\xFF\xFF\x00\x20\x02\x01"
                            "\xFF\xFF\x00\x20\x03\x01"
                            "\xFF\xFF\x00\x60\x04\x01"
                            "\xFF\xFF\x00\x60\x05\x03"
                            "\xFF\xFF\x00\x60\x06\x01"
                            "\xFF\xFF\x00\x60\xFF\x00";
    
    esp_err_t err = vcp->tx_blocking(initialdata, sizeof(initialdata));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Initial TX failed: %s", esp_err_to_name(err));
        return false;
    }

    int16_t odrive_position = 0x0000;
    encoder_estimate_msg_t enc_msg;

    uint8_t data[6];
    element_update_t pending_update = { .key = (machine_element_key_t)-1, .value = 0 };

    // Consume ODrive encoder updates and manipulator element updates
    while (xQueueReceive(odrive_encoder_queue, &enc_msg, -1)) {
        odrive_position = enc_msg.value;

        // Check if device disconnected (non-blocking check)
        if (xSemaphoreTake(device_disconnected_sem, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Device disconnected, breaking to reconnect");
            if (s_machine_ctx) {
                enqueue_button_event(BUTTON_CONTROLLER_DISCONNECTED, 0);
            }
            return true;  // Graceful disconnect
        }

        // Fetch next manipulator update (human-interface) if available
        if (machine_ctx && machine_ctx->manipulator_queue) {
            element_update_t temp;
            if (xQueueReceive(machine_ctx->manipulator_queue, &temp, 0) == pdTRUE) {
                pending_update = temp;
            } else {
                pending_update.key = (machine_element_key_t)-1;
                pending_update.value = 0;
            }
        } else {
            pending_update.key = (machine_element_key_t)-1;
            pending_update.value = 0;
        }

        s_last_sent_item_index = (int8_t)pending_update.key;

        data[0] = 0xFF;
        data[1] = 0xFF;
        data[2] = (uint8_t)(odrive_position & 0xFF);        // Low byte
        data[3] = (uint8_t)((odrive_position >> 8) & 0xFF); // High byte
        data[4] = (uint8_t)s_last_sent_item_index;
        data[5] = (uint8_t)pending_update.value;

        err = vcp->tx_blocking(data, sizeof(data));

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TX failed: %s, device may be disconnected", esp_err_to_name(err));
            return false;  // Error disconnect
        }
       

        /*ESP_LOGI(TAG, "Sent %zu bytes: pos=%d idx=%d val=%d", sizeof(data), odrive_position,
                 (int)s_last_sent_item_index, (int)pending_update.value);*/
    }
    return true;
}

/**
 * @brief Main application
 *
 * This function sets Virtual COM Port drivers for the CH340 manipulator ESP32.
 */
extern "C" void serial_manipulator_task(void *pvParameters)
{
    s_machine_ctx = reinterpret_cast<machine_state_ctx_t *>(pvParameters);
    if (s_machine_ctx) {
        machine_state_init_defaults(s_machine_ctx);
    }

    //esp_log_level_set(TAG, ESP_LOG_WARN);  // Only shows WARN, ERROR, and VERBOSE

    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Create stream buffer for RX data
    rx_stream_buffer = xStreamBufferCreate(RX_STREAM_BUFFER_SIZE, 1);
    assert(rx_stream_buffer != NULL);

    // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // Register VCP drivers to VCP service
    //VCP::register_driver<FT23x>();
    //VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();

    // Do everything else in a loop, so we can demonstrate USB device reconnections
    while (true) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = 5000, // 5 seconds, enough time to plug the device in or experiment with timeout
            .out_buffer_size = 512,
            .in_buffer_size = 512,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        // You don't need to know the device's VID and PID. Just plug in any device and the VCP service will load correct (already registered) driver for the device
        ESP_LOGI(TAG, "Opening any VCP device...");
        auto vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

        if (vcp == nullptr) {
            ESP_LOGI(TAG, "Failed to open VCP device");
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "Setting up line coding");
        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = EXAMPLE_BAUDRATE,
            .bCharFormat = EXAMPLE_STOP_BITS,
            .bParityType = EXAMPLE_PARITY,
            .bDataBits = EXAMPLE_DATA_BITS,
        };
        ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

        // Start transmitting manipulator data
        tx_processor(vcp.get(), s_machine_ctx);
      
        // Keep the task alive to keep receiving data and handling device events
        vTaskDelay(pdMS_TO_TICKS(1000));
        

        // We are done. Wait for device disconnection and start over
        ESP_LOGI(TAG, "Done. You can reconnect the VCP device to run again.");
        //xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
    }
}
