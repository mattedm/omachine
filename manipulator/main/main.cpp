// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Manipulator: haptic handle and display, linked to omachine-controller over ESP-NOW.
//   motor_task   - FOC haptic handle (core 1)
//   display_task - GC9A01 round display (core 0)
//   link_task    - ESP-NOW link to the controller, button light (core 0)
//   inputs       - rotary encoder and buttons, sampled by a 1 ms timer
//   power        - soft power latch, charging status, motor supply voltage, driver faults (core 0)
//
// From the battery, the button turns the board on once held for 3 s (it lights blue), and off
// when held for 3 s again. Plugging in USB turns the regulator on by itself. Started that way,
// with the button not held, the board is "off": only the charging screen runs, with the handle
// free and no link. A press of the button then starts it as normal, and holding the button (3 s)
// restarts it back to the charging screen while USB is still plugged in.
#include "firmware_valid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "power.h"
#include "inputs.h"
#include "motor_task.h"
#include "display_task.h"
#include "link_task.h"
#include "encoder_test.h"
#include "open_loop_test.h"
#include "sensor_test.h"

// Debugging tool: 1 runs only the rotary encoder test screen (encoder_test.cpp), with no motor
// or link, for checking the knob and its cable. Leave at 0 for normal use.
#define ENCODER_TEST 0

// Debugging tool: 1 spins the motor open loop (open_loop_test.cpp), with no encoder, link or
// display, for checking the driver and motor wiring. The knob sets the speed. Leave at 0 for normal use.
#define OPEN_LOOP_TEST 0

// Debugging tool: 1 checks the magnetic encoder is talking (sensor_test.cpp), with or without a
// magnet, results on the serial console. No motor, link or display. Leave at 0 for normal use.
#define SENSOR_TEST 0

static const char *TAG = "main";

extern "C" void app_main(void)
{
  power_init();   // first
  inputs_init();
  firmware_valid_start();

  bool displayStarted = false;
#if !ENCODER_TEST && !OPEN_LOOP_TEST && !SENSOR_TEST
  if (!power_on_usb()) {
    // From the battery: show the countdown while the button is held
    displayScreen = SCREEN_POWERING_ON;
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 16384, NULL, 1, NULL, 0);
    displayStarted = true;
  }
#endif
  // On the battery, the button must be held 3 s before the board stays on
  if (!power_wait_for_on()) {
    displayScreen = SCREEN_BLANK;   // the screen has no backlight control: clear it before the power goes
    vTaskDelay(pdMS_TO_TICKS(100));
    power_off_now();
  }
  // Turned on from the battery, but it's too low to run: say so, then off
  if (power_battery_too_low_to_start()) {
    displayScreen = SCREEN_BATTERY_LOW;
    vTaskDelay(pdMS_TO_TICKS(4000));
    displayScreen = SCREEN_BLANK;
    vTaskDelay(pdMS_TO_TICKS(100));
    while (autoButton.pressed) {
      vTaskDelay(pdMS_TO_TICKS(20));   // the button itself holds the power on: wait for it to be let go
    }
    power_off_now();
  }
  if (displayStarted) {
    displayScreen = SCREEN_STARTING;   // the handle homes next
  }
  power_start_task();

#if ENCODER_TEST
  xTaskCreatePinnedToCore([](void *) { encoderTest(); }, "EncTest", 16384, NULL, 1, NULL, 0);
  return;
#endif

#if OPEN_LOOP_TEST
  xTaskCreatePinnedToCore(openLoopTest, "OpenLoop", 6144, NULL, 5, NULL, 1);
  return;
#endif

#if SENSOR_TEST
  xTaskCreatePinnedToCore([](void *) { sensorTest(); }, "SensorTest", 6144, NULL, 1, NULL, 1);
  return;
#endif

  if (power_on_usb() && !autoButton.pressed) {
    // Charging only. Let go of the latch so that unplugging USB turns the board off.
    power_set_hold(false);
    standby = true;
    displayScreen = SCREEN_CHARGING;
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 16384, NULL, 1, NULL, 0);
    displayStarted = true;
    ESP_LOGI(TAG, "On USB: charging screen until the button is pressed");
    int unpluggedMs = 0;
    while (!powerOnRequested) {
      // Unplugged: turn off (the latch is already released, but the USB pull-up would keep it on)
      unpluggedMs = power_on_usb() ? 0 : unpluggedMs + 20;
      if (unpluggedMs >= 200) {
        ESP_LOGI(TAG, "USB unplugged on the charging screen: off");
        power_off_now();
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    power_set_hold(true);   // now running for real: stay on if USB is unplugged
    displayScreen = SCREEN_STARTING;
    ESP_LOGI(TAG, "Button pressed: starting");
  }

  setButtonLight(BUTTON_LIGHT_BLUE);   // initializing: shows the board is latched on; link_task takes over

  // FOC task (core 1 to itself, higher priority). It finds the end stops first; the link and
  // display start only after that, since the link scales positions by the end stops.
  TaskHandle_t motorTaskHandle = NULL;
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 6144, xTaskGetCurrentTaskHandle(), 5, &motorTaskHandle, 1);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  while (homingShort) {
    // The handle stopped short of the full stroke: wait for a press of either button, then home again
    setButtonLight(BUTTON_LIGHT_YELLOW);
    displayScreen = SCREEN_HOMING_SHORT;
    if (!displayStarted) {
      xTaskCreatePinnedToCore(displayTask, "DisplayTask", 16384, NULL, 1, NULL, 0);
      displayStarted = true;
    }
    uint8_t autoPresses = autoButton.presses, rotaryPresses = rotaryButton.presses;
    while (autoButton.presses == autoPresses && rotaryButton.presses == rotaryPresses) {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    homingShort = false;
    displayScreen = SCREEN_STARTING;
    setButtonLight(BUTTON_LIGHT_BLUE);
    xTaskNotifyGive(motorTaskHandle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  if (homingFailed) {
    // Motor off, no link: show why, and wait for the button to turn the board off
    setButtonLight(BUTTON_LIGHT_RED);
    displayScreen = SCREEN_HOMING_FAILED;
    if (!displayStarted) {
      xTaskCreatePinnedToCore(displayTask, "DisplayTask", 16384, NULL, 1, NULL, 0);
    }
    return;
  }

  // ESP-NOW link task (core 0)
  xTaskCreatePinnedToCore(linkTask, "LinkTask", 8192, NULL, 1, NULL, 0);

  // LCD display task (core 0)
  displayScreen = SCREEN_OPERATING;
  menuAllowed = true;   // holding the rotary button now opens the menu
  if (!displayStarted) {
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 16384, NULL, 1, NULL, 0);
  }
}
