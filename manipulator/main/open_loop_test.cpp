// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Debugging tool: spins the motor open loop, with no encoder. Enable with OPEN_LOOP_TEST in main.cpp.
//
// For checking the DRV8311 and the motor wiring without the magnetic encoder fitted. The rotary
// knob changes the speed (1 rad/s a detent, negative turns the other way, 0 stops). The speed,
// VM, the driver's nFAULT and SLEEP lines, the PWM duty read back from the pins and SimpleFOC's own
// phase voltages and duties go to the serial console once a second.
//
// Open loop pushes a fixed voltage round the windings whatever the rotor does, so the motor
// warms up even when it isn't turning. OPEN_LOOP_VOLTS keeps that to about 0.3 A.
//
// The log also reads the three PWM pins and SLEEP back on the ESP32: duty is the fraction of
// samples the pin was high, so it shows the ESP32 side is driving the DRV8311 whatever the motor does.
#include "open_loop_test.h"
#include "board_pins.h"
#include "inputs.h"
#include "power.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "esp_rom_sys.h"

static const char *TAG = "open_loop";

#define OPEN_LOOP_VOLTS     3.0f   // phase voltage limit, as the normal firmware (about 0.3 A in 9.6 ohm)
#define OPEN_LOOP_START     5.0f   // rad/s at start, about 0.8 turns a second
#define OPEN_LOOP_MAX       30.0f  // rad/s, either way

static BLDCMotor olMotor = BLDCMotor(7);
static BLDCDriver3PWM olDriver = BLDCDriver3PWM(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_PWM_C_PIN, MOTOR_SLEEP_PIN);

// Fraction of time a PWM pin reads high, sampled over about 2 ms (many 20 kHz periods)
static float pinDuty(gpio_num_t pin)
{
  int high = 0;
  const int n = 4000;
  for (int i = 0; i < n; i++) {
    high += gpio_get_level(pin);
    esp_rom_delay_us(0);
  }
  return (float)high / n;
}

void openLoopTest(void *)
{
  ESP_LOGW(TAG, "OPEN LOOP TEST BUILD: no encoder, link or normal display");

  olDriver.pwm_frequency = 20000;
  olDriver.voltage_power_supply = power_motor_supply_volts();
  olDriver.voltage_limit = 3;
  olDriver.init();
  olMotor.linkDriver(&olDriver);
  olMotor.voltage_limit = OPEN_LOOP_VOLTS;
  olMotor.velocity_limit = OPEN_LOOP_MAX;
  olMotor.controller = MotionControlType::velocity_openloop;
  olMotor.foc_modulation = FOCModulationType::SinePWM;
  olMotor.init();
  olMotor.enable();
  // Let the PWM and SLEEP pins be read back (output only after the driver's setup)
  gpio_input_enable(MOTOR_PWM_A_PIN);
  gpio_input_enable(MOTOR_PWM_B_PIN);
  gpio_input_enable(MOTOR_PWM_C_PIN);
  gpio_input_enable(MOTOR_SLEEP_PIN);

  float target = OPEN_LOOP_START;
  uint8_t lastCount = rotaryCount;
  int64_t lastLog = 0;
  setButtonLight(BUTTON_LIGHT_BLUE);

  for (;;) {
    // Knob: one detent is 1 rad/s
    uint8_t count = rotaryCount;
    int8_t steps = (int8_t)(count - lastCount);
    lastCount = count;
    if (steps != 0) {
      target += steps;
      if (target > OPEN_LOOP_MAX) target = OPEN_LOOP_MAX;
      if (target < -OPEN_LOOP_MAX) target = -OPEN_LOOP_MAX;
    }

    olDriver.voltage_power_supply = power_motor_supply_volts();
    olMotor.move(target);   // works out the open-loop angle and voltage
    olMotor.loopFOC();      // applies them to the windings (move alone sets nothing in SimpleFOC 2.4)

    int64_t t = esp_timer_get_time();
    if (t - lastLog > 1000000) {
      lastLog = t;
      ESP_LOGI(TAG, "speed %.0f rad/s  VM %.2f V  nFAULT %s  SLEEP %d  duty A %.2f B %.2f C %.2f",
               target, power_motor_supply_volts(), gpio_get_level(MOTOR_FAULT_PIN) ? "high" : "LOW (fault)",
               gpio_get_level(MOTOR_SLEEP_PIN), pinDuty(MOTOR_PWM_A_PIN), pinDuty(MOTOR_PWM_B_PIN),
               pinDuty(MOTOR_PWM_C_PIN));
      ESP_LOGI(TAG, "  foc: status %d enabled %d drvinit %d  Ua %.2f Ub %.2f Uc %.2f  dc %.2f %.2f %.2f",
               (int)olMotor.motor_status, (int)olMotor.enabled, (int)olDriver.initialized,
               olMotor.Ua, olMotor.Ub, olMotor.Uc, olDriver.dc_a, olDriver.dc_b, olDriver.dc_c);
    }
    vTaskDelay(1);
  }
}
