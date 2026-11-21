// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "power.h"
#include "board_pins.h"
#include "inputs.h"
#include "motor_task.h"
#include "display_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "hal/usb_serial_jtag_ll.h"

#include <math.h>

static const char *TAG = "power";

#define VBAT_DIVIDER 2.0f       // R16, R17: 1M / 1M
#define VBAT_SAMPLES 16
#define VM_ON_USB 5.0f          // the BQ24075 passes USB straight through to VSYS
#define VM_MIN 3.0f             // the DRV8311's minimum; also guards the duty calculation against a bad reading
#define VM_MAX 5.5f             // the BQ24075's OUT regulation
#define POWER_TASK_PERIOD_MS 100
#define FAULT_REPORT_SAMPLES 2  // nFAULT also pulses low for up to 3 ms when the driver wakes
#define NO_BATTERY_VOLTS 2.5f   // below this, nothing is on J9 (or its protection has cut out)
#define BATTERY_FILTER 0.1f     // per POWER_TASK_PERIOD_MS: smooths the reading over about a second
// The percentage shown is steadier than that (see updateBatteryPercent)
#define SOC_FILTER 0.005f       // per POWER_TASK_PERIOD_MS: about 20 s
#define SOC_FALL_S_PER_PCT 10   // on the battery it may only fall, at most 1 % every 10 s
#define SOC_RISE_S_PER_PCT 20   // on USB (charging) it may only rise, at most 1 % every 20 s

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t adc_cali;
static float motorSupply = VM_ON_USB;
static volatile float batteryVolts = 0.0f;
static float socVolts = 0.0f;              // battery voltage averaged over about 20 s
static volatile float shownPercent = -1;   // the percentage reported; < 0 until there is a battery
static volatile bool motorLimited = false;
static volatile bool shuttingDown = false;

static bool onUsbPower()
{
  return gpio_get_level(USB_POWER_GOOD_PIN) == 0;
}

bool power_on_usb()
{
  return onUsbPower();
}

void power_set_hold(bool on)
{
  gpio_set_level(POWER_HOLD_PIN, on ? 1 : 0);
}

// False if the ADC failed, leaving volts alone
static bool readBatteryVolts(float &volts)
{
  int sum_mv = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    int raw, mv;
    if (adc_oneshot_read(adc, VBAT_ADC_CHANNEL, &raw) != ESP_OK ||
        adc_cali_raw_to_voltage(adc_cali, raw, &mv) != ESP_OK) {
      return false;
    }
    sum_mv += mv;
  }
  volts = VBAT_DIVIDER * (sum_mv / VBAT_SAMPLES) / 1000.0f;
  return true;
}

// VSYS is the battery unless USB is powering it. There is no sense line on VSYS itself.
static void updateMotorSupply()
{
  float volts = motorSupply;
  if (onUsbPower()) {
    volts = VM_ON_USB;
  } else if (!readBatteryVolts(volts)) {
    return;
  }
  if (volts < VM_MIN) volts = VM_MIN;
  if (volts > VM_MAX) volts = VM_MAX;
  motorSupply = volts;
}

static void updateBatteryPercent();

static void updateBattery()
{
  float volts;
  if (!readBatteryVolts(volts)) {
    return;
  }
  float filtered = batteryVolts;
  batteryVolts = (filtered == 0.0f) ? volts : filtered + BATTERY_FILTER * (volts - filtered);
  updateBatteryPercent();
}

// On USB with no battery the charger (U2) can only pass about 1 A, set by R9, and has nothing to
// make up a peak above that, so VSYS and 3V3 collapse (brownout reset) when the handle is pushed.
// Limit the motor current then. Comes on at once; goes off only once the battery has been back a while.
#define MOTOR_UNLIMIT_MS 2000
static void updateMotorLimit()
{
  static int batteryMs = 0;
  bool noBattery = power_charge_state() == CHARGE_NO_BATTERY;
  if (noBattery) {
    batteryMs = 0;
    if (!motorLimited) {
      motorLimited = true;
      ESP_LOGW(TAG, "No battery: motor limited to %.2f A so USB alone can power it", NO_BATTERY_MOTOR_AMPS);
    }
  } else if (motorLimited) {
    batteryMs += POWER_TASK_PERIOD_MS;
    if (batteryMs >= MOTOR_UNLIMIT_MS) {
      motorLimited = false;
      ESP_LOGI(TAG, "Battery back: full motor power");
    }
  }
}

float power_motor_amps(float amps)
{
  return (motorLimited && amps > NO_BATTERY_MOTOR_AMPS) ? NO_BATTERY_MOTOR_AMPS : amps;
}

bool power_motor_limited()
{
  return motorLimited;
}

ChargeState power_charge_state()
{
  if (!onUsbPower()) return CHARGE_NOT_ON_USB;
  if (batteryVolts < NO_BATTERY_VOLTS) return CHARGE_NO_BATTERY;
  return gpio_get_level(CHARGE_STATUS_PIN) == 0 ? CHARGE_CHARGING : CHARGE_DONE;
}

// Resting voltage of a typical Li-ion/LiPo cell at 0, 5, 10 ... 100 % of the typical curve. This
// cell's charger finishes at about the 95 % point (4.15 V), so that reads as 100 %: the scale is
// stretched by 100/95 and the last point (4.20 V) is past the top.
static const float socTable[] = {3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
                                 3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
#define SOC_FULL_PCT 95.0f   // the typical curve's percentage shown as 100

static float voltsToPercent(float v)
{
  const int n = sizeof(socTable) / sizeof(socTable[0]);
  if (v <= socTable[0]) return 0;
  if (v >= socTable[n - 1]) return 100;
  int i = 1;
  while (v > socTable[i]) i++;
  float frac = (v - socTable[i - 1]) / (socTable[i] - socTable[i - 1]);
  return fminf(100.0f, 5.0f * (i - 1 + frac) * 100.0f / SOC_FULL_PCT);
}

// The cell's voltage moves with load (homing, the radio, the motor) and charging lifts it toward
// 4.2 V, and on the flat middle of the curve 20 mV is about 5 %, so the raw percentage swings. The
// shown percentage follows a 20 s average, only falls on the battery and only rises while on USB,
// each at a limited rate, and starts from a reading taken before the motor runs.
//
// 100 % is kept for the charger saying it has finished: while it is still charging the shown
// percentage stops at 99, and once it is done it shows 100 (with "Charged" on the charging screen).
#define CHARGING_MAX_PCT 99.0f
static void updateBatteryPercent()
{
  if (batteryVolts < NO_BATTERY_VOLTS) {
    shownPercent = -1; socVolts = 0;   // no battery: start afresh when one is connected
    return;
  }
  socVolts = (socVolts == 0.0f) ? batteryVolts : socVolts + SOC_FILTER * (batteryVolts - socVolts);
  ChargeState charge = power_charge_state();
  if (charge == CHARGE_DONE) {
    shownPercent = 100;
    return;
  }
  float target = voltsToPercent(socVolts);
  if (charge == CHARGE_CHARGING) {
    target = fminf(target, CHARGING_MAX_PCT);
    if (shownPercent > CHARGING_MAX_PCT) shownPercent = CHARGING_MAX_PCT;   // charging again after "done"
  }
  float shown = shownPercent;
  if (shown < 0) {
    shownPercent = target;
    return;
  }
  if (onUsbPower()) {
    float step = (float)POWER_TASK_PERIOD_MS / (SOC_RISE_S_PER_PCT * 1000.0f);
    if (target > shown) shown += fminf(target - shown, step);
  } else {
    float step = (float)POWER_TASK_PERIOD_MS / (SOC_FALL_S_PER_PCT * 1000.0f);
    if (target < shown) shown -= fminf(shown - target, step);
  }
  shownPercent = shown;
}

uint8_t power_battery_percent()
{
  float p = shownPercent;
  if (p < 0) return (uint8_t)lroundf(voltsToPercent(batteryVolts));
  return (uint8_t)lroundf(p);
}

static const char *resetReasonName(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_POWERON:   return "power on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "crash (panic)";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "wake from deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (3V3 dipped)";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "unknown";
  }
}

void power_init()
{
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_BROWNOUT || reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
      reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
    ESP_LOGW(TAG, "Last reset: %s", resetReasonName(reason));
  } else {
    ESP_LOGI(TAG, "Last reset: %s", resetReasonName(reason));
  }

  gpio_config_t inputs = {};   // all have pull-ups on the board
  inputs.pin_bit_mask = (1ULL << USB_POWER_GOOD_PIN) | (1ULL << CHARGE_STATUS_PIN) | (1ULL << MOTOR_FAULT_PIN);
  inputs.mode = GPIO_MODE_INPUT;
  ESP_ERROR_CHECK(gpio_config(&inputs));

  gpio_hold_dis(POWER_HOLD_PIN);   // held low through deep sleep by power_off_now
  gpio_hold_dis(LCD_BL_PIN);

  // From the battery, the latch waits for power_wait_for_on: the button holds the regulator on meanwhile
  gpio_set_level(POWER_HOLD_PIN, onUsbPower() ? 1 : 0);
  gpio_config_t hold = {};
  hold.pin_bit_mask = 1ULL << POWER_HOLD_PIN;
  hold.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&hold));

  adc_oneshot_unit_init_cfg_t unit = {};
  unit.unit_id = VBAT_ADC_UNIT;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
  adc_oneshot_chan_cfg_t chan = {};
  chan.atten = ADC_ATTEN_DB_12;   // reads up to about 3.1 V; a full battery is 2.1 V here
  chan.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, VBAT_ADC_CHANNEL, &chan));
  adc_cali_curve_fitting_config_t cali = {};
  cali.unit_id = VBAT_ADC_UNIT;
  cali.chan = VBAT_ADC_CHANNEL;
  cali.atten = chan.atten;
  cali.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali, &adc_cali));
#if MOTOR_CURRENT_LOG
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, MOTOR_SOA_ADC_CHANNEL, &chan));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, MOTOR_SOB_ADC_CHANNEL, &chan));
#endif

  updateMotorSupply();
  updateBattery();
  ESP_LOGI(TAG, "Motor supply %.2f V (%s)", motorSupply, onUsbPower() ? "USB" : "battery");
  updateMotorLimit();
}

// The ESP32's USB pulls D+ up to 3.3 V. Through U7's steering diode that puts about 2.5 V on VBUS
// with no cable plugged in, and D2 passes it on to the regulator's EN, so the board holds itself
// on. Switching the pull-up off breaks that loop. (Only USB being plugged in turns it back on: the
// ROM sets it up again at every boot.)
static void releaseUsbPullup()
{
  usb_serial_jtag_pull_override_vals_t pulls = {};   // no pull-ups or pull-downs on D+ and D-
  usb_serial_jtag_ll_phy_enable_pull_override(&pulls);
}

// After the latch is let go, VBUS's 10 uF has to drain through D2 and R20 before EN goes low
#define POWER_DOWN_WAIT_MS 3000

void power_off_now()
{
  releaseUsbPullup();
  gpio_set_level(POWER_HOLD_PIN, 0);
  gpio_set_level(MOTOR_SLEEP_PIN, 0);
  gpio_set_level(LCD_BL_PIN, 0);
  gpio_config_t out = {};
  out.pin_bit_mask = (1ULL << LCD_BL_PIN) | (1ULL << MOTOR_SLEEP_PIN);
  out.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&out));

  // Wait for the button to be let go first: it holds the power on itself
  bool released = !autoButton.pressed;
  for (int ms = 0; ms < POWER_DOWN_WAIT_MS; ms += 10) {
    if (onUsbPower()) {
      esp_restart();   // plugged in before the power went: come up on the charging screen
    }
    if (!autoButton.pressed) {
      released = true;
    } else if (released) {
      // Pressed again before the power went, so the button now holds it on: start afresh, as from
      // off, rather than carry on here (where this press would count as a hold to turn off)
      esp_restart();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Still running: the power is stuck on. Red: the button line is released, so EN is
  // held up some other way. Yellow: the button line still reads pressed (button or its cable).
  bool buttonLine = gpio_get_level(BUTTON_PIN) == 0;
  ESP_LOGW(TAG, "Power still on with the latch released (button line %s)", buttonLine ? "pressed" : "released");
  for (int i = 0; i < 5; i++) {
    setButtonLight(buttonLine ? BUTTON_LIGHT_YELLOW : BUTTON_LIGHT_RED);
    vTaskDelay(pdMS_TO_TICKS(200));
    setButtonLight(BUTTON_LIGHT_OFF);
    vTaskDelay(pdMS_TO_TICKS(300));
  }

  // Deep sleep, keeping the latch and the backlight low, until PGOOD goes low (USB plugged in)
  gpio_hold_en(POWER_HOLD_PIN);
  gpio_hold_en(LCD_BL_PIN);
  esp_sleep_enable_ext1_wakeup_io(1ULL << USB_POWER_GOOD_PIN, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_deep_sleep_start();
}

bool power_wait_for_on()
{
  if (onUsbPower()) {
    return true;   // USB keeps the regulator on; main decides between the charging screen and starting
  }
  // Timed from boot, so the time the chip takes to start counts towards the hold
  while (autoButton.pressed) {
    if (esp_timer_get_time() >= (int64_t)POWER_ON_HOLD_MS * 1000) {
      gpio_set_level(POWER_HOLD_PIN, 1);
      setButtonLight(BUTTON_LIGHT_BLUE);   // latched: the button can be let go
      ESP_LOGI(TAG, "Held for %d ms: on", POWER_ON_HOLD_MS);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Let go too soon, or no press at all: the restart after turning off, a bounce of the button as
  // it was released, or a knock. The regulator should go off now the button isn't holding it on.
  ESP_LOGI(TAG, "Button not held for %d ms: staying off", POWER_ON_HOLD_MS);
  return false;
}

bool power_battery_too_low_to_start()
{
  float volts;
  if (onUsbPower() || !readBatteryVolts(volts)) {
    return false;
  }
  if (volts < BATTERY_START_VOLTS) {
    ESP_LOGW(TAG, "Battery %.2f V, below %.2f V: not starting", volts, BATTERY_START_VOLTS);
    return true;
  }
  return false;
}

bool power_shutting_down()
{
  return shuttingDown;
}

float power_motor_supply_volts()
{
  return motorSupply;
}

#define USB_RELEASE_WAIT_MS 2000   // on USB, restart after this even if the button still reads pressed

static void powerOff()
{
  ESP_LOGI(TAG, "Powering off");
  gpio_set_level(MOTOR_SLEEP_PIN, 0);
  gpio_set_level(LCD_BL_PIN, 0);
  gpio_set_level(POWER_HOLD_PIN, 0);

  // The button itself keeps the regulator on until it is let go, then the power goes. On USB,
  // restart instead, which comes up on the charging screen since the button is no longer held.
  // On USB, don't wait for ever: a button line that kept reading pressed left the board stuck on
  // "Turning off" (Nov 2026). Restarting with it still pressed is safe: a press held at start-up
  // is ignored, so it can't turn the board off again.
  int waitedMs = 0;
  while (autoButton.pressed) {
    if (waitedMs % 1000 == 0) {
      ESP_LOGW(TAG, "Waiting for the button to be let go (line %s, USB %s)",
               gpio_get_level(BUTTON_PIN) == 0 ? "pressed" : "released", onUsbPower() ? "yes" : "no");
    }
    if (onUsbPower() && waitedMs >= USB_RELEASE_WAIT_MS) {
      ESP_LOGW(TAG, "Button still pressed after %d ms on USB: restarting anyway", waitedMs);
      vTaskDelay(pdMS_TO_TICKS(50));   // let the log out
      esp_restart();
    }
    vTaskDelay(pdMS_TO_TICKS(POWER_TASK_PERIOD_MS));
    waitedMs += POWER_TASK_PERIOD_MS;
  }
  if (onUsbPower()) {
    esp_restart();
  }
  power_off_now();
}

// Battery below BATTERY_CUTOFF_VOLTS for BATTERY_CUTOFF_MS while running from it: motor off, show
// the message for a few seconds, then off. batteryVolts is already smoothed over about a second.
#define OFF_MESSAGE_MS 4000
static volatile DisplayScreen requestedOffScreen = SCREEN_OPERATING;   // SCREEN_OPERATING: none

// Motor off, 'screen' (saying why) for OFF_MESSAGE_MS, then off. Plugging USB in meanwhile
// restarts onto the charging screen; already on USB, off is that restart anyway (see powerOff).
static void showThenOff(DisplayScreen screen)
{
  shuttingDown = true;               // the motor task powers the motor down
  gpio_set_level(MOTOR_SLEEP_PIN, 0);
  menuOpen = false;
  displayScreen = screen;
  bool usbAtStart = onUsbPower();
  for (int ms = 0; ms < OFF_MESSAGE_MS; ms += POWER_TASK_PERIOD_MS) {
    if (!usbAtStart && onUsbPower()) {
      esp_restart();   // plugged in meanwhile: come up on the charging screen
    }
    vTaskDelay(pdMS_TO_TICKS(POWER_TASK_PERIOD_MS));
  }
  displayScreen = SCREEN_BLANK;   // the screen has no backlight control: clear it before the power goes
  vTaskDelay(pdMS_TO_TICKS(100));
  powerOff();
}

void power_request_off(DisplayScreen screen)
{
  requestedOffScreen = screen;
}

static void checkBatteryCutoff()
{
  static int lowMs = 0;
  float volts = batteryVolts;
  if (onUsbPower() || volts < NO_BATTERY_VOLTS || volts >= BATTERY_CUTOFF_VOLTS) {
    lowMs = 0;
    return;
  }
  lowMs += POWER_TASK_PERIOD_MS;
  if (lowMs < BATTERY_CUTOFF_MS) {
    return;
  }
  ESP_LOGW(TAG, "Battery %.2f V, below %.2f V for %d s: turning off", volts, BATTERY_CUTOFF_VOLTS,
           BATTERY_CUTOFF_MS / 1000);
  showThenOff(SCREEN_BATTERY_LOW);
}

static void powerTask(void *parameters)
{
  int faultSamples = 0;
  for (;;) {
    updateMotorSupply();
    motorSetSupplyVolts(motorSupply);
    updateBattery();
    updateMotorLimit();
    checkBatteryCutoff();
    if (requestedOffScreen != SCREEN_OPERATING) {
      showThenOff(requestedOffScreen);
    }

    if (gpio_get_level(MOTOR_FAULT_PIN) == 0) {
      if (++faultSamples == FAULT_REPORT_SAMPLES) {
        ESP_LOGW(TAG, "Motor driver fault");
      }
    } else {
      faultSamples = 0;
    }

    // Log the main button, to tell real presses from a line that reads pressed by itself
    static bool lastPressed = false;
    static int64_t pressedAtUs = 0;
    bool pressed = autoButton.pressed;
    if (pressed != lastPressed) {
      int64_t now = esp_timer_get_time();
      if (pressed) {
        pressedAtUs = now;
        ESP_LOGI(TAG, "Main button pressed (line %s, USB %s)", gpio_get_level(BUTTON_PIN) == 0 ? "low" : "high",
                 onUsbPower() ? "yes" : "no");
      } else {
        ESP_LOGI(TAG, "Main button released after %lld ms", (long long)((now - pressedAtUs) / 1000));
      }
      lastPressed = pressed;
    }

    if (powerOffRequested) {
      powerOffRequested = false;
      ESP_LOGW(TAG, "Main button held %d ms: turning off (line %s, USB %s)", POWER_OFF_HOLD_MS,
               gpio_get_level(BUTTON_PIN) == 0 ? "pressed" : "released", onUsbPower() ? "yes" : "no");
      powerOff();
    }

    vTaskDelay(pdMS_TO_TICKS(POWER_TASK_PERIOD_MS));
  }
}

#if MOTOR_CURRENT_LOG
// Diagnostic: peak motor phase current from the DRV8311's SOA and SOB. The outputs are only valid
// while that phase's low side is on, and this samples them free-running, not in step with the PWM,
// so most samples catch part of a period; the peak over a few thousand samples still shows the
// largest current. Zero is a slow average of all samples (the phase currents average out to 0).
#define CSA_VOLTS_PER_AMP 2.0f
#define CSA_SATURATED_V   1.35f   // |reading - zero| beyond this is at the end of the linear range (about 0.7 A)
#define CURRENT_WINDOW_MS 200
#define CURRENT_LOG_ABOVE_A 0.15f // log a window only if its peak is above this
static void currentLogTask(void *parameters)
{
  float zeroA = 1.65f, zeroB = 1.65f;
  float maxSeen = 0;
  int64_t windowStart = esp_timer_get_time(), lastSummary = windowStart;
  float peakA = 0, peakB = 0, peakSpeed = 0;
  int samples = 0, saturated = 0;
  uint32_t lastLoops = motorLoopCount;
  int activeWindows = 0;   // the first window after the motor starts only settles the zero
  for (;;) {
    for (int i = 0; i < 50; i++) {
      int rawA, rawB, mvA, mvB;
      if (adc_oneshot_read(adc, MOTOR_SOA_ADC_CHANNEL, &rawA) != ESP_OK ||
          adc_oneshot_read(adc, MOTOR_SOB_ADC_CHANNEL, &rawB) != ESP_OK ||
          adc_cali_raw_to_voltage(adc_cali, rawA, &mvA) != ESP_OK ||
          adc_cali_raw_to_voltage(adc_cali, rawB, &mvB) != ESP_OK) {
        continue;
      }
      float vA = mvA / 1000.0f, vB = mvB / 1000.0f;
      zeroA += 0.0005f * (vA - zeroA);
      zeroB += 0.0005f * (vB - zeroB);
      float dA = fabsf(vA - zeroA), dB = fabsf(vB - zeroB);
      if (dA > CSA_SATURATED_V || dB > CSA_SATURATED_V) saturated++;
      if (dA / CSA_VOLTS_PER_AMP > peakA) peakA = dA / CSA_VOLTS_PER_AMP;
      if (dB / CSA_VOLTS_PER_AMP > peakB) peakB = dB / CSA_VOLTS_PER_AMP;
      samples++;
    }
    float speed = fabsf(motor.shaft_velocity);
    if (speed > peakSpeed) peakSpeed = speed;
    int64_t now = esp_timer_get_time();
    if (now - windowStart >= CURRENT_WINDOW_MS * 1000) {
      // Only while the control loop runs (after homing): asleep, the outputs sit at the top of the range
      uint32_t loops = motorLoopCount;
      bool active = loops != lastLoops;
      lastLoops = loops;
      activeWindows = active ? activeWindows + 1 : 0;
      if (activeWindows < 2) {
        peakA = peakB = peakSpeed = 0;
        samples = saturated = 0;
        windowStart = now;
        continue;
      }
      float peak = fmaxf(peakA, peakB);
      if (peak > maxSeen) maxSeen = peak;
      if (peak > CURRENT_LOG_ABOVE_A || saturated > 0 || peakSpeed > 5.0f) {
        ESP_LOGI(TAG, "Phase current peak A %.2f B %.2f A%s  speed up to %.0f rad/s  limit %.2f A%s",
                 peakA, peakB, saturated ? "  SATURATED: over about 0.7 A" : "", peakSpeed, motor.current_limit,
                 motor.enabled ? "" : "  DRIVER ASLEEP");
      }
      if (now - lastSummary >= 5000000) {
        lastSummary = now;
        ESP_LOGI(TAG, "Current sense zero A %.3f B %.3f V, highest peak so far %.2f A", zeroA, zeroB, maxSeen);
      }
      peakA = peakB = peakSpeed = 0;
      samples = saturated = 0;
      windowStart = now;
    }
    vTaskDelay(1);   // let core 0's idle task run
  }
}
#endif

void power_start_task()
{
  xTaskCreatePinnedToCore(powerTask, "PowerTask", 4096, NULL, 1, NULL, 0);
#if MOTOR_CURRENT_LOG
  xTaskCreatePinnedToCore(currentLogTask, "CurrentLog", 4096, NULL, 1, NULL, 0);
#endif
}
