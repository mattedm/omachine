// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "motor_task.h"
#include "link_task.h"
#include "board_pins.h"
#include "power.h"
#include "motor_calibration.h"
#include "firmware_update.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "motor";

// Magnetic encoder, one transfer per reading
#if ENCODER_MT6701
EncoderSensor sensor(SPI3_HOST, ENC_CLK_PIN, ENC_MISO_PIN, ENC_CS_PIN);   // MISO carries the MT6701's DO
#else
EncoderSensor sensor(SPI3_HOST, ENC_CLK_PIN, ENC_MISO_PIN, ENC_MOSI_PIN, ENC_CS_PIN);
#endif

//  BLDCMotor(int pp, (optional R, KV))
//  - pp  - pole pair number
//  - R   - phase resistance value - optional: one winding, half the 9.3 ohms measured between two
//          motor wires. It turns current into volts here, so too high a value drives more current
//          than current_limit (9.6 gave about 0.7 A for 0.35 A).
//  - KV  - motor KV rating [rpm/V] - optional
BLDCMotor motor = BLDCMotor(7, 4.65, 340);

// Phase current limit with a battery on J9; power_motor_amps lowers it on USB alone
#define MOTOR_CURRENT_LIMIT 0.65f

// The velocity loop's output is a current, turned into volts with the phase resistance. Its gains
// were tuned when the firmware had R = 9.6 ohm; with the real 4.65 they need scaling by 9.6/4.65 to
// give the same volts per rad/s of error, or the handle runs out of push near its target (it then
// stalled just short of the ODrive in SYNCING).
#define VELOCITY_GAIN_SCALE (9.6f / 4.65f)

// Angle loop acceleration limits (rad/s per s): gentle in MANUAL, quicker while SYNCING
#define MANUAL_OUTPUT_RAMP 300.0f   // was 140: the handle lagged a pushed carriage
#define SYNC_OUTPUT_RAMP   400.0f
// MANUAL: the handle aims this share of the way from where it is to the ODrive. Lower feels softer
// in the hand; higher follows a pushed carriage more quickly (0.5 was the original halfway).
#define MANUAL_FOLLOW      (2.0f / 3.0f)

// DRV8311H: one PWM per phase, enabled by its nSLEEP pin
static BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_PWM_C_PIN, MOTOR_SLEEP_PIN);

float endlimit = 0.0;
float startlimit = 0.0;
float midpoint = 0.0;

volatile float odriveposition;

volatile uint32_t motorLoopCount = 0;

volatile bool homingFailed = false;
volatile bool homingStalled = false;
volatile bool homingShort = false;
volatile uint8_t homingTravelPct = 0;

volatile MotorCalState motorCalState = MOTOR_CAL_IDLE;

// Give up looking for an end stop after this many turns of the motor: the handle isn't on the rail
// (or the belt is off), so it would spin for ever
#define HOMING_MAX_TURNS 10
// End stops closer than this (shaft radians) mean the handle never moved: too weak, or stuck.
// A normal homing finds about 6.5.
#define HOMING_MIN_TRAVEL 1.0f
// The full stroke between the end stops (shaft radians; homing logs show 6.52-6.53). Less than
// HOMING_SHORT_PCT of it means the handle stalled on the way (friction, or too little current)
// and the end stops are wrong: the user is asked to press a button to home again.
#define HOMING_FULL_TRAVEL 6.53f
#define HOMING_SHORT_PCT   97

void motorSetSupplyVolts(float volts)
{
  driver.voltage_power_supply = volts;
}

// The sensor alignment in NVS (see motor_calibration.h). Each value is checked against the build,
// so a different encoder or pole count makes it align afresh rather than use a stale offset.
#define CAL_NAMESPACE "motor"

static void nvsInit()
{
  esp_err_t err = nvs_flash_init();   // link_task's Wi-Fi start calls it again, harmlessly
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

static bool loadCalibration(float &zeroAngle, Direction &direction)
{
  nvs_handle_t h;
  if (nvs_open(CAL_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
    return false;   // nothing stored yet
  }
  uint32_t bits = 0;
  int8_t dir = 0;
  uint8_t encoder = 0xff, polePairs = 0;
  bool found = nvs_get_u32(h, "zea", &bits) == ESP_OK && nvs_get_i8(h, "dir", &dir) == ESP_OK &&
               nvs_get_u8(h, "enc", &encoder) == ESP_OK && nvs_get_u8(h, "pp", &polePairs) == ESP_OK;
  nvs_close(h);
  if (!found) {
    return false;
  }
  memcpy(&zeroAngle, &bits, sizeof(zeroAngle));
  if (encoder != ENCODER_MT6701 || polePairs != motor.pole_pairs || (dir != Direction::CW && dir != Direction::CCW) ||
      !isfinite(zeroAngle)) {
    ESP_LOGW(TAG, "Stored alignment is for another setup (encoder %u, %u pole pairs): ignoring it", encoder, polePairs);
    return false;
  }
  direction = (Direction)dir;
  return true;
}

static void saveCalibration()
{
  nvs_handle_t h;
  esp_err_t err = nvs_open(CAL_NAMESPACE, NVS_READWRITE, &h);
  if (err == ESP_OK) {
    uint32_t bits;
    float zeroAngle = motor.zero_electric_angle;
    memcpy(&bits, &zeroAngle, sizeof(bits));
    err = nvs_set_u32(h, "zea", bits);
    if (err == ESP_OK) err = nvs_set_i8(h, "dir", (int8_t)motor.sensor_direction);
    if (err == ESP_OK) err = nvs_set_u8(h, "enc", ENCODER_MT6701);
    if (err == ESP_OK) err = nvs_set_u8(h, "pp", (uint8_t)motor.pole_pairs);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
  }
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Stored the alignment: zero electric angle %.4f, direction %d", motor.zero_electric_angle,
             (int)motor.sensor_direction);
  } else {
    ESP_LOGE(TAG, "Couldn't store the alignment: %s", esp_err_to_name(err));
  }
}

// Sensor, driver and loop settings; the alignment (initFOC) is separate
static void setupMotor()
{
  // initialise magnetic sensor hardware
  sensor.clock_speed = 5000000;   // 5 MHz
  sensor.init();

  // pwm frequency to be used [Hz] (esp_simplefoc's MCPWM driver always runs at 20 kHz)
  driver.pwm_frequency = 20000;
  // power supply voltage [V]: VSYS, 3 V to 4.2 V on the battery and about 5 V on USB
  driver.voltage_power_supply = power_motor_supply_volts();
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 3;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  // SimpleFOC 2.4 ignores the phase resistance in voltage torque mode; estimated_current turns the
  // velocity loop's output (amps) into volts with R and back-EMF, as 2.3.5 did in voltage mode
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.LPF_current_q.Tf = 0.0;   // 2.3.5 had no current filter here

  motor.current_limit = power_motor_amps(MOTOR_CURRENT_LIMIT);
  motor.velocity_limit = 35;

  // velocity loop PID
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 250.0;
  motor.PID_velocity.limit = 300.0;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.04;

  // angle loop PID
  motor.P_angle.P = 14.2;
  motor.P_angle.I = 0.00;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = MANUAL_OUTPUT_RAMP;
  motor.P_angle.limit = 200.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.01;

  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.init();
}

// Aligns the sensor, or with zero_electric_angle and sensor_direction already set just checks in
static void alignMotor()
{
  motor.enable();   // a failed alignment disables the motor
  motor.initFOC();
  if (motor.motor_status != FOCMotorStatus::motor_ready) {
    ESP_LOGE(TAG, "FOC init failed (status %d), the motor is disabled", (int)motor.motor_status);
  }
}

// Returns false if an end stop wasn't found within HOMING_MAX_TURNS
static bool findEndStops()
{
  homingStalled = false;
  motor.PID_velocity.P = 0.2 * VELOCITY_GAIN_SCALE;
  motor.voltage_limit = 2.8;
  static const unsigned long REFRESH_INTERVAL = 500; // ms
  unsigned long lastRefreshTime;
  bool donelimit = false;
  float previousangle;
  previousangle = motor.shaft_angle+10.0;
  motor.voltage_limit = 3.0;
  motor.controller = MotionControlType::velocity;
  lastRefreshTime = millis();

  // Drive slowly into each end stop; it is reached once the shaft stops moving. The top (start)
  // first, then the bottom (end), so the handle is left resting at the bottom.
  float searchStart = motor.shaft_angle;
  while(!donelimit)
  {
    if (fabsf(motor.shaft_angle - searchStart) > HOMING_MAX_TURNS * _2PI) {
      ESP_LOGE(TAG, "No end stop after %d turns: is the handle on the rail?", HOMING_MAX_TURNS);
      return false;
    }
    motor.current_limit = power_motor_amps(MOTOR_CURRENT_LIMIT);
    motor.move(+8);
    motor.loopFOC();   // also updates the sensor
    if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
    {
      lastRefreshTime = millis();
      if (fabsf(motor.shaft_angle-previousangle)<0.002) {
        donelimit = true;
        ESP_LOGI(TAG, "Start limit: %f", motor.shaft_angle);
      }
      previousangle = motor.shaft_angle;
    }
  }
  startlimit = previousangle;

  donelimit = false;
  searchStart = motor.shaft_angle;
  while(!donelimit)
  {
    if (fabsf(motor.shaft_angle - searchStart) > HOMING_MAX_TURNS * _2PI) {
      ESP_LOGE(TAG, "No second end stop after %d turns: is the handle on the rail?", HOMING_MAX_TURNS);
      return false;
    }
    motor.current_limit = power_motor_amps(MOTOR_CURRENT_LIMIT);
    motor.move(-8);
    motor.loopFOC();   // also updates the sensor
    if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
    {
      lastRefreshTime = millis();
      if (fabsf(motor.shaft_angle-previousangle)<0.002) {
        donelimit = true;
        ESP_LOGI(TAG, "End limit: %f", motor.shaft_angle);
      }
      previousangle = motor.shaft_angle;
    }
  }
  endlimit = previousangle;
  if (fabsf(startlimit - endlimit) < HOMING_MIN_TRAVEL) {
    ESP_LOGE(TAG, "End stops only %.3f apart: the handle didn't move (stuck, or too little current%s)",
             fabsf(startlimit - endlimit), power_motor_limited() ? " with no battery" : "");
    homingStalled = true;
    return false;
  }
  float travel = fabsf(startlimit - endlimit);
  float pct = travel * 100.0f / HOMING_FULL_TRAVEL;
  homingTravelPct = (uint8_t)fminf(pct + 0.5f, 255.0f);
  homingShort = pct < HOMING_SHORT_PCT;
  if (homingShort) {
    ESP_LOGW(TAG, "Short stroke: end stops %.3f apart, %.1f %% of the full %.2f", travel, pct, HOMING_FULL_TRAVEL);
  } else {
    ESP_LOGI(TAG, "Stroke %.3f, %.1f %% of the full %.2f", travel, pct, HOMING_FULL_TRAVEL);
  }
  midpoint = (startlimit + endlimit)/2;
  odriveposition = endlimit;
  ESP_LOGI(TAG, "Midpoint: %f", midpoint);
  return true;
}

// With no battery, moving the handle fast turns the motor into a generator, and the energy it
// pushes back into VSYS has nowhere to go (the charger can't pass it back to USB), so VSYS
// overshoots and the board resets. Commanding zero current doesn't stop it: the back-EMF is
// larger than the KV-based estimate, so about 0.4 A still flowed back at 35-45 rad/s. So with no
// battery the current fades out between these shaft speeds (rad/s), and above NO_BATTERY_FADE_END
// the driver sleeps (all outputs off): the back-EMF (1-2 V here) is then far below VSYS and
// nothing comes back. It wakes again below NO_BATTERY_WAKE_SPEED.
#define NO_BATTERY_FADE_START 10.0f
#define NO_BATTERY_FADE_END   25.0f
#define NO_BATTERY_WAKE_SPEED 15.0f

// Puts the driver to sleep while the handle moves fast with no battery (see above). Uses its own
// speed over SPEED_WINDOW_US rather than shaft_velocity, which is filtered over about 40 ms and so
// noticed a sudden jerk too late; and once asleep stays asleep for at least NO_BATTERY_MIN_SLEEP_US,
// so the speed passing through zero as the handle is swung back and forth doesn't wake it mid-swing.
//
// 'powered' false (no link to the controller) keeps the driver asleep regardless, so the handle
// hangs free instead of holding a position nothing is following.
#define SPEED_WINDOW_US          5000
#define NO_BATTERY_MIN_SLEEP_US  500000
static void freeWhileFast(bool powered = true)
{
  static int64_t windowStart = 0, sleptAt = 0;
  static float windowAngle = 0, speed = 0;
  int64_t now = esp_timer_get_time();
  if (now - windowStart >= SPEED_WINDOW_US) {
    speed = fabsf(motor.shaft_angle - windowAngle) * 1e6f / (float)(now - windowStart);
    windowAngle = motor.shaft_angle;
    windowStart = now;
  }

  if (!powered) {
    if (motor.enabled) motor.disable();
  } else if (motor.enabled) {
    if (power_motor_limited() && speed > NO_BATTERY_FADE_END) {
      motor.disable();
      sleptAt = now;
    }
  } else if (!power_motor_limited() ||
             (speed < NO_BATTERY_WAKE_SPEED && now - sleptAt >= NO_BATTERY_MIN_SLEEP_US)) {
    motor.enable();
  }
}

static float runningCurrentLimit()
{
  float amps = power_motor_amps(MOTOR_CURRENT_LIMIT);
  if (power_motor_limited()) {
    float speed = fabsf(motor.shaft_velocity);
    if (speed >= NO_BATTERY_FADE_END) {
      amps = 0;
    } else if (speed > NO_BATTERY_FADE_START) {
      amps *= (NO_BATTERY_FADE_END - speed) / (NO_BATTERY_FADE_END - NO_BATTERY_FADE_START);
    }
  }
  return amps;
}

// Runs the angle loop towards 'target' until the handle has sat within MOVE_TOLERANCE of it for
// MOVE_SETTLE_US; false if it hasn't by MOVE_TIMEOUT_US (someone is holding the handle)
#define MOVE_TOLERANCE  0.05f      // shaft radians
#define MOVE_SETTLE_US  150000
#define MOVE_TIMEOUT_US 4000000
static bool moveHandleTo(float target)
{
  motor.controller = MotionControlType::angle;
  motor.voltage_limit = 3.0;
  motor.P_angle.output_ramp = SYNC_OUTPUT_RAMP;
  int64_t start = esp_timer_get_time(), settledSince = 0;
  uint32_t loops = 0;
  for (;;) {
    motor.loopFOC();
    motor.current_limit = runningCurrentLimit();
    freeWhileFast();
    motor.move(target);
    int64_t now = esp_timer_get_time();
    if (fabsf(motor.shaft_angle - target) > MOVE_TOLERANCE) {
      settledSince = 0;
    } else if (settledSince == 0) {
      settledSince = now;
    } else if (now - settledSince >= MOVE_SETTLE_US) {
      return true;
    }
    if (now - start >= MOVE_TIMEOUT_US) {
      return false;
    }
    if (++loops >= 200) {
      loops = 0;
      vTaskDelay(1);
    }
  }
}

// Moves the handle to the middle of its travel, where the alignment's small turn each way can't
// meet an end stop, aligns there and stores the result. With 'goBack', returns the handle to where
// it was. If the handle doesn't get there, or the alignment doesn't check out (no movement seen, or
// the sensor direction came out reversed, which would also flip the end stops), the previous
// alignment is kept and it returns false. SimpleFOC's pole pair check isn't used: with the belt
// and carriage on, friction makes the rotor lag the field each way, so the sweep it measures comes
// out short and the check fails although the alignment is fine.
static bool alignAtMiddle(bool goBack)
{
  float back = motor.shaft_angle;
  bool ok = moveHandleTo(midpoint);
  if (!ok) {
    ESP_LOGW(TAG, "Alignment: the handle didn't reach the middle (held?)");
  } else {
    float oldAngle = motor.zero_electric_angle;
    Direction oldDirection = motor.sensor_direction;
    motor.zero_electric_angle = NOT_SET;
    motor.sensor_direction = Direction::UNKNOWN;
    alignMotor();
    ok = motor.motor_status == FOCMotorStatus::motor_ready && motor.sensor_direction == oldDirection;
    if (ok) {
      ESP_LOGI(TAG, "Aligned at the middle: zero electric angle %.4f (was %.4f), pole pair check %s",
               motor.zero_electric_angle, oldAngle, motor.pp_check_result ? "ok" : "failed (ignored)");
      saveCalibration();
    } else {
      ESP_LOGW(TAG, "Alignment at the middle didn't check out (status %d, direction %d, was %d): kept the old one",
               (int)motor.motor_status, (int)motor.sensor_direction, (int)oldDirection);
      motor.zero_electric_angle = oldAngle;
      motor.sensor_direction = oldDirection;
      motor.motor_status = FOCMotorStatus::motor_ready;
      motor.enable();
    }
  }
  if (goBack) {
    moveHandleTo(back);
  }
  return ok;
}

void motorTask(void *parameters)
{
  setupMotor();

  nvsInit();
  float storedAngle;
  Direction storedDirection;
  bool stored = loadCalibration(storedAngle, storedDirection);
  if (stored) {
    // SimpleFOC skips the alignment when both are set
    motor.zero_electric_angle = storedAngle;
    motor.sensor_direction = storedDirection;
    ESP_LOGI(TAG, "Using the stored alignment: zero electric angle %.4f, direction %d", storedAngle, (int)storedDirection);
  } else {
    ESP_LOGI(TAG, "No stored alignment: aligning here, then again at the middle once homed");
  }
  alignMotor();

  bool homed;
  for (;;) {
    homed = findEndStops();
    if (!homed && stored) {
      // The encoder or magnet may have moved since the alignment was stored
      ESP_LOGW(TAG, "Homing failed with the stored alignment: aligning afresh and trying again");
      motor.zero_electric_angle = NOT_SET;
      motor.sensor_direction = Direction::UNKNOWN;
      alignMotor();
      stored = false;
      homed = findEndStops();
    }
    if (!homed || !homingShort) {
      break;
    }
    // Short of the full stroke: let the handle go, and home again once app_main says the
    // button was pressed (it clears homingShort first)
    motor.disable();
    xTaskNotifyGive((TaskHandle_t)parameters);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Homing again");
    motor.enable();
  }
  if (!homed) {
    motor.disable();   // stop spinning and let the shaft go
    homingFailed = true;
    xTaskNotifyGive((TaskHandle_t)parameters);
    vTaskDelete(NULL);
  }

  motor.PID_velocity.P = 0.08 * VELOCITY_GAIN_SCALE; //0.06;
  motor.voltage_limit = 2.4;
  motor.controller = MotionControlType::angle;

  if (!stored) {
    // Before the link starts, so nothing follows the handle while it moves
    alignAtMiddle(false);
  }
  vTaskDelay(pdMS_TO_TICKS(200));

  xTaskNotifyGive((TaskHandle_t)parameters);

  uint32_t loops = 0;
  for (;;) {
    if (motorCalState == MOTOR_CAL_REQUESTED) {
      // From the menu. The link reports the handle's last position meanwhile, so the machine
      // doesn't follow it to the middle and back.
      motorCalState = MOTOR_CAL_RUNNING;
      bool ok = alignAtMiddle(true);
      motorCalState = ok ? MOTOR_CAL_DONE : MOTOR_CAL_FAILED;
    }

    // IMPORTANT - call as frequently as possible
    // iterative setting FOC phase voltage (also updates the sensor)
    motor.loopFOC();

    motor.current_limit = runningCurrentLimit();
    uint8_t state = elements[OM_ELEMENT_STATE_INDEX];
    // No link: power the motor down. The first polls after it comes back still say DISCONNECTED
    // until the controller notices; it then sends SYNCING, which brings the handle (which may have
    // been moved meanwhile) back to the held ODrive. enable() resets the PIDs, so nothing jumps.
    // Likewise while a firmware update has the radio away from the link, and once the battery
    // is too low and the board is about to turn off.
    freeWhileFast(linkUp && state != OM_STATE_DISCONNECTED && !updateInProgress && !power_shutting_down());
    // AUTO or PREAUTO, and INITIAL or HOMING (the ODrive position isn't meaningful until the
    // controller has homed; SYNCING brings the handle over afterwards), and while the ODrive is
    // being set up (its motor is off the rail): leave the handle free
    if ((state == OM_STATE_AUTO) || (state == OM_STATE_PREAUTO) || (state == OM_STATE_INITIAL) || (state == OM_STATE_HOMING) ||
        (state == OM_STATE_ODRIVE_SETUP) || (state == OM_STATE_ODRIVE_FAULT))
    {
      motor.voltage_limit = 1.8;
      motor.move (motor.shaft_angle);
    }
    else if (state == OM_STATE_SYNCING)
    {
      // Bringing the handle over to the held ODrive: aim at it directly (not halfway, as in MANUAL)
      // and allow faster acceleration, so it gets there quickly instead of creeping in at the end
      motor.voltage_limit = 3.0;
      motor.P_angle.output_ramp = SYNC_OUTPUT_RAMP;
      motor.move (odriveposition);
    }
    else
    {
      // MANUAL: aiming part of the way to the ODrive (MANUAL_FOLLOW) gives the handle its soft feel
      motor.voltage_limit = 3.0;
      motor.P_angle.output_ramp = MANUAL_OUTPUT_RAMP;
      motor.move (motor.shaft_angle + MANUAL_FOLLOW * (odriveposition - motor.shaft_angle));
    }

    motorLoopCount = motorLoopCount + 1;

    // sleep one tick every 200 loops so core 1's idle task still gets to run
    if (++loops >= 200) {
      loops = 0;
      vTaskDelay (1);
    }
  }
}
