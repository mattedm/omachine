// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "inputs.h"
#include "rotary.h"

#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "inputs";

volatile uint8_t rotaryCount = 0;
volatile uint8_t menuRotaryCount = 0;

static Rotary r1(ROTARY_PIN1, ROTARY_PIN2);

DebouncedButton autoButton = {BUTTON_PIN, false, 0, 0, false, 0, 0};
DebouncedButton rotaryButton = {ROTARY_BUTTON_PIN, false, 0, 0, false, 0, 0};

volatile bool powerOffRequested = false;
volatile bool powerOffCountdown = false;
volatile bool standby = false;
volatile bool powerOnRequested = false;
volatile bool menuAllowed = false;
volatile bool menuCountdown = false;
volatile bool menuOpen = false;

static void rotaryState()
{
  unsigned char result = r1.process();
  int8_t step = (result == DIR_CW) ? 1 : (result == DIR_CCW) ? -1 : 0;
  if (ROTARY_REVERSED) {
    step = -step;
  }
  if (menuOpen) {
    menuRotaryCount = menuRotaryCount + step;
  } else {
    rotaryCount = rotaryCount + step;
  }
}

// Called every 1 ms: a press counts once the button has been released and stable for BUTTON_DEBOUNCE_MS
static void debounceButton(DebouncedButton &b)
{
  bool raw = !gpio_get_level(b.pin);   // active low
  if (raw == b.pressed) {
    b.stableMs = 0;
    return;
  }
  if (++b.stableMs >= BUTTON_DEBOUNCE_MS) {
    b.pressed = raw;
    b.stableMs = 0;
    if (!raw) {
      if (b.ignoreRelease) {
        // held for the power or the menu: not a press
      } else if (menuOpen) {
        b.menuPresses = b.menuPresses + 1;
      } else {
        b.presses = b.presses + 1;
      }
      b.ignoreRelease = false;
    }
  }
}

// Called every 1 ms. Held past POWER_OFF_COUNTDOWN_MS, a press stops counting as a press and the
// off countdown starts; held to POWER_OFF_HOLD_MS, it turns the board off. A press that turned
// the board on can't turn it off, however long it lasts.
static void checkPowerOffHold()
{
  static bool armed = false;   // this press can turn the board off
  if (!autoButton.pressed) {
    autoButton.heldMs = 0;
    armed = false;
    powerOffCountdown = false;
    return;
  }
  if (autoButton.heldMs >= POWER_OFF_HOLD_MS) {
    return;
  }
  if (++autoButton.heldMs == 1) {
    armed = !autoButton.ignoreRelease;
  }
  if (!armed) {
    return;
  }
  if (autoButton.heldMs == POWER_OFF_COUNTDOWN_MS) {
    autoButton.ignoreRelease = true;   // no longer a press for the controller
    powerOffCountdown = true;
  } else if (autoButton.heldMs == POWER_OFF_HOLD_MS) {
    powerOffRequested = true;
  }
}

// Called every 1 ms: like checkPowerOffHold, for the rotary button and the menu
static void checkMenuHold()
{
  if (!rotaryButton.pressed) {
    rotaryButton.heldMs = 0;
    menuCountdown = false;
    return;
  }
  if (rotaryButton.heldMs >= MENU_HOLD_MS) {
    return;
  }
  ++rotaryButton.heldMs;
  if (!menuAllowed || menuOpen) {
    return;   // a hold in the menu is just a press
  }
  if (rotaryButton.heldMs == MENU_COUNTDOWN_MS) {
    rotaryButton.ignoreRelease = true;   // no longer a press for the controller
    menuCountdown = true;
  } else if (rotaryButton.heldMs == MENU_HOLD_MS && menuCountdown) {
    menuCountdown = false;
    menuOpen = true;
  }
}

static void periodic_timer_callback(void *arg)
{
  if (standby) {
    // Turning the knob or pressing it on the charging screen mustn't reach the controller later
    debounceButton(autoButton);
    if (autoButton.pressed) {
      autoButton.ignoreRelease = true;
      standby = false;
      powerOnRequested = true;
    }
    return;
  }
  rotaryState();
  debounceButton(autoButton);
  debounceButton(rotaryButton);
  checkPowerOffHold();
  checkMenuHold();
}

void setButtonLight(ButtonLightColor color)
{
  // active low
  gpio_set_level(BUTTON_LIGHT_R_PIN, (color == BUTTON_LIGHT_RED || color == BUTTON_LIGHT_YELLOW) ? 0 : 1);
  gpio_set_level(BUTTON_LIGHT_G_PIN, (color == BUTTON_LIGHT_GREEN || color == BUTTON_LIGHT_YELLOW) ? 0 : 1);
  gpio_set_level(BUTTON_LIGHT_B_PIN, color == BUTTON_LIGHT_BLUE ? 0 : 1);
}

void inputs_init()
{
  // The main button and rotary A and B have pull-ups on the board. An internal pull-up as well
  // would hold rotary A and B above 0 V through their 10k series resistors, near the logic threshold.
  gpio_config_t in = {};
  in.pin_bit_mask = (1ULL << BUTTON_PIN) | (1ULL << ROTARY_PIN1) | (1ULL << ROTARY_PIN2);
  in.mode = GPIO_MODE_INPUT;
  ESP_ERROR_CHECK(gpio_config(&in));

  gpio_config_t rotarySwitch = {};
  rotarySwitch.pin_bit_mask = 1ULL << ROTARY_BUTTON_PIN;
  rotarySwitch.mode = GPIO_MODE_INPUT;
  rotarySwitch.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&rotarySwitch));

  // Still held from turning the board on: that press isn't for the controller
  if (gpio_get_level(BUTTON_PIN) == 0) {
    autoButton.pressed = true;
    autoButton.ignoreRelease = true;
  }

  setButtonLight(BUTTON_LIGHT_OFF);   // before they become outputs, so the light doesn't flash
  gpio_config_t out = {};
  out.pin_bit_mask = (1ULL << BUTTON_LIGHT_R_PIN) | (1ULL << BUTTON_LIGHT_G_PIN) | (1ULL << BUTTON_LIGHT_B_PIN);
  out.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&out));
  // The board has no series resistors for this LED: keep the pins at their weakest drive
  ESP_ERROR_CHECK(gpio_set_drive_capability(BUTTON_LIGHT_R_PIN, GPIO_DRIVE_CAP_0));
  ESP_ERROR_CHECK(gpio_set_drive_capability(BUTTON_LIGHT_G_PIN, GPIO_DRIVE_CAP_0));
  ESP_ERROR_CHECK(gpio_set_drive_capability(BUTTON_LIGHT_B_PIN, GPIO_DRIVE_CAP_0));

  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &periodic_timer_callback,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "inputs",
      .skip_unhandled_events = true,
  };
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));   // 1 ms
  ESP_LOGI(TAG, "Input sampling started");
}
