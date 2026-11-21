// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "display_task.h"
#include "link_task.h"
#include "board_pins.h"
#include "power.h"
#include "inputs.h"
#include "motor_calibration.h"
#include "firmware_update.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "fonts/roboto_light_16.h"
#include "fonts/roboto_light_24.h"
#include "fonts/roboto_light_40.h"
#include "fonts/roboto_light_16_Bold.h"
#include "fonts/roboto_light_24_Bold.h"
#include "fonts/roboto_light_40_Bold.h"

static const char *TAG = "display";

// GC9A01 240x240, on the second SPI bus
class LGFX_GC9A01 : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;

public:
  LGFX_GC9A01()
  {
    auto bus = _bus.config();
    bus.spi_host = SPI2_HOST;
    bus.spi_mode = 0;
    bus.freq_write = 27000000;
    bus.pin_sclk = LCD_CLK_PIN;
    bus.pin_mosi = LCD_MOSI_PIN;
    bus.pin_miso = -1;
    bus.pin_dc = LCD_DC_PIN;
    bus.dma_channel = SPI_DMA_CH_AUTO;
    _bus.config(bus);
    _panel.setBus(&_bus);

    auto panel = _panel.config();
    panel.pin_cs = LCD_CS_PIN;
    panel.pin_rst = LCD_RST_PIN;
    panel.pin_busy = -1;
    panel.panel_width = 240;
    panel.panel_height = 240;
    panel.invert = true;
    panel.readable = false;
    _panel.config(panel);
    setPanel(&_panel);
  }
};

static LGFX_GC9A01 tft;
static LGFX_Sprite img(&tft);

// The display redraws from a snapshot of these, and only when it differs from what is on screen
#define REDRAW_MIN_INTERVAL_MS 100   // at most 10 redraws a second, however fast values change
static uint8_t displayed[OM_ELEMENT_COUNT];  // element values currently on screen
static bool displayedLinkUp = false;
static int8_t displayedRssi = 0;
static uint8_t displayedHandlePct = 0;        // the handle's own battery (this board's)
static bool displayedHandleBattery = false;   // false: nothing on J9
static bool displayedHandleCharging = false;
static uint8_t displayedOdriveError = OM_ODRIVE_ERROR_NONE;

static const char *strokes[] = {"Jack","Pulse","Pump","Classic","Tease","Crescendo","Slow In","Heartbeat","Breathing","Wander","Pendulum"};   // order of stroke_id_t in omachine-controller

static const char *machinestates[OM_STATE_COUNT] = {"Initializing","Homing","Syncing","Manual","Auto","Starting","Link lost","Setting up","ODrive fault"};   // indexed by OM_STATE_*

// What an ODrive error bit (ODriveError, firmware 0.6) means, for the fault screen
static const char *odriveErrorText(uint8_t bit)
{
  switch (bit) {
    case 0:  return "Still starting";
    case 1:  return "System error";
    case 2:  return "Timing error";
    case 3:  return "No position estimate";
    case 4:  return "Bad configuration";
    case 5:  return "Gate driver fault";
    case 6:  return "Missing input";
    case 8:  return "Supply voltage high";
    case 9:  return "Supply voltage low";
    case 10: return "Supply current high";
    case 11: return "Regen current high";
    case 12: return "Current limit";
    case 13: return "Motor too hot";
    case 14: return "ODrive too hot";
    case 15: return "Too fast";
    case 16: return "Past position limit";
    case 17: return "Current asked too high";
    case 24: return "Watchdog";
    case 25: return "E-stop";
    case 26: return "Spinout";
    case 27: return "Brake resistor";
    case 28: return "Thermistor unplugged";
    case 30: return "Calibration error";
    case OM_ODRIVE_ERROR_NONE: return "Not running";
    default: return "Unknown error";
  }
}

volatile DisplayScreen displayScreen = SCREEN_OPERATING;

#define CHARGING_REDRAW_MS 1000   // the charging screen changes slowly

static uint32_t millis() { return (uint32_t)(esp_timer_get_time() / 1000); }

// TFT_eSPI's alphaBlend: alpha 255 gives fgc, 0 gives bgc (RGB565)
static uint16_t alphaBlend(uint8_t alpha, uint16_t fgc, uint16_t bgc)
{
  uint32_t rxb = bgc & 0xF81F;
  rxb += ((fgc & 0xF81F) - rxb) * (alpha >> 2) >> 6;
  uint32_t xgx = bgc & 0x07E0;
  xgx += ((fgc & 0x07E0) - xgx) * alpha >> 8;
  return (rxb & 0xF81F) | (xgx & 0x07E0);
}

// Like TFT_eSPI's drawSmoothArc with round ends: angles 0 at 6 o'clock, clockwise. LovyanGFX has
// no anti-aliased arc, so the band is a plain fillArc with anti-aliased round caps.
static void drawRoundArc(int32_t x, int32_t y, int32_t r, int32_t ir, int32_t start_angle, int32_t end_angle, uint16_t color)
{
  img.fillArc(x, y, r, ir, start_angle + 90, end_angle + 90, color);   // LovyanGFX: 0 at 3 o'clock
  float mid = (r + ir) / 2.0f;
  int32_t cap = (r - ir) / 2;
  int32_t angles[2] = {start_angle, end_angle};
  for (int32_t a : angles) {
    float rad = (a + 90) * (float)M_PI / 180.0f;
    img.fillSmoothCircle(x + lroundf(mid * cosf(rad)), y + lroundf(mid * sinf(rad)), cap, color);
  }
}

// Battery fill colour: red -> yellow -> green as the charge goes up
static uint16_t batteryColor(uint8_t pct)
{
  return (pct >= 50) ? alphaBlend((pct - 50) * 255 / 50, TFT_GREEN, TFT_YELLOW)
                     : alphaBlend(pct * 255 / 50, TFT_YELLOW, TFT_RED);
}

// Small battery symbol with its percent beside it, for the operating screen's top and bottom rows.
// Returns the width drawn; smallBatteryWidth() gives it beforehand. The font must be set.
#define SMALL_BATTERY_W 20
#define SMALL_BATTERY_H 10
#define SMALL_BATTERY_TEXT_GAP 5   // nub (2) and space before the text
static int16_t smallBatteryWidth(const char *label)
{
  return SMALL_BATTERY_W + SMALL_BATTERY_TEXT_GAP + img.textWidth(label);
}

static int16_t drawSmallBattery(int16_t bx, int16_t cy, uint8_t pct, bool charging, const char *label)
{
  const int16_t bw = SMALL_BATTERY_W, bh = SMALL_BATTERY_H;
  int16_t by = cy - bh / 2;
  uint16_t outline = (pct >= 20) ? (uint16_t)TFT_LIGHTGREY : (uint16_t)TFT_RED;
  img.drawRoundRect(bx, by, bw, bh, 2, outline);
  img.fillRect(bx + bw, cy - 2, 2, 4, outline);   // terminal nub
  int16_t fill_w = (int16_t)((bw - 4) * pct / 100);
  if (fill_w > 0) {
    img.fillRect(bx + 2, by + 2, fill_w, bh - 4, batteryColor(pct));
  }
  if (charging) {
    // lightning bolt over the fill
    int16_t mx = bx + bw / 2;
    img.fillTriangle(mx + 2, by + 1, mx - 3, cy + 1, mx, cy + 1, TFT_WHITE);
    img.fillTriangle(mx - 2, by + bh - 2, mx + 3, cy - 1, mx, cy - 1, TFT_WHITE);
  }
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(ML_DATUM);
  img.drawString(label, bx + bw + SMALL_BATTERY_TEXT_GAP, cy);
  return smallBatteryWidth(label);
}

// The board's own battery and charge status, while it is "off" on USB (or homing after the button)
static void drawChargingScreen(bool starting)
{
  ChargeState state = power_charge_state();
  uint8_t pct = power_battery_percent();
  bool battery = state != CHARGE_NO_BATTERY;
  const int16_t cx = tft.width() / 2;

  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);

  const char *title = starting ? "Starting"
                    : state == CHARGE_CHARGING ? "Charging"
                    : state == CHARGE_DONE ? "Charged"
                    : state == CHARGE_NO_BATTERY ? "No battery"
                    : "On battery";   // USB went while this was drawn; the board is about to go off
  img.setFont(&Roboto_Light_40);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(TC_DATUM);
  img.drawString(title, cx, 50);

  // large battery symbol, filled to the charge
  const int16_t bw = 100, bh = 46, nub = 8;
  const int16_t bx = cx - (bw + nub) / 2, by = 112;
  uint16_t outline = (battery && pct < 20) ? (uint16_t)TFT_RED : (uint16_t)TFT_LIGHTGREY;
  img.drawRoundRect(bx, by, bw, bh, 6, outline);
  img.drawRoundRect(bx + 1, by + 1, bw - 2, bh - 2, 5, outline);
  img.fillRoundRect(bx + bw, by + bh / 2 - 9, nub, 18, 2, outline);
  if (battery) {
    int16_t fill_w = (int16_t)((bw - 8) * pct / 100);
    if (fill_w > 0) {
      img.fillRoundRect(bx + 4, by + 4, fill_w, bh - 8, 3, batteryColor(pct));
    }
    char label[8];
    snprintf(label, sizeof(label), "%u%%", pct);
    img.setFont(&Roboto_Light_24);
    img.setTextDatum(TC_DATUM);
    img.drawString(label, cx, 170);
  }

  if (!starting) {
    img.setFont(&Roboto_Light_16);
    img.setTextColor(TFT_DARKGREY, TFT_BLACK);
    img.setTextDatum(TC_DATUM);
    img.drawString("Press to start", cx, 205);
  }

  img.pushSprite(0, 0);
}

// While the button is held to turn on (from the battery) or off: a bar, styled like the operating
// screen's power bar, shrinks to nothing when the board turns on or off. 'left' is 1 to 0.
#define HOLD_REDRAW_MS 30
#define HOLD_BAR_W 160
#define POWERING_ON_COLOR 0x04BF    // light blue (RGB565), like the button once it latches
#define POWERING_OFF_COLOR 0xFA80   // orange
static void drawHoldScreen(float left, const char *title, const char *subtitle, uint16_t color)
{
  const int16_t cx = tft.width() / 2, cy = tft.height() / 2;
  if (left < 0) left = 0;
  if (left > 1) left = 1;

  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(BC_DATUM);
  img.drawString(title, cx, cy - 22);

  if (subtitle) {
    // grey track, and the bar over it shrinking towards the centre
    img.fillRoundRect(cx - HOLD_BAR_W / 2, cy - 3, HOLD_BAR_W, 6, 3, TFT_DARKGREY);
    int16_t w = (int16_t)lroundf(HOLD_BAR_W * left);
    if (w >= 12) {
      img.fillRoundRect(cx - w / 2, cy - 6, w, 12, 6, color);
    } else if (w > 0) {
      img.fillSmoothCircle(cx, cy, w / 2 + 1, color);
    }

    img.setFont(&Roboto_Light_16);
    img.setTextColor(TFT_DARKGREY, TFT_BLACK);
    img.setTextDatum(TC_DATUM);
    img.drawString(subtitle, cx, cy + 22);
  }

  img.pushSprite(0, 0);
}

// Counting from boot, since the hold to turn on starts before the chip does
static void drawPoweringOnScreen()
{
  float left = 1.0f - esp_timer_get_time() / (POWER_ON_HOLD_MS * 1000.0f);
  drawHoldScreen(left, "Keep holding", "to turn on", POWERING_ON_COLOR);
}

static void drawPoweringOffScreen(bool turningOff)
{
  if (turningOff) {
    drawHoldScreen(0, "Turning off", nullptr, POWERING_OFF_COLOR);   // goes once the button is let go
    return;
  }
  float left = (float)(POWER_OFF_HOLD_MS - autoButton.heldMs) / (POWER_OFF_HOLD_MS - POWER_OFF_COUNTDOWN_MS);
  drawHoldScreen(left, "Keep holding", "to turn off", POWERING_OFF_COLOR);
}

// While the rotary button is held on the way to opening the menu
#define MENU_HOLD_COLOR 0x04BF   // light blue, like the selected menu item
static void drawMenuCountdownScreen()
{
  float left = (float)(MENU_HOLD_MS - rotaryButton.heldMs) / (MENU_HOLD_MS - MENU_COUNTDOWN_MS);
  drawHoldScreen(left, "Keep holding", "for the menu", MENU_HOLD_COLOR);
}

// The menu: turn the knob to choose, press it to select. Closes by itself after MENU_TIMEOUT_MS
// without input.
#define MENU_TIMEOUT_MS 20000
#define MENU_RESULT_MS  3000   // how long a result stays up (a press closes it sooner)
#define MENU_SAVE_TIMEOUT_MS 2000   // no answer from the controller by then: the save failed
#define MENU_UPDATE_RESULT_MS 10000   // update and Wi-Fi results stay up longer: there's more to read
#define MENU_WIFI_SETUP_TIMEOUT_MS 300000   // the setup network closes by itself after 5 minutes
typedef enum { MENU_ITEM_POWER, MENU_ITEM_SAVE, MENU_ITEM_CALIBRATE, MENU_ITEM_ODRIVE_SETUP, MENU_ITEM_WIFI,
               MENU_ITEM_UPDATE, MENU_ITEM_EXIT, MENU_ITEM_COUNT } MenuItem;
static const char *menuItems[MENU_ITEM_COUNT] = {"Power", "Save settings", "Calibrate motor", "Set up ODrive",
                                                 "Wi-Fi setup", "Update firmware", "Exit"};
#define MENU_ROWS 5   // items on screen at once; the list scrolls to keep the selection in view
typedef enum { MENU_PAGE_LIST, MENU_PAGE_POWER, MENU_PAGE_SAVING, MENU_PAGE_CALIBRATING, MENU_PAGE_WIFI_SETUP,
               MENU_PAGE_UPDATING, MENU_PAGE_SETUP_CONFIRM, MENU_PAGE_SETUP_RUNNING, MENU_PAGE_SETUP_STOP,
               MENU_PAGE_RESULT } MenuPage;
#define SETUP_ACK_TIMEOUT_MS 3000   // "Set up ODrive" not answered by then: not started
#define SETUP_RUNNING_STAGES (OM_SETUP_SAVING - OM_SETUP_CHECKING + 1)
static const char *setupStageNames[] = {"", "Checking", "Configuring", "Calibrating", "Anticogging", "Saving"};   // OM_SETUP_* running stages

// What the result page shows (after a save or a calibration)
typedef struct {
  const char *title;
  uint16_t color;
  const char *line;
  const char *hint;
} MenuResult;

static void drawMenuScreen(int selected)
{
  const int16_t cx = tft.width() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_DARKGREY, TFT_BLACK);
  img.setTextDatum(MC_DATUM);
  img.drawString("Menu", cx, 35);
  // keep one item above the selection in view where there is one
  int first = selected - 1;
  if (first > MENU_ITEM_COUNT - MENU_ROWS) first = MENU_ITEM_COUNT - MENU_ROWS;
  if (first < 0) first = 0;
  for (int i = first; i < first + MENU_ROWS && i < MENU_ITEM_COUNT; i++) {
    if (i == selected) {
      img.setFont(&Roboto_Light_24_Bold);
      img.setTextColor(TFT_BLUE, TFT_BLACK);
    } else {
      img.setFont(&Roboto_Light_24);
      img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    }
    img.drawString(menuItems[i], cx, 78 + (i - first) * 28);
  }
  // small arrows where the list goes on
  if (first > 0) img.fillTriangle(cx - 6, 62, cx + 6, 62, cx, 56, TFT_DARKGREY);
  if (first + MENU_ROWS < MENU_ITEM_COUNT) img.fillTriangle(cx - 6, 206, cx + 6, 206, cx, 212, TFT_DARKGREY);
  img.pushSprite(0, 0);
}

// The Power page: the knob turns powerCount, which the controller applies to its Power setting.
// The value shown is the controller's, so it only moves while the link is up.
static void drawPowerScreen(bool link, uint8_t power)
{
  const int16_t cx = tft.width() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_DARKGREY, TFT_BLACK);
  img.setTextDatum(MC_DATUM);
  img.drawString("Power", cx, 55);

  char text[8];
  if (link) snprintf(text, sizeof(text), "%u", power);
  else snprintf(text, sizeof(text), "--");
  img.setFont(&Roboto_Light_40_Bold);
  img.setTextColor(TFT_BLUE, TFT_BLACK);
  img.drawString(text, cx, 100);

  // the bar the operating screen used to show, twice as wide
  uint8_t pct = link ? (power > 100 ? 100 : power) : 0;
  uint16_t color = alphaBlend(pct * (255.0 / 100), TFT_RED, TFT_YELLOW);
  img.fillRoundRect(cx - 100 - 3, 136 + 3, 200 + 6, 6, 3, TFT_DARKGREY);
  if (link) {
    int16_t w = pct * 2 + 6;
    img.fillRoundRect(cx - w / 2, 136, w, 12, 6, color);
  }

  img.setFont(&Roboto_Light_16);
  img.setTextColor(TFT_DARKGREY, TFT_BLACK);
  img.setTextDatum(TC_DATUM);
  img.drawString(link ? "Turn, then press" : "No link", cx, 172);
  img.pushSprite(0, 0);
}

// A title, a line below it and an optional dimmer one, like the homing failed screen
static void drawMessageScreen(const char *title, uint16_t titleColor, const char *line, const char *hint)
{
  const int16_t cx = tft.width() / 2, cy = tft.height() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(titleColor, TFT_BLACK);
  img.setTextDatum(BC_DATUM);
  img.drawString(title, cx, cy - 8);
  img.setFont(&Roboto_Light_16);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(TC_DATUM);
  img.drawString(line, cx, cy + 8);
  if (hint) {
    img.setTextColor(TFT_DARKGREY, TFT_BLACK);
    img.drawString(hint, cx, cy + 34);
  }
  img.pushSprite(0, 0);
}

// A question with two answers side by side: turn to choose, press to confirm
static void drawChoiceScreen(const char *title, const char *line, const char *hint, const char *left,
                             const char *right, int chosen)
{
  const int16_t cx = tft.width() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(MC_DATUM);
  img.drawString(title, cx, 62);
  img.setFont(&Roboto_Light_16);
  img.drawString(line, cx, 100);
  if (hint) {
    img.setTextColor(TFT_DARKGREY, TFT_BLACK);
    img.drawString(hint, cx, 122);
  }
  const char *answers[2] = {left, right};
  for (int i = 0; i < 2; i++) {
    if (i == chosen) {
      img.setFont(&Roboto_Light_24_Bold);
      img.setTextColor(TFT_BLUE, TFT_BLACK);
    } else {
      img.setFont(&Roboto_Light_24);
      img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    }
    img.drawString(answers[i], i == 0 ? cx - 50 : cx + 50, 170);
  }
  img.pushSprite(0, 0);
}

// "Set up ODrive" under way: the stage, a bar through it and the time so far
static void drawSetupRunningScreen(uint8_t stage, uint8_t progress, uint32_t elapsedS, bool link, bool started)
{
  const int16_t cx = tft.width() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(MC_DATUM);
  bool running = started && stage >= OM_SETUP_CHECKING && stage <= OM_SETUP_SAVING;
  img.drawString(running ? setupStageNames[stage] : "Starting", cx, 66);

  char text[24];
  img.setFont(&Roboto_Light_16);
  img.setTextColor(TFT_DARKGREY, TFT_BLACK);
  if (running) {
    snprintf(text, sizeof(text), "Step %u of %d", (unsigned)(stage - OM_SETUP_CHECKING + 1), SETUP_RUNNING_STAGES);
    img.drawString(text, cx, 94);
    int pct = progress > 100 ? 100 : progress;
    img.fillRoundRect(cx - 80, 116, 160, 8, 4, TFT_DARKGREY);
    img.fillRoundRect(cx - 80, 116, 16 + pct * 144 / 100, 8, 4, 0x04BF);
  }
  snprintf(text, sizeof(text), "%u:%02u", (unsigned)(elapsedS / 60), (unsigned)(elapsedS % 60));
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.drawString(text, cx, 146);
  img.setTextColor(link ? TFT_DARKGREY : TFT_ORANGE, TFT_BLACK);
  img.drawString(link ? "Press to stop" : "No link", cx, 178);
  img.pushSprite(0, 0);
}

// The result page for a "Set up ODrive" result stage
static MenuResult setupResult(uint8_t stage)
{
  switch (stage) {
    case OM_SETUP_DONE:             return {"ODrive set up", TFT_GREEN, "Settings saved", "The controller restarts"};
    case OM_SETUP_CANCELLED:        return {"Setup stopped", TFT_ORANGE, "Old settings kept", nullptr};
    case OM_SETUP_FAILED_NO_ODRIVE: return {"Setup failed", TFT_RED, "No answer from the ODrive", "Check the CAN cable"};
    case OM_SETUP_FAILED_VERSION:   return {"Setup failed", TFT_RED, "Unexpected ODrive firmware", "Old settings kept"};
    case OM_SETUP_FAILED_CONFIG:    return {"Setup failed", TFT_RED, "A setting didn't write", "Old settings kept"};
    case OM_SETUP_FAILED_CALIBRATE: return {"Setup failed", TFT_RED, "Motor calibration failed", "Old settings kept"};
    case OM_SETUP_FAILED_STALL:     return {"Motor stalled", TFT_RED, "Is it off the rail?", "Old settings kept"};
    case OM_SETUP_FAILED_ANTICOG:   return {"Setup failed", TFT_RED, "Anticogging failed", "Old settings kept"};
    case OM_SETUP_FAILED_SAVE:      return {"Save unconfirmed", TFT_ORANGE, "The ODrive didn't confirm", "Run the setup again?"};
    default:                        return {"Setup ended", TFT_ORANGE, "Result not received", nullptr};
  }
}

// A firmware update under way: what it's doing, with a progress bar while there is one
static void drawUpdatingScreen()
{
  const int16_t cx = tft.width() / 2, cy = tft.height() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(BC_DATUM);
  img.drawString("Updating", cx, cy - 20);
  img.setFont(&Roboto_Light_16);
  img.setTextDatum(TC_DATUM);
  img.drawString(updateStep(), cx, cy - 6);
  int pct = updateProgress;
  if (pct >= 0) {
    if (pct > 100) pct = 100;
    img.fillRoundRect(cx - 80, cy + 22, 160, 8, 4, TFT_DARKGREY);
    img.fillRoundRect(cx - 80, cy + 22, 16 + pct * 144 / 100, 8, 4, 0x04BF);
  }
  img.setTextColor(TFT_DARKGREY, TFT_BLACK);
  img.drawString("Leave it switched on", cx, cy + 44);
  img.pushSprite(0, 0);
}

// From motor_task.h (not included: its SimpleFOC headers clash with millis() here)
extern volatile bool homingStalled;
extern volatile uint8_t homingTravelPct;

// The handle couldn't find its end stops, so the motor is off and nothing else runs
static void drawHomingFailedScreen()
{
  const int16_t cx = tft.width() / 2, cy = tft.height() / 2;
  img.fillSprite(TFT_BLACK);
  img.setTextSize(1);
  img.setFont(&Roboto_Light_24);
  img.setTextColor(TFT_RED, TFT_BLACK);
  img.setTextDatum(BC_DATUM);
  img.drawString(homingStalled ? "Handle stuck" : "No end stops", cx, cy - 8);
  img.setFont(&Roboto_Light_16);
  img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  img.setTextDatum(TC_DATUM);
  img.drawString(homingStalled ? (power_motor_limited() ? "Plug in the battery?" : "Is the rail free?")
                               : "Is the handle on the rail?", cx, cy + 8);
  img.setTextColor(TFT_DARKGREY, TFT_BLACK);
  img.drawString("Hold the button to turn off", cx, cy + 34);
  img.pushSprite(0, 0);
}

// Operating screen rows, top to bottom (y in pixels)
#define TOP_ROW_Y     22    // centre of the handle's battery and the signal bars
#define STATUS_Y      74    // machine state
#define STROKE_Y      145   // stroke name (centre)
#define LABEL_ROW_Y   172   // Speed and Length; their values below
#define BOTTOM_BATTERY_Y 216   // centre of the controller's battery

static void arcloop()
{
  // Draw from the snapshot displayTask took ('displayed'), not the live elements[] that linkTask
  // keeps updating.
  const uint8_t *speed = &displayed[OM_ELEMENT_SPEED];
  const uint8_t *strokeLength = &displayed[OM_ELEMENT_LENGTH];
  const uint8_t *strokeIndex = &displayed[OM_ELEMENT_STROKE_INDEX];
  const uint8_t *activeElementIndex = &displayed[OM_ELEMENT_ACTIVE_INDEX];
  const uint8_t *machineStateIndex = &displayed[OM_ELEMENT_STATE_INDEX];
  const uint8_t *batteryPercent = &displayed[OM_ELEMENT_BATTERY];

  char text[16];
  uint16_t x = tft.width()/2;   // Position of centre of arc
  uint16_t y = tft.height()/2;
  uint16_t speed_fg_color = alphaBlend(speed[0]*(255.0/100), TFT_RED, TFT_YELLOW);
  uint16_t length_fg_color = alphaBlend(strokeLength[0]*(255.0/100), TFT_RED, TFT_YELLOW);
  img.fillSprite(TFT_BLACK);   // clear last frame

  uint8_t radius       = x-1;                    // Outer arc radius
  uint8_t thickness    = 10;                     // Thickness
  uint8_t inner_radius = radius - thickness;     // Calculate inner radius

  // 0 degrees is at 6 o'clock position
  // Arcs are drawn clockwise from start_angle to end_angle
  uint16_t start_angle = 50;
  uint16_t end_angle   = 130;
  uint16_t speed_angle = ((end_angle-start_angle)*(int(speed[0])/100.0))+start_angle+1;

  // With no link the elements are only defaults or stale, so the controller's values (the arcs'
  // fill, the stroke, speed, length and the controller's battery) aren't drawn; the grey tracks stay.
  // An ODrive fault shows its cause and how to retry in their place.
  const bool fault = displayedLinkUp && machineStateIndex[0] == OM_STATE_ODRIVE_FAULT;
  const bool valid = displayedLinkUp && !fault;

  drawRoundArc(x, y, radius-3, inner_radius+4, start_angle, end_angle, TFT_DARKGREY);
  if (valid) drawRoundArc(x, y, radius, inner_radius, start_angle, speed_angle, speed_fg_color);

  uint16_t length_start_angle = 230;
  uint16_t length_end_angle   = 310;
  uint16_t length_angle = length_end_angle-((length_end_angle-length_start_angle)*(int(strokeLength[0])/100.0));

  drawRoundArc(x, y, radius-3, inner_radius+4, length_start_angle, length_end_angle, TFT_DARKGREY);
  if (valid) drawRoundArc(x, y, radius, inner_radius, length_angle, length_end_angle+1, length_fg_color);

  img.setTextSize(1);

//STROKE

  if (valid) {
  if (activeElementIndex[0] == 3) {
    img.setFont(&Roboto_Light_24_Bold);
    img.setTextColor(TFT_BLUE,TFT_BLACK);
  } else
  {
    img.setFont(&Roboto_Light_24);
    img.setTextColor(TFT_LIGHTGREY,TFT_BLACK);
  }

  img.setTextDatum(MC_DATUM);
  const uint8_t numStrokes = sizeof(strokes) / sizeof(strokes[0]);
  img.drawString(strokeIndex[0] < numStrokes ? strokes[strokeIndex[0]] : "?", tft.width() / 2, STROKE_Y);
  }

  if (fault) {
    img.setFont(&Roboto_Light_24);
    img.setTextColor(TFT_ORANGE, TFT_BLACK);
    img.setTextDatum(MC_DATUM);
    img.drawString(odriveErrorText(displayedOdriveError), tft.width() / 2, STROKE_Y);
    img.setFont(&Roboto_Light_16);
    img.setTextColor(TFT_DARKGREY, TFT_BLACK);
    img.setTextDatum(TC_DATUM);
    img.drawString("Press to retry", tft.width() / 2, LABEL_ROW_Y + 9);
  }

//TOP ROW: the handle's own battery (with a bolt while it charges from USB), then the link's signal bars

  {
    const int16_t cy = TOP_ROW_Y, gap = 12;
    const int8_t thresholds[] = {-90, -80, -70, -60};   // dBm needed for each bar (weakest RSSI of the last second)
    const int16_t barw = 3, bargap = 2, maxh = 13, bars_w = 4 * barw + 3 * bargap;

    char handleLabel[8];
    if (displayedHandleBattery) snprintf(handleLabel, sizeof(handleLabel), "%u", displayedHandlePct);
    else snprintf(handleLabel, sizeof(handleLabel), "--");
    img.setFont(&Roboto_Light_16);
    int16_t bx = tft.width() / 2 - (smallBatteryWidth(handleLabel) + gap + bars_w) / 2;
    bx += drawSmallBattery(bx, cy, displayedHandleBattery ? displayedHandlePct : 0, displayedHandleCharging, handleLabel) + gap;

    uint8_t bars = 0;
    if (displayedLinkUp) {
      while (bars < 4 && displayedRssi >= thresholds[bars]) {
        bars++;
      }
    }
    uint16_t on_color = (bars <= 1) ? TFT_RED : TFT_LIGHTGREY;
    const int16_t bottom = cy + 6;
    for (uint8_t i = 0; i < 4; i++) {
      int16_t h = 4 + i * (maxh - 4) / 3;
      img.fillRect(bx + i * (barw + bargap), bottom - h, barw, h, i < bars ? on_color : (uint16_t)TFT_DARKGREY);
    }
  }

//BOTTOM: the controller's battery (OM_BATTERY_UNKNOWN until the ODrive has reported it)

  if (valid && batteryPercent[0] != OM_BATTERY_UNKNOWN) {
    char label[8];
    uint8_t pct = batteryPercent[0] > 100 ? 100 : batteryPercent[0];
    snprintf(label, sizeof(label), "%u", pct);
    img.setFont(&Roboto_Light_16);
    drawSmallBattery(tft.width() / 2 - smallBatteryWidth(label) / 2, BOTTOM_BATTERY_Y, pct, false, label);
  }

// STATUS
  img.setFont(&Roboto_Light_40);
  img.setTextColor(TFT_LIGHTGREY,TFT_BLACK);
  img.setTextDatum(TC_DATUM);
  const uint8_t numStates = sizeof(machinestates) / sizeof(machinestates[0]);
  img.drawString(!displayedLinkUp ? "No link" : machineStateIndex[0] < numStates ? machinestates[machineStateIndex[0]] : "?", tft.width() / 2, STATUS_Y);

  if (valid) {
//SPEED
  if (activeElementIndex[0] == 1) {
    img.setFont(&Roboto_Light_16_Bold);
    img.setTextColor(TFT_BLUE,TFT_BLACK);
  } else
  {
    img.setFont(&Roboto_Light_16);
    img.setTextColor(TFT_LIGHTGREY,TFT_BLACK);
  }

  img.setTextDatum(TC_DATUM);
  img.drawString("Speed", 67, LABEL_ROW_Y);
  snprintf(text, sizeof(text), "%d", speed[0]);
  img.drawString(text, 67, LABEL_ROW_Y + 18);

//LENGTH
  if (activeElementIndex[0] == 2) {
    img.setFont(&Roboto_Light_16_Bold);
    img.setTextColor(TFT_BLUE,TFT_BLACK);
  } else
  {
    img.setFont(&Roboto_Light_16);
    img.setTextColor(TFT_LIGHTGREY,TFT_BLACK);
  }
  img.setTextDatum(TC_DATUM);
  img.drawString("Length", 175, LABEL_ROW_Y);
  snprintf(text, sizeof(text), "%d", strokeLength[0]);
  img.drawString(text, 175, LABEL_ROW_Y + 18);
  }

  img.pushSprite(0,0);
}

void displayTask(void *parameters)
{
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  // backlight on only now the screen is cleared, so its power-up contents don't show
  gpio_set_level(LCD_BL_PIN, 1);
  gpio_config_t backlight = {};
  backlight.pin_bit_mask = 1ULL << LCD_BL_PIN;
  backlight.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&backlight));

  // allocate the full-screen sprite once and reuse it for every refresh
  img.setColorDepth(8);
  if (img.createSprite(tft.width(), tft.height()) == nullptr) {
    ESP_LOGE(TAG, "Sprite allocation failed");
    vTaskDelete(NULL);
  }

  bool firstDraw = true;
  uint32_t lastDrawMs = 0;
  DisplayScreen shownScreen = SCREEN_OPERATING;
  bool offCountdownShown = false;
  bool turningOff = false;   // the countdown ran out: "Turning off" stays until the power goes
  bool menuCountdownShown = false;
  bool menuShown = false;     // menuOpen has been seen and the menu set up
  bool menuRedraw = false;
  MenuPage menuPage = MENU_PAGE_LIST;
  int menuSelected = 0;
  uint8_t lastMenuRotary = 0, lastMenuPresses = 0;
  uint32_t menuInputMs = 0, menuResultMs = 0;
  MenuResult menuResult = {};
  uint32_t menuUpdateShown = 0;   // updateStepCount and updateProgress last drawn
  int menuUpdatePctShown = -2;
  uint32_t menuUpdateDrawMs = 0;
  uint32_t menuResultForMs = MENU_RESULT_MS;
  char menuWifiSsid[33] = {};
  uint8_t menuSavedCountAtRequest = 0;
  bool menuPowerLink = false;
  uint8_t menuPowerShown = 0;
  int setupChoice = 0;              // confirm and stop pages: 0 the left answer, 1 the right
  bool setupAcked = false;          // the controller has taken the request (or one was running)
  bool setupSeenRunning = false;    // the controller has been seen in OM_STATE_ODRIVE_SETUP
  uint32_t setupStartMs = 0;
  uint8_t setupStageShown = 0xff, setupProgressShown = 0xff;
  uint32_t setupElapsedShown = 0xffffffff;
  bool setupLinkShown = false;
  for (;;) {
    DisplayScreen screen = displayScreen;
    // The off countdown covers whichever screen is up, until the button is let go
    if (powerOffCountdown || turningOff) {
      if (powerOffRequested || autoButton.heldMs >= POWER_OFF_HOLD_MS) {
        turningOff = true;
      }
      if (!offCountdownShown || millis() - lastDrawMs >= HOLD_REDRAW_MS) {
        drawPoweringOffScreen(turningOff);
        lastDrawMs = millis();
      }
      offCountdownShown = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (offCountdownShown) {
      offCountdownShown = false;   // let go before the end: back to what was there
      lastDrawMs = 0;
      firstDraw = true;
      menuRedraw = true;
    }

    // Holding the rotary button: the countdown to the menu
    if (menuCountdown) {
      if (!menuCountdownShown || millis() - lastDrawMs >= HOLD_REDRAW_MS) {
        drawMenuCountdownScreen();
        lastDrawMs = millis();
      }
      menuCountdownShown = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (menuCountdownShown) {
      menuCountdownShown = false;   // opened, or let go too soon: either way redraw
      lastDrawMs = 0;
      firstDraw = true;
    }

    if (menuOpen) {
      uint32_t now = millis();
      if (!menuShown) {
        menuShown = true;
        menuPage = MENU_PAGE_LIST;
        menuSelected = 0;
        menuResultForMs = MENU_RESULT_MS;
        lastMenuRotary = menuRotaryCount;
        lastMenuPresses = rotaryButton.menuPresses;
        menuInputMs = now;
        menuRedraw = true;
      }
      int8_t turn = (int8_t)(uint8_t)(menuRotaryCount - lastMenuRotary);
      lastMenuRotary += turn;
      uint8_t presses = rotaryButton.menuPresses - lastMenuPresses;
      lastMenuPresses += presses;
      bool close = false;

      if (menuPage == MENU_PAGE_LIST) {
        if (turn != 0) {
          int sel = menuSelected - turn;   // turning down (clockwise) moves down the list
          menuSelected = sel < 0 ? 0 : sel >= MENU_ITEM_COUNT ? MENU_ITEM_COUNT - 1 : sel;
          menuInputMs = now;
          menuRedraw = true;
        }
        if (presses > 0) {
          menuInputMs = now;
          if (menuSelected == MENU_ITEM_POWER) {
            menuPage = MENU_PAGE_POWER;
            menuPowerLink = linkUp;
            menuPowerShown = elements[OM_ELEMENT_POWER];
            menuRedraw = true;
          } else if (menuSelected == MENU_ITEM_SAVE) {
            if (linkUp) {
              menuSavedCountAtRequest = controllerSavedCount;
              saveCount = saveCount + 1;
              menuPage = MENU_PAGE_SAVING;
            } else {
              menuResult = {"Not saved", TFT_RED, "No link to the controller", nullptr};
              menuPage = MENU_PAGE_RESULT;
              menuResultMs = now;
            }
            menuRedraw = true;
          } else if (menuSelected == MENU_ITEM_WIFI) {
            wifiSetupStart();
            menuPage = MENU_PAGE_WIFI_SETUP;
            menuRedraw = true;
          } else if (menuSelected == MENU_ITEM_UPDATE) {
            firmwareUpdateStart();
            menuPage = MENU_PAGE_UPDATING;
            menuUpdatePctShown = -2;
            menuRedraw = true;
          } else if (menuSelected == MENU_ITEM_CALIBRATE) {
            motorCalState = MOTOR_CAL_REQUESTED;
            menuPage = MENU_PAGE_CALIBRATING;
            menuRedraw = true;
          } else if (menuSelected == MENU_ITEM_ODRIVE_SETUP) {
            if (linkUp && elements[OM_ELEMENT_STATE_INDEX] == OM_STATE_ODRIVE_SETUP) {
              // already running (the handle restarted since?): follow it
              setupAcked = true;
              setupSeenRunning = true;
              setupStartMs = now;
              menuPage = MENU_PAGE_SETUP_RUNNING;
            } else {
              setupChoice = 0;   // Cancel: the safe answer
              menuPage = MENU_PAGE_SETUP_CONFIRM;
            }
            menuRedraw = true;
          } else {
            close = true;
          }
        } else if (now - menuInputMs >= MENU_TIMEOUT_MS) {
          close = true;
        }
      } else if (menuPage == MENU_PAGE_POWER) {
        bool link = linkUp;
        if (turn != 0) {
          if (link) powerCount = powerCount + turn;   // with no link the clicks would land later, unseen
          menuInputMs = now;
        }
        uint8_t power = elements[OM_ELEMENT_POWER];
        if (link != menuPowerLink || power != menuPowerShown) {
          menuPowerLink = link;
          menuPowerShown = power;
          menuRedraw = true;
        }
        if (presses > 0 || now - menuInputMs >= MENU_TIMEOUT_MS) {
          close = true;
        }
      } else if (menuPage == MENU_PAGE_WIFI_SETUP) {
        if (wifiSetupState == WIFI_SETUP_SAVED) {
          if (!wifiStoredSsid(menuWifiSsid, sizeof(menuWifiSsid))) menuWifiSsid[0] = 0;
          vTaskDelay(pdMS_TO_TICKS(1000));   // let the phone get the "Saved" page before the network goes
          wifiSetupStop();
          menuResult = {"Wi-Fi saved", TFT_GREEN, menuWifiSsid, nullptr};
          menuPage = MENU_PAGE_RESULT;
          menuResultMs = millis();
          menuResultForMs = MENU_UPDATE_RESULT_MS;
          menuRedraw = true;
        } else if (presses > 0 || now - menuInputMs >= MENU_WIFI_SETUP_TIMEOUT_MS) {
          wifiSetupStop();
          close = true;
        }
      } else if (menuPage == MENU_PAGE_UPDATING) {
        UpdateState st = updateState;
        if (st == UPDATE_DONE || st == UPDATE_FAILED) {
          menuResult = {updateResultTitle(), st == UPDATE_DONE ? (uint16_t)TFT_GREEN : (uint16_t)TFT_RED,
                        updateResultLine(), nullptr};
          updateState = UPDATE_IDLE;
          menuPage = MENU_PAGE_RESULT;
          menuResultMs = now;
          menuResultForMs = MENU_UPDATE_RESULT_MS;
          menuRedraw = true;
        } else if (updateStepCount != menuUpdateShown ||
                   (updateProgress != menuUpdatePctShown && now - menuUpdateDrawMs >= 250)) {
          menuRedraw = true;   // presses do nothing: an update can't be stopped halfway
        }
      } else if (menuPage == MENU_PAGE_SAVING) {
        bool saved = controllerSavedCount != menuSavedCountAtRequest;
        if (saved || now - menuInputMs >= MENU_SAVE_TIMEOUT_MS) {
          menuResult = saved ? MenuResult{"Settings saved", TFT_GREEN, "Used at start-up", nullptr}
                             : MenuResult{"Not saved", TFT_RED, "No answer", "from the controller"};
          menuPage = MENU_PAGE_RESULT;
          menuResultMs = now;
          menuRedraw = true;
        }
      } else if (menuPage == MENU_PAGE_SETUP_CONFIRM || menuPage == MENU_PAGE_SETUP_STOP) {
        if (turn != 0) {
          int c = setupChoice - turn;   // clockwise: to the right
          setupChoice = c < 0 ? 0 : c > 1 ? 1 : c;
          menuInputMs = now;
          menuRedraw = true;
        }
        bool confirm = menuPage == MENU_PAGE_SETUP_CONFIRM;
        if (presses > 0 && setupChoice == 1) {
          if (confirm) {
            if (linkUp) {
              setupCount = setupCount + 1;
              setupAcked = false;
              setupSeenRunning = false;
              setupStartMs = now;
              menuPage = MENU_PAGE_SETUP_RUNNING;
            } else {
              menuResult = {"Not started", TFT_RED, "No link to the controller", nullptr};
              menuPage = MENU_PAGE_RESULT;
              menuResultMs = now;
            }
          } else {
            setupCancelCount = setupCancelCount + 1;   // the result page follows once it has stopped
            menuPage = MENU_PAGE_SETUP_RUNNING;
          }
          setupStageShown = 0xff;
          menuRedraw = true;
        } else if (presses > 0 || now - menuInputMs >= MENU_TIMEOUT_MS) {
          // the left answer, or no answer: don't start; keep the running setup going
          if (confirm) {
            close = true;
          } else {
            menuPage = MENU_PAGE_SETUP_RUNNING;
            setupStageShown = 0xff;
            menuRedraw = true;
          }
        }
      } else if (menuPage == MENU_PAGE_SETUP_RUNNING) {
        // No time-out: it stays up until the setup ends (the controller holds its result a few
        // seconds, then restarts)
        bool link = linkUp;
        uint8_t stage = controllerSetupStage;
        uint8_t state = elements[OM_ELEMENT_STATE_INDEX];
        if (!setupAcked && link && controllerSetupAck == setupCount && stage != OM_SETUP_IDLE) {
          setupAcked = true;
        }
        if (setupAcked && link && state == OM_STATE_ODRIVE_SETUP) {
          setupSeenRunning = true;
        }
        bool result = false;
        if (setupAcked && link && stage >= OM_SETUP_FIRST_RESULT) {
          menuResult = setupResult(stage);
          result = true;
        } else if (setupSeenRunning && link && state != OM_STATE_ODRIVE_SETUP) {
          menuResult = setupResult(OM_SETUP_IDLE);   // the controller restarted while the link was down
          result = true;
        } else if (!setupAcked && now - setupStartMs >= SETUP_ACK_TIMEOUT_MS) {
          menuResult = {"Not started", TFT_RED, "No answer", "from the controller"};
          result = true;
        }
        if (result) {
          menuPage = MENU_PAGE_RESULT;
          menuResultMs = now;
          menuResultForMs = MENU_UPDATE_RESULT_MS;
          menuRedraw = true;
        } else if (presses > 0 && setupAcked) {
          setupChoice = 0;   // Keep on: the safe answer
          menuInputMs = now;
          menuPage = MENU_PAGE_SETUP_STOP;
          menuRedraw = true;
        } else {
          uint32_t elapsed = (now - setupStartMs) / 1000;
          if (stage != setupStageShown || controllerSetupProgress != setupProgressShown ||
              elapsed != setupElapsedShown || link != setupLinkShown) {
            menuRedraw = true;
          }
        }
      } else if (menuPage == MENU_PAGE_CALIBRATING) {
        MotorCalState cal = motorCalState;
        if (cal == MOTOR_CAL_DONE || cal == MOTOR_CAL_FAILED) {
          menuResult = cal == MOTOR_CAL_DONE
              ? MenuResult{"Calibrated", TFT_GREEN, "Alignment stored", nullptr}
              : MenuResult{"Calibration failed", TFT_RED, "Kept the old alignment", "Let go and try again"};
          motorCalState = MOTOR_CAL_IDLE;
          menuPage = MENU_PAGE_RESULT;
          menuResultMs = now;
          menuRedraw = true;
        }
      } else if (presses > 0 || now - menuResultMs >= menuResultForMs) {
        close = true;
      }

      if (close) {
        menuOpen = false;
        menuShown = false;
        firstDraw = true;
        continue;
      }
      if (menuRedraw) {
        menuRedraw = false;
        if (menuPage == MENU_PAGE_LIST) {
          drawMenuScreen(menuSelected);
        } else if (menuPage == MENU_PAGE_POWER) {
          drawPowerScreen(menuPowerLink, menuPowerShown);
        } else if (menuPage == MENU_PAGE_SAVING) {
          drawMessageScreen("Saving", TFT_LIGHTGREY, "Power, speed, length", "and stroke");
        } else if (menuPage == MENU_PAGE_WIFI_SETUP) {
          drawMessageScreen("Wi-Fi setup", TFT_LIGHTGREY, "Join " WIFI_SETUP_SSID, "then open " WIFI_SETUP_ADDRESS);
        } else if (menuPage == MENU_PAGE_UPDATING) {
          menuUpdateShown = updateStepCount;
          menuUpdatePctShown = updateProgress;
          menuUpdateDrawMs = now;
          drawUpdatingScreen();
        } else if (menuPage == MENU_PAGE_CALIBRATING) {
          drawMessageScreen("Calibrating", TFT_LIGHTGREY, "Let go of the handle", "It moves to the middle");
        } else if (menuPage == MENU_PAGE_SETUP_CONFIRM) {
          drawChoiceScreen("Set up ODrive", "Take the motor off the rail", "It takes about 12 minutes",
                           "Cancel", "Start", setupChoice);
        } else if (menuPage == MENU_PAGE_SETUP_STOP) {
          drawChoiceScreen("Stop the setup?", "The ODrive keeps", "its old settings", "Keep on", "Stop", setupChoice);
        } else if (menuPage == MENU_PAGE_SETUP_RUNNING) {
          setupStageShown = controllerSetupStage;
          setupProgressShown = controllerSetupProgress;
          setupElapsedShown = (now - setupStartMs) / 1000;
          setupLinkShown = linkUp;
          drawSetupRunningScreen(setupStageShown, setupProgressShown, setupElapsedShown, setupLinkShown, setupAcked);
        } else {
          drawMessageScreen(menuResult.title, menuResult.color, menuResult.line, menuResult.hint);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (screen == SCREEN_POWERING_ON || screen == SCREEN_BLANK) {
      if (screen == SCREEN_POWERING_ON && (screen != shownScreen || millis() - lastDrawMs >= HOLD_REDRAW_MS)) {
        drawPoweringOnScreen();
        lastDrawMs = millis();
      } else if (screen == SCREEN_BLANK && screen != shownScreen) {
        img.fillSprite(TFT_BLACK);
        img.pushSprite(0, 0);
      }
      shownScreen = screen;
      firstDraw = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (screen == SCREEN_BATTERY_LOW || screen == SCREEN_NO_LINK_OFF) {
      if (screen != shownScreen) {
        if (screen == SCREEN_BATTERY_LOW) {
          drawMessageScreen("Battery low", TFT_RED, "Plug in to charge", "Turning off");
        } else {
          drawMessageScreen("No link", TFT_LIGHTGREY, "Turning off", "Is the controller on?");
        }
        shownScreen = screen;
      }
      firstDraw = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (screen == SCREEN_HOMING_SHORT) {
      if (screen != shownScreen) {
        char reached[24];
        snprintf(reached, sizeof(reached), "Reached %u %%", (unsigned)homingTravelPct);
        drawMessageScreen("Short stroke", TFT_ORANGE, reached, "Press to try again");
        shownScreen = screen;
      }
      firstDraw = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (screen == SCREEN_HOMING_FAILED) {
      if (screen != shownScreen) {
        drawHomingFailedScreen();
        shownScreen = screen;
      }
      firstDraw = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (screen != SCREEN_OPERATING) {
      if (screen != shownScreen || millis() - lastDrawMs >= CHARGING_REDRAW_MS) {
        drawChargingScreen(screen == SCREEN_STARTING);
        shownScreen = screen;
        lastDrawMs = millis();
      }
      firstDraw = true;   // the operating screen draws in full once it takes over
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    // Redraw only when something on screen would change, and not more often than REDRAW_MIN_INTERVAL_MS.
    // arcloop() draws from 'displayed', so values arriving mid-draw can't tear the frame.
    uint8_t now[OM_ELEMENT_COUNT];
    memcpy(now, elements, sizeof(now));
    bool link = linkUp;
    int8_t rssi = linkRssi;   // changes at most once a second
    ChargeState charge = power_charge_state();
    bool handleBattery = charge != CHARGE_NO_BATTERY;
    bool handleCharging = charge == CHARGE_CHARGING;
    uint8_t handlePct = power_battery_percent();
    uint8_t odriveError = controllerOdriveError;
    bool changed = firstDraw || link != displayedLinkUp || (link && rssi != displayedRssi) ||
                   odriveError != displayedOdriveError ||
                   handleBattery != displayedHandleBattery || handleCharging != displayedHandleCharging ||
                   abs(handlePct - displayedHandlePct) >= 2 ||   // the reading wanders a little under motor load
                   memcmp(now, displayed, sizeof(now)) != 0;
    if (changed && millis() - lastDrawMs >= REDRAW_MIN_INTERVAL_MS) {
      memcpy(displayed, now, sizeof(now));
      displayedLinkUp = link;
      displayedRssi = rssi;
      displayedHandleBattery = handleBattery;
      displayedHandleCharging = handleCharging;
      displayedHandlePct = handlePct;
      displayedOdriveError = odriveError;
      firstDraw = false;
      arcloop();
      lastDrawMs = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
