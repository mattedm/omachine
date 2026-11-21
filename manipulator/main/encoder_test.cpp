// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Debugging tool: rotary encoder (J7) diagnostic screen. Enable with ENCODER_TEST in main.cpp.
//
// Shows rotary A and B live, a trace of both (2 ms a pixel, held after the last movement so it
// can be read), edge counts per channel from GPIO interrupts, a x4 quadrature count with the
// number of impossible transitions (both lines changing at once), the firmware's own step count
// (rotaryCount, from the 1 ms Rotary state machine) and the two buttons. The same numbers go to
// the serial console once a second.
//
// A good knob and cable: A and B edge counts about equal, Bad stays at (or near) 0, Quad moves 4
// and Steps 1 per detent. One channel with far more edges than the other, Bad climbing and Steps
// stuck at 0 is a chattering line, such as a bad crimp or a strand touching the next GH pin.
#include "encoder_test.h"
#include "board_pins.h"
#include "inputs.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "fonts/roboto_light_16.h"
#include "fonts/roboto_light_24.h"

static const char *TAG = "enc_test";

class LGFX_EncTest : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;

public:
  LGFX_EncTest()
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

static LGFX_EncTest lcd;
static LGFX_Sprite img(&lcd);

// Interrupt counts
static volatile uint32_t edgesA = 0, edgesB = 0;
static volatile int32_t quadCount = 0;
static volatile uint32_t badTransitions = 0;
static volatile uint8_t lastAB = 0;

// Trace: one sample (bit 0 = A, bit 1 = B) every SAMPLE_US
#define SAMPLE_US   2000
#define RING_SIZE   1024
#define TRACE_W     200
#define TRACE_HOLD  40   // samples shown after the last edge before the trace stops scrolling
static uint8_t ring[RING_SIZE];
static volatile uint32_t ringIdx = 0;      // samples written so far
static volatile uint32_t lastChangeIdx = 0;

static inline uint8_t readAB()
{
  return (gpio_get_level(ROTARY_PIN2) << 1) | gpio_get_level(ROTARY_PIN1);
}

// x4 decode: index is (old << 2) | new; 2 marks an impossible transition
static const int8_t quadTable[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};

static void IRAM_ATTR abIsr(void *arg)
{
  if ((int)(intptr_t)arg == 0) {
    edgesA = edgesA + 1;
  } else {
    edgesB = edgesB + 1;
  }
  uint8_t ab = (gpio_get_level(ROTARY_PIN2) << 1) | gpio_get_level(ROTARY_PIN1);
  int8_t d = quadTable[(lastAB << 2) | ab];
  if (d == 2) {
    badTransitions = badTransitions + 1;
  } else {
    quadCount = quadCount + d;
  }
  lastAB = ab;
}

static void sampleTimer(void *arg)
{
  uint32_t i = ringIdx;
  uint8_t s = readAB();
  if (i > 0 && s != ring[(i - 1) % RING_SIZE]) {
    lastChangeIdx = i;
  }
  ring[i % RING_SIZE] = s;
  ringIdx = i + 1;
}

static void drawLevelBox(int x, int y, const char *name, bool high)
{
  uint16_t color = high ? TFT_GREEN : TFT_RED;
  img.fillRoundRect(x, y, 70, 34, 6, color);
  img.setTextColor(TFT_BLACK);
  img.setFont(&Roboto_Light_24);
  img.setTextDatum(MC_DATUM);
  char text[12];
  snprintf(text, sizeof(text), "%s %d", name, high ? 1 : 0);
  img.drawString(text, x + 35, y + 18);
}

static void drawTrace(const uint8_t *samples, int x0, int yTop, int h, int bit, uint16_t color)
{
  img.drawFastHLine(x0, yTop + h, TRACE_W, TFT_DARKGREY);
  for (int i = 0; i < TRACE_W; i++) {
    bool level = samples[i] & (1 << bit);
    int y = level ? yTop : yTop + h;
    img.drawPixel(x0 + i, y, color);
    if (i > 0 && ((samples[i - 1] ^ samples[i]) & (1 << bit))) {
      img.drawFastVLine(x0 + i, yTop, h + 1, color);
    }
  }
}

void encoderTest()
{
  ESP_LOGW(TAG, "ENCODER TEST BUILD: motor, link and normal display are not running");

  lcd.init();
  lcd.setRotation(2);
  lcd.fillScreen(TFT_BLACK);
  gpio_set_level(LCD_BL_PIN, 1);
  gpio_config_t backlight = {};
  backlight.pin_bit_mask = 1ULL << LCD_BL_PIN;
  backlight.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&backlight));
  img.setColorDepth(16);
  if (img.createSprite(240, 240) == nullptr) {
    ESP_LOGE(TAG, "No memory for the sprite");
    vTaskDelete(NULL);
  }

  // inputs_init has made A and B inputs (board pull-ups); add interrupts on both edges
  lastAB = readAB();
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(err);
  }
  gpio_set_intr_type(ROTARY_PIN1, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(ROTARY_PIN2, GPIO_INTR_ANYEDGE);
  gpio_isr_handler_add(ROTARY_PIN1, abIsr, (void *)0);
  gpio_isr_handler_add(ROTARY_PIN2, abIsr, (void *)1);

  const esp_timer_create_args_t timerArgs = {
      .callback = &sampleTimer,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "enc_trace",
      .skip_unhandled_events = true,
  };
  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SAMPLE_US));

  static uint8_t shown[TRACE_W];
  memset(shown, readAB(), sizeof(shown));
  int64_t lastLog = 0;
  char text[40];

  for (;;) {
    // Trace window: ends TRACE_HOLD samples after the last change, so it stops once the knob does
    uint32_t now = ringIdx;
    uint32_t end = lastChangeIdx + TRACE_HOLD;
    if (end > now) {
      end = now;
    }
    if (now - end < RING_SIZE - TRACE_W && end >= TRACE_W) {
      for (int i = 0; i < TRACE_W; i++) {
        shown[i] = ring[(end - TRACE_W + i) % RING_SIZE];
      }
    }

    uint8_t ab = readAB();
    img.fillScreen(TFT_BLACK);

    img.setTextColor(TFT_WHITE);
    img.setFont(&Roboto_Light_16);
    img.setTextDatum(MC_DATUM);
    img.drawString("Encoder test", 120, 22);

    drawLevelBox(43, 38, "A", ab & 1);
    drawLevelBox(127, 38, "B", ab & 2);

    drawTrace(shown, 20, 86, 22, 0, TFT_YELLOW);
    drawTrace(shown, 20, 118, 22, 1, TFT_CYAN);
    img.setFont(&Roboto_Light_16);
    img.setTextDatum(MR_DATUM);
    img.setTextColor(TFT_YELLOW);
    img.drawString("A", 16, 97);
    img.setTextColor(TFT_CYAN);
    img.drawString("B", 16, 129);
    img.setTextColor(TFT_DARKGREY);
    img.setTextDatum(MC_DATUM);
    img.drawString("400 ms", 120, 152);

    img.setTextColor(TFT_WHITE);
    snprintf(text, sizeof(text), "Edges A %lu  B %lu", (unsigned long)edgesA, (unsigned long)edgesB);
    img.drawString(text, 120, 172);
    snprintf(text, sizeof(text), "Quad %ld  Bad %lu", (long)quadCount, (unsigned long)badTransitions);
    img.drawString(text, 120, 192);
    snprintf(text, sizeof(text), "Steps %d  %s %s", (int)(int8_t)rotaryCount,
             rotaryButton.pressed ? "SW" : "--", autoButton.pressed ? "BTN" : "---");
    img.drawString(text, 120, 212);

    img.pushSprite(0, 0);

    int64_t t = esp_timer_get_time();
    if (t - lastLog > 1000000) {
      lastLog = t;
      ESP_LOGI(TAG, "A=%d B=%d edgesA=%lu edgesB=%lu quad=%ld bad=%lu steps=%d",
               ab & 1, (ab >> 1) & 1, (unsigned long)edgesA, (unsigned long)edgesB,
               (long)quadCount, (unsigned long)badTransitions, (int)(int8_t)rotaryCount);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
