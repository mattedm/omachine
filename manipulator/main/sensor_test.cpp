// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Debugging tool: checks the magnetic encoder (J5) is talking, magnet or not. Enable with
// SENSOR_TEST in main.cpp. Results go to the serial console once a second.
//
// Every reading is taken twice, once with the ESP32's pull-up on MISO and once with its
// pull-down. A sensor that is driving the line gives the same answer both times; a missing
// sensor, an open MISO or an unpowered board gives 0xFFFF with the pull-up and 0x0000 with the
// pull-down, which is reported as "floating".
//
// AS5048A (ENCODER_MT6701 0): reads the error, diagnostics/AGC, magnitude and angle registers.
// With no magnet: diagnostics shows COMP_H (field too weak), AGC near 255 and a small magnitude,
// and the angle wanders. With a magnet at the right height: OCF set, COMP_L and COMP_H clear, AGC
// mid-range.
// MT6701 (ENCODER_MT6701 1): reads 24-bit SSI frames and checks their CRC. With no magnet the
// status shows the field too weak.
#include "sensor_test.h"
#include "board_pins.h"
#include "motor_task.h"   // ENCODER_MT6701

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "sensor_test";

static spi_device_handle_t dev;

static void spiInit(int bits)
{
  spi_bus_config_t bus = {};
  bus.mosi_io_num = ENCODER_MT6701 ? -1 : ENC_MOSI_PIN;
  bus.miso_io_num = ENC_MISO_PIN;
  bus.sclk_io_num = ENC_CLK_PIN;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED));

  spi_device_interface_config_t devcfg = {};
  devcfg.mode = 1;
  devcfg.clock_speed_hz = 1000000;   // slow: this is a wiring check
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 1;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &dev));

  gpio_config_t cs = {};
  cs.pin_bit_mask = 1ULL << ENC_CS_PIN;
  cs.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&cs));
  gpio_set_level(ENC_CS_PIN, 1);
}

static uint32_t transfer(uint32_t tx, int bits)
{
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = bits;
  for (int i = 0; i < bits / 8; i++) {
    t.tx_data[i] = tx >> (bits - 8 - 8 * i);
  }
  gpio_set_level(ENC_CS_PIN, 0);
  esp_rom_delay_us(2);
  spi_device_polling_transmit(dev, &t);
  esp_rom_delay_us(1);
  gpio_set_level(ENC_CS_PIN, 1);
  esp_rom_delay_us(2);
  uint32_t rx = 0;
  for (int i = 0; i < bits / 8; i++) {
    rx = (rx << 8) | t.rx_data[i];
  }
  return rx;
}

static void misoPull(bool up)
{
  gpio_set_pull_mode(ENC_MISO_PIN, up ? GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY);
  esp_rom_delay_us(50);
}

#if !ENCODER_MT6701

// AS5048A command: bit 15 even parity, bit 14 read, bits 13..0 address
static uint16_t readCmd(uint16_t addr)
{
  uint16_t cmd = 0x4000 | (addr & 0x3FFF);
  if (__builtin_parity(cmd)) {
    cmd |= 0x8000;
  }
  return cmd;
}

// Reads one register: the answer comes back in the frame after the command
static uint16_t readReg(uint16_t addr)
{
  transfer(readCmd(addr), 16);
  return transfer(readCmd(0x0000), 16);   // NOP
}

static const char *checkFrame(uint16_t r)
{
  if (__builtin_parity(r)) return "PARITY BAD";
  if (r & 0x4000) return "error flag";
  return "ok";
}

void sensorTest()
{
  ESP_LOGW(TAG, "SENSOR TEST BUILD (AS5048A): no motor, link or display");
  spiInit(16);
  for (;;) {
    misoPull(true);
    uint16_t angleUp = readReg(0x3FFF);
    misoPull(false);
    uint16_t angleDown = readReg(0x3FFF);
    misoPull(true);

    if (angleUp == 0xFFFF && angleDown == 0x0000) {
      ESP_LOGE(TAG, "MISO floating: no answer from the sensor (check 3V3, GND, CLK, CS and MISO at J5)");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    uint16_t err = readReg(0x0001);    // reading clears it
    uint16_t diag = readReg(0x3FFD);
    uint16_t mag = readReg(0x3FFE);
    uint16_t angle = readReg(0x3FFF);

    ESP_LOGI(TAG, "angle %5u (%6.1f deg) [%s]  magnitude %5u [%s]",
             angle & 0x3FFF, (angle & 0x3FFF) * 360.0f / 16384.0f, checkFrame(angle),
             mag & 0x3FFF, checkFrame(mag));
    ESP_LOGI(TAG, "  diag 0x%04x: AGC %u  OCF %d  COF %d  COMP_L %d  COMP_H %d%s   error reg 0x%04x%s%s%s",
             diag, diag & 0xFF, (diag >> 8) & 1, (diag >> 9) & 1, (diag >> 10) & 1, (diag >> 11) & 1,
             (diag >> 11) & 1 ? " (field too weak: no magnet?)" : (diag >> 10) & 1 ? " (field too strong)" : "",
             err & 0x3FFF, err & 1 ? " framing" : "", err & 2 ? " bad-command" : "", err & 4 ? " parity" : "");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#else

static uint8_t crc6(uint32_t data18)
{
  uint8_t crc = 0;
  for (int i = 17; i >= 0; i--) {
    uint8_t feedback = ((crc >> 5) & 1) ^ ((data18 >> i) & 1);
    crc = (crc << 1) & 0x3F;
    if (feedback) crc ^= 0x03;
  }
  return crc;
}

void sensorTest()
{
  ESP_LOGW(TAG, "SENSOR TEST BUILD (MT6701): no motor, link or display");
  spiInit(24);
  for (;;) {
    misoPull(true);
    uint32_t up = transfer(0, 24);
    misoPull(false);
    uint32_t down = transfer(0, 24);
    misoPull(true);
    if (up == 0xFFFFFF && down == 0) {
      ESP_LOGE(TAG, "MISO floating: no answer from the sensor (check 3V3, GND, CLK, CS and DO at J5)");
    } else {
      uint32_t f = transfer(0, 24);
      uint32_t st = (f >> 6) & 0xF;
      ESP_LOGI(TAG, "frame 0x%06lx: angle %lu (%.1f deg)  status 0x%lx%s  CRC %s",
               (unsigned long)f, (unsigned long)(f >> 10), (f >> 10) * 360.0f / 16384.0f, (unsigned long)st,
               (st & 3) == 2 ? " (field too weak: no magnet?)" : (st & 3) == 1 ? " (field too strong)" : "",
               crc6(f >> 6) == (f & 0x3F) ? "ok" : "BAD");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#endif
