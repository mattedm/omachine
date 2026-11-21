// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fast_mt6701.h"

#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "mt6701";

// CRC-6, polynomial x^6 + x + 1, over the 18 data bits, MSB first
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

static bool frameValid(uint32_t frame)
{
  return crc6(frame >> 6) == (frame & 0x3F);
}

void FastMT6701::init()
{
  spi_bus_config_t bus = {};
  bus.mosi_io_num = -1;   // SSI has no data into the chip
  bus.miso_io_num = miso;
  bus.sclk_io_num = sclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  ESP_ERROR_CHECK(spi_bus_initialize(host, &bus, SPI_DMA_DISABLED));

  spi_device_interface_config_t devcfg = {};
  devcfg.mode = 1;   // CPOL 0, CPHA 1: DO changes on the rising edge, sampled on the falling edge
  devcfg.clock_speed_hz = clock_speed;
  devcfg.spics_io_num = -1;   // chip select driven by hand, see readFrame()
  devcfg.queue_size = 1;
  ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &dev));
  // the sensor is alone on its bus and only the motor task reads it
  ESP_ERROR_CHECK(spi_device_acquire_bus(dev, portMAX_DELAY));

  gpio_config_t cs_cfg = {};
  cs_cfg.pin_bit_mask = 1ULL << cs;
  cs_cfg.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&cs_cfg));
  gpio_set_level(cs, 1);

  // A few frames to confirm the wiring and the CRC before the motor relies on them
  int good = 0;
  uint32_t frame = 0;
  for (int i = 0; i < 8; i++) {
    frame = readFrame();
    if (frameValid(frame)) good++;
    esp_rom_delay_us(10);
  }
  ESP_LOGI(TAG, "Frame 0x%06lx: angle %lu, status 0x%lx, %d of 8 frames passed CRC",
           (unsigned long)frame, (unsigned long)(frame >> 10), (unsigned long)((frame >> 6) & 0xF), good);
  if (frame == 0 || frame == 0xFFFFFF) {
    ESP_LOGE(TAG, "No data from the MT6701: check the cable, and that the breakout's MODE pin is high");
  } else if (good == 0 && MT6701_CHECK_CRC) {
    ESP_LOGW(TAG, "No frame passed CRC: if the angle above changes as the handle turns, set MT6701_CHECK_CRC to 0");
  }

  Sensor::init();
}

float FastMT6701::getSensorAngle()
{
  uint32_t frame = readFrame();
#if MT6701_CHECK_CRC
  if (!frameValid(frame)) {
    crc_errors = crc_errors + 1;
    return last_angle;
  }
#endif
  uint32_t status = (frame >> 6) & 0x3;
  if (status == 1 || status == 2) {
    field_warnings = field_warnings + 1;
  }
  last_angle = (frame >> 10) * (_2PI / 16384.0f);
  return last_angle;
}

uint32_t FastMT6701::readFrame()
{
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_RXDATA;
  t.length = 24;
  t.rxlength = 24;
  // CSN low at least 100 ns before the first clock edge, as the AS5048A reader does by hand
  gpio_set_level(cs, 0);
  esp_rom_delay_us(1);
  spi_device_polling_transmit(dev, &t);
  gpio_set_level(cs, 1);
  return ((uint32_t)t.rx_data[0] << 16) | ((uint32_t)t.rx_data[1] << 8) | t.rx_data[2];
}
