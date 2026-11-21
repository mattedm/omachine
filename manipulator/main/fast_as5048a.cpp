// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fast_as5048a.h"

#include "esp_rom_sys.h"

// commands include the read bit (14) and even parity bit (15)
static const uint16_t CMD_READ_ANGLE  = 0xFFFF;  // read 0x3FFF
static const uint16_t CMD_CLEAR_ERROR = 0x4001;  // read 0x0001 (clears error flag)

void FastAS5048A::init()
{
  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = sclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  ESP_ERROR_CHECK(spi_bus_initialize(host, &bus, SPI_DMA_DISABLED));

  spi_device_interface_config_t devcfg = {};
  devcfg.mode = 1;   // CPOL 0, CPHA 1
  devcfg.clock_speed_hz = clock_speed;
  devcfg.spics_io_num = -1;   // chip select driven by hand, see transfer()
  devcfg.queue_size = 1;
  ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, &dev));
  // the sensor is alone on its bus and only the motor task reads it
  ESP_ERROR_CHECK(spi_device_acquire_bus(dev, portMAX_DELAY));

  gpio_config_t cs_cfg = {};
  cs_cfg.pin_bit_mask = 1ULL << cs;
  cs_cfg.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&cs_cfg));
  gpio_set_level(cs, 1);

  clearErrorAndPrime();
  Sensor::init();
}

float FastAS5048A::getSensorAngle()
{
  uint16_t resp = transfer(CMD_READ_ANGLE);

  // bit 15 = even parity, bit 14 = error flag, bits 13..0 = angle
  if (__builtin_parity(resp) || (resp & 0x4000)) {
    errors = errors + 1;
    if (resp & 0x4000) clearErrorAndPrime();
    return last_angle;
  }
  last_angle = (resp & 0x3FFF) * (_2PI / 16384.0f);
  return last_angle;
}

uint16_t FastAS5048A::transfer(uint16_t cmd)
{
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 16;
  t.tx_data[0] = cmd >> 8;
  t.tx_data[1] = cmd & 0xFF;
  // The AS5048A needs CSn low >= 350 ns before the first clock edge; the driver's own CS gives
  // under one clock period in full-duplex mode, so drive it by hand as the Arduino version did
  gpio_set_level(cs, 0);
  esp_rom_delay_us(1);
  spi_device_polling_transmit(dev, &t);
  gpio_set_level(cs, 1);
  return (uint16_t)((t.rx_data[0] << 8) | t.rx_data[1]);
}

// Clear the error flag, then queue a read-angle command so the next
// getSensorAngle() call receives an angle. Responses here are discarded.
void FastAS5048A::clearErrorAndPrime()
{
  transfer(CMD_CLEAR_ERROR);
  esp_rom_delay_us(1);        // CSn high time between frames (min 350ns)
  transfer(CMD_READ_ANGLE);
  esp_rom_delay_us(1);
}
