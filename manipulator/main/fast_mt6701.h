// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * MT6701 reader for SimpleFOC over SSI, on the ESP-IDF SPI master driver.
 *
 * SSI is read-only: each reading is one 24-bit frame clocked out on DO while CSN is low,
 * with no command to send (MT6701 datasheet rev 1.8, section 7.8):
 *   bits 23..10  angle D[13:0]
 *   bits  9..6   magnetic status Mg[3:0] (Mg[1:0]: 1 field too strong, 2 field too weak)
 *   bits  5..0   CRC-6 over the 18 bits above, polynomial x^6 + x + 1, MSB first
 * DO changes on CLK's rising edge and is sampled on the falling edge (SPI mode 1).
 * The chip's MODE pin must be high (I2C/SSI) for its pins 6-8 to be DO, CLK and CSN.
 */
#pragma once

#include "esp_simplefoc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Reject frames whose CRC doesn't match. If every reading is rejected (crc_errors counts up on
// every loop and init() logs a warning), set this to 0 and report it: the CRC init value is not
// given in the datasheet, and 0 is assumed.
#define MT6701_CHECK_CRC 1

class FastMT6701 : public Sensor
{
  public:
    FastMT6701(spi_host_device_t host, gpio_num_t sclk, gpio_num_t miso, gpio_num_t cs)
      : host(host), sclk(sclk), miso(miso), cs(cs) {}

    int clock_speed = 5000000;           // MT6701 supports up to about 15 MHz (TCLK >= 64 ns)
    volatile uint32_t crc_errors = 0;    // frames rejected for a bad CRC
    volatile uint32_t field_warnings = 0;  // frames flagging the magnet too strong or too weak

    void init() override;

  protected:
    float getSensorAngle() override;

  private:
    spi_host_device_t host;
    gpio_num_t sclk, miso, cs;
    spi_device_handle_t dev = nullptr;
    float last_angle = 0;

    uint32_t readFrame();
};
