// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Fast AS5048A reader for SimpleFOC, on the ESP-IDF SPI master driver.
 *
 * The AS5048A answers each SPI command during the *next* frame. By sending
 * the read-angle command on every transfer, each reading needs only one
 * 16-bit transfer and no delay; the angle returned is the one requested by
 * the previous call (one control loop old).
 *
 * esp_simplefoc's own AS5048a class does two transfers per reading and lets the
 * SPI driver drive chip select, which gives the sensor too little CSn setup time.
 */
#pragma once

#include "esp_simplefoc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

class FastAS5048A : public Sensor
{
  public:
    FastAS5048A(spi_host_device_t host, gpio_num_t sclk, gpio_num_t miso, gpio_num_t mosi, gpio_num_t cs)
      : host(host), sclk(sclk), miso(miso), mosi(mosi), cs(cs) {}

    int clock_speed = 5000000;     // AS5048A supports up to 10 MHz
    volatile uint32_t errors = 0;  // count of rejected readings (parity / error flag)

    void init() override;

  protected:
    float getSensorAngle() override;

  private:
    spi_host_device_t host;
    gpio_num_t sclk, miso, mosi, cs;
    spi_device_handle_t dev = nullptr;
    float last_angle = 0;

    uint16_t transfer(uint16_t cmd);
    void clearErrorAndPrime();
};
