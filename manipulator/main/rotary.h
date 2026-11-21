// SPDX-FileCopyrightText: 2011 Ben Buxton
// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-only
/*
 * Rotary encoder handler, ported from Ben Buxton's Arduino library v1.1 (full-step mode).
 *
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 * Contact: bb@cactii.net
 * Port to ESP-IDF: Copyright 2026 MattEdm, under the same licence.
 */
#pragma once

#include "driver/gpio.h"

// Values returned by 'process'
#define DIR_NONE 0x0   // no complete step yet
#define DIR_CW 0x10    // clockwise step
#define DIR_CCW 0x20   // anti-clockwise step

class Rotary
{
  public:
    Rotary(gpio_num_t pin1, gpio_num_t pin2) : pin1(pin1), pin2(pin2) {}
    // Process pin(s); the pins must already be inputs with pull-ups
    unsigned char process();
  private:
    unsigned char state = 0;
    gpio_num_t pin1;
    gpio_num_t pin2;
};
