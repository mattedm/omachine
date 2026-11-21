// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// Pin assignment of the DRV8311 1S battery board: U6, an ESP32-S3-MINI-1-N8 (no PSRAM, so GPIO26 is free).
// Net names are those in drv8311-breakout.kicad_sch.
#pragma once

#include "driver/gpio.h"

// DRV8311H motor driver
#define MOTOR_PWM_A_PIN    GPIO_NUM_37   // M1A -> INHA
#define MOTOR_PWM_B_PIN    GPIO_NUM_38   // M1B -> INHB
#define MOTOR_PWM_C_PIN    GPIO_NUM_39   // M1C -> INHC
#define MOTOR_SLEEP_PIN    GPIO_NUM_36   // SLEEP -> nSLEEP, 100k pull-down: motor off until driven high
#define MOTOR_FAULT_PIN    GPIO_NUM_35   // FAULT <- nFAULT, 100k pull-up
// Low-side current sense outputs (GAIN tied to AVDD: 2 V/A, 0 A at CSAREF/2 = 1.65 V, linear
// 0.25 V to 3.05 V, so about +/-0.7 A). Only valid while that phase's low side is on.
#define MOTOR_SOA_ADC_CHANNEL ADC_CHANNEL_7   // GPIO8: SOA
#define MOTOR_SOB_ADC_CHANNEL ADC_CHANNEL_8   // GPIO9: SOB

// AS5048A magnetic encoder (J5)
#define ENC_CLK_PIN        GPIO_NUM_47
#define ENC_MISO_PIN       GPIO_NUM_34
#define ENC_MOSI_PIN       GPIO_NUM_33
#define ENC_CS_PIN         GPIO_NUM_48

// GC9A01 display (J8). CS and RST are on UART0's pins, so the console must stay on USB
#define LCD_CLK_PIN        GPIO_NUM_40
#define LCD_MOSI_PIN       GPIO_NUM_41
#define LCD_DC_PIN         GPIO_NUM_42
#define LCD_CS_PIN         GPIO_NUM_43
#define LCD_RST_PIN        GPIO_NUM_44
#define LCD_BL_PIN         GPIO_NUM_4    // high = backlight on

// Illuminated power/auto button (J6). The LED's anode is on +3V3, so its pins are active low
#define BUTTON_PIN         GPIO_NUM_13   // BTN_SENSE, low while pressed (Q1, 100k pull-up)
#define BUTTON_LIGHT_R_PIN GPIO_NUM_16
#define BUTTON_LIGHT_G_PIN GPIO_NUM_17
#define BUTTON_LIGHT_B_PIN GPIO_NUM_18

// Rotary encoder (J7). A and B have 10k pull-ups and RC filters on the board; the switch has neither
#define ROTARY_PIN1        GPIO_NUM_21   // ENCA
#define ROTARY_PIN2        GPIO_NUM_26   // ENCB
#define ROTARY_BUTTON_PIN  GPIO_NUM_7    // ROT_SW2, needs the internal pull-up

// Power
#define POWER_HOLD_PIN     GPIO_NUM_12   // PWR_HOLD: high keeps the regulator on when running from the battery
#define USB_POWER_GOOD_PIN GPIO_NUM_5    // PGOOD (BQ24075), low while USB power is valid
#define CHARGE_STATUS_PIN  GPIO_NUM_6    // CHG (BQ24075), low while charging (R11, 100k pull-up)
#define VBAT_ADC_UNIT      ADC_UNIT_1
#define VBAT_ADC_CHANNEL   ADC_CHANNEL_0 // GPIO1: VBATLEVEL, the battery through a 1M/1M divider
