// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shared type definitions
typedef struct {
    uint16_t value;
} encoder_estimate_msg_t;

// External queue declarations
extern QueueHandle_t odrive_encoder_queue;

extern QueueHandle_t manipulator_encoder_queue;

void espnow_manipulator_task(void *pvParameters);



#ifdef __cplusplus
}
#endif

#endif // SHARED_TYPES_H