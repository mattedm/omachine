// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later

#include "log_async.h"

#include <stdarg.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"

#define LOG_BUFFER_BYTES 8192
#define LOG_LINE_MAX     256          // longer lines are cut short
#define LOG_TASK_PRIO    1            // below everything that does real work

static RingbufHandle_t s_buf;
static volatile uint32_t s_dropped = 0;

static int async_vprintf(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int len = vsnprintf(NULL, 0, fmt, copy);
    va_end(copy);
    if (len <= 0) {
        return len;
    }
    if (len >= LOG_LINE_MAX) {
        len = LOG_LINE_MAX - 1;
    }
    // Format straight into the buffer: no line-sized copy on the caller's (possibly small) stack
    void *item = NULL;
    if (xRingbufferSendAcquire(s_buf, &item, len + 1, 0) != pdTRUE || item == NULL) {
        s_dropped = s_dropped + 1;
        return len;
    }
    vsnprintf((char *)item, len + 1, fmt, args);
    xRingbufferSendComplete(s_buf, item);
    return len;
}

static void log_task(void *arg)
{
    uint32_t reported = 0;
    for (;;) {
        size_t size = 0;
        char *line = (char *)xRingbufferReceive(s_buf, &size, portMAX_DELAY);
        if (line) {
            fwrite(line, 1, size - 1, stdout);   // size counts the terminating 0
            vRingbufferReturnItem(s_buf, line);
        }
        uint32_t dropped = s_dropped;
        if (dropped != reported && xRingbufferGetCurFreeSize(s_buf) > LOG_BUFFER_BYTES / 2) {
            printf("W log: %" PRIu32 " log lines dropped (the console couldn't keep up)\n", dropped - reported);
            reported = dropped;
        }
        fflush(stdout);
    }
}

void log_async_start(void)
{
    s_buf = xRingbufferCreate(LOG_BUFFER_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_buf == NULL) {
        return;   // logging stays direct
    }
    xTaskCreate(log_task, "log", 3072, NULL, LOG_TASK_PRIO, NULL);
    esp_log_set_vprintf(async_vprintf);
}
