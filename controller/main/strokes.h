// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// The auto strokes. Each is a function of time (strokes.c) that gives the offset into the stroke,
// 0..length, scaled to the Length setting; the Speed setting scales its time.
//
// To change a stroke, edit its function in strokes.c. To add one:
//   1. write its function in strokes.c: start at 0 when millis is 0, and repeat smoothly
//      (end where it started, without a jump in position or speed);
//   2. add its id to stroke_id_t below, before STROKE_COUNT;
//   3. add it to s_strokes[] in strokes.c, with its name;
//   4. add the same name, at the same place, to strokes[] in manipulator/main/display_task.cpp,
//      which is what the handle shows (both lists must be in the same order).
// The knob steps through them in this order. Stroke tables made from simulations (Pendulum,
// Lorenz) come from tools/gen_loop_strokes.py.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STROKE_JACK = 0,
    STROKE_PULSE,
    STROKE_PUMP,
    STROKE_CLASSIC,
    STROKE_TEASE,
    STROKE_CRESCENDO,
    STROKE_SLOW_IN,
    STROKE_HEARTBEAT,
    STROKE_BREATHING,
    STROKE_WANDER,
    STROKE_PENDULUM,
    STROKE_BOUNCE,
    STROKE_SPRING,
    STROKE_LORENZ,
    STROKE_GALLOP,
    STROKE_SWING,
    STROKE_RAMP_UP,
    STROKE_FLUTTER,
    STROKE_WAVES,
    STROKE_COUNT
} stroke_id_t;

// The stroke's offset (0..length) at millis into it; 0 for an unknown stroke
float stroke_eval(stroke_id_t stroke, uint32_t millis, float length);
// Its name, as the handle shows it
const char *stroke_name(stroke_id_t stroke);

#ifdef __cplusplus
}
#endif
