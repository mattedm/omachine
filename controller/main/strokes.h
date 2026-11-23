// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// The auto strokes. Each is a function of time (strokes.c) that gives the offset into the stroke,
// 0..length, scaled to the Length setting; the Speed setting scales its time.
//
// Strokes can be written at any pace: each is played at its own rate so that all of them reach the
// same top speed at a given Speed setting (stroke_pace()). What matters is a stroke's shape.
//
// To change a stroke, edit its function in strokes.c. To add one:
//   1. write its function in strokes.c: start at 0 when millis is 0, and repeat smoothly
//      (end where it started, without a jump in position or speed);
//   2. add its id to stroke_id_t below, before STROKE_COUNT;
//   3. add it to s_strokes[] in strokes.c, with its name (what the handle shows).
// The handle gets the names from the controller over the link, so nothing changes on the manipulator.
// Names can be up to OM_STROKE_NAME_LEN - 1 (15) characters, and there can be up to OM_MAX_STROKES.
// The knob steps through them in this order. tools/plot_strokes.py redraws docs/strokes.png. Stroke tables made from simulations (Pendulum,
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
// The top speed every stroke is paced to, in stroke lengths per second at Speed 1x. It's the median
// of the strokes' own top speeds, so the middle stroke plays as written; tools/plot_strokes.py
// prints each stroke's top speed and pace, and suggests this value after the strokes change.
#define STROKE_TOP_SPEED 3.4f

// How fast its time runs (times the Speed setting's rate), so that every stroke reaches the same top
// speed at the same Speed setting; measured from the stroke itself, so a new stroke is paced too
float stroke_pace(stroke_id_t stroke);
// Measures every stroke's pace now, so choosing one doesn't take a moment in the control loop
void strokes_init(void);

#ifdef __cplusplus
}
#endif
