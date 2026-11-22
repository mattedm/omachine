// SPDX-FileCopyrightText: 2026 MattEdm
// SPDX-License-Identifier: GPL-3.0-or-later
// The auto strokes: one function per stroke, and the table (s_strokes) that names them. See
// strokes.h for how to add or change one.

#include "strokes.h"
#include "omachine_protocol.h"
#include "stroke_pendulum_table.h"
#include "stroke_lorenz_table.h"

#include <math.h>
#include <stddef.h>

static float stroke_classic(uint32_t millis, float length)
{
    return length * ((cosf((millis / 1000.0f) * (float)M_PI) - 1.0f) / -2.0f);
}

static float stroke_pulse(uint32_t millis, float length)
{
    float distance = (float)(millis % 2000u);
    if (distance >= 1000.0f) {
        distance = 2000.0f - distance;
    }
    distance = distance / 1000.0f;
    distance = (0.7f * distance) + (0.3f * ((cosf((millis / 107.0f) * (float)M_PI) - 1.0f) / -2.0f));
    return length * distance;
}

static float stroke_pump(uint32_t millis, float length)
{
    float distance = (float)(millis % 3000u);
    if (distance < 2000.0f) {
        distance = distance / 2000.0f;
    } else {
        const float factor = 160.0f;
        distance = (((-(powf(2.0f, (distance - 2000.0f) / factor)) + 1.0f) /
                    (powf(2.0f, 1000.0f / factor) - 1.0f)) + 1.0f);
    }
    return length * distance;
}

static float stroke_jack(uint32_t millis, float length)
{
    float distance = (float)(millis % 2000u);
    if (distance >= 1000.0f) {
        distance = 2000.0f - distance;
    }
    distance = distance / 1000.0f;
    return length * distance;
}

// One cosine stroke out to depth (0..1) and back, phase 0..1
static float cosine_stroke(float phase, float depth)
{
    return depth * (1.0f - cosf(phase * 2.0f * (float)M_PI)) / 2.0f;
}

// Three shallow strokes, then one full one
static float stroke_tease(uint32_t millis, float length)
{
    uint32_t cycle = millis % 4000u;
    float depth = (cycle < 3000u) ? 0.3f : 1.0f;
    return length * cosine_stroke((cycle % 1000u) / 1000.0f, depth);
}

// Each stroke deeper than the last, 20% to 100% over 8 strokes, then shallow again
static float stroke_crescendo(uint32_t millis, float length)
{
    uint32_t n = (millis / 1000u) % 8u;
    float depth = 0.2f + 0.8f * n / 7.0f;
    return length * cosine_stroke((millis % 1000u) / 1000.0f, depth);
}

// Slow out to full (1.2 s), quick return (0.3 s)
static float stroke_slow_in(uint32_t millis, float length)
{
    uint32_t t = millis % 1500u;
    float d = (t < 1200u) ? (1.0f - cosf((float)M_PI * t / 1200.0f)) / 2.0f
                          : (1.0f + cosf((float)M_PI * (t - 1200u) / 300.0f)) / 2.0f;
    return length * d;
}

// A strong beat, a softer one, then a rest
static float stroke_heartbeat(uint32_t millis, float length)
{
    uint32_t t = millis % 1600u;
    if (t < 350u) {
        return length * cosine_stroke(t / 350.0f, 1.0f);
    }
    if (t < 700u) {
        return length * cosine_stroke((t - 350u) / 350.0f, 0.6f);
    }
    return 0.0f;
}

// 1 s strokes whose depth swells from 30% to 100% and back over 10 s
static float stroke_breathing(uint32_t millis, float length)
{
    float depth = 0.3f + 0.7f * (1.0f - cosf(2.0f * (float)M_PI * millis / 10000.0f)) / 2.0f;
    return length * depth * (1.0f - cosf((float)M_PI * millis / 500.0f)) / 2.0f;
}

// Three smooth waves with unrelated periods, so the pattern never quite repeats. Each term runs
// 0..weight (weights sum to 1) and starts at 0, so there is no jump when AUTO starts.
static float stroke_wander(uint32_t millis, float length)
{
    float t = millis / 1000.0f;
    float d = 0.6f * (1.0f - cosf(2.0f * (float)M_PI * t / 1.3f)) / 2.0f
            + 0.3f * (1.0f - cosf(2.0f * (float)M_PI * t / 2.9f)) / 2.0f
            + 0.1f * (1.0f - cosf(2.0f * (float)M_PI * t / 0.7f)) / 2.0f;
    return length * d;
}

// A looped stroke from tools/gen_loop_strokes.py (n samples, 0..32767, at rate Hz), smoothly
// interpolated (Catmull-Rom) between samples
static float loop_stroke_eval(const uint16_t *s, int n, int rate, uint32_t millis, float length)
{
    const uint32_t loop_ms = (uint32_t)n * 1000u / (uint32_t)rate;
    float pos = (float)(millis % loop_ms) * (float)rate / 1000.0f;
    int i = (int)pos;
    float f = pos - (float)i;
    float p0 = s[(i + n - 1) % n];
    float p1 = s[i % n];
    float p2 = s[(i + 1) % n];
    float p3 = s[(i + 2) % n];
    float d = 0.5f * (2.0f * p1 + (p2 - p0) * f + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * f * f +
                      (3.0f * (p1 - p2) + p3 - p0) * f * f * f) / 32767.0f;
    return length * fminf(fmaxf(d, 0.0f), 1.0f);   // the curve can overshoot a sample a little
}

// A double pendulum's lower bob, swinging side to side: irregular swings and flicks (a 20 s loop)
static float stroke_pendulum(uint32_t millis, float length)
{
    return loop_stroke_eval(pendulum_samples, PENDULUM_SAMPLES, PENDULUM_RATE_HZ, millis, length);
}

// The Lorenz system: circling near one end with growing swings, then flipping to the other,
// unpredictably (a 24 s loop)
static float stroke_lorenz(uint32_t millis, float length)
{
    return loop_stroke_eval(lorenz_samples, LORENZ_SAMPLES, LORENZ_RATE_HZ, millis, length);
}

// Ease 0 -> 1 over u = 0..1, starting and ending at rest
static float ease(float u)
{
    return (1.0f - cosf((float)M_PI * u)) / 2.0f;
}

// A dropped ball: a full-depth drop (depth = how far it has fallen), bounces that get shorter and
// quicker, a rest on the floor, then a smooth lift back to the top and a pause
#define BOUNCE_DROP_S     0.45f
#define BOUNCE_E          0.65f    // restitution: each bounce leaves at this share of the speed
#define BOUNCE_COUNT      4
#define BOUNCE_REST_S     0.35f
#define BOUNCE_LIFT_S     0.9f
#define BOUNCE_PAUSE_S    0.3f
static float stroke_bounce(uint32_t millis, float length)
{
    // duration of bounce k: 2 * drop * e^k; total of BOUNCE_COUNT of them
    float bounces_s = 0.0f;
    for (int k = 1; k <= BOUNCE_COUNT; k++) {
        bounces_s += 2.0f * BOUNCE_DROP_S * powf(BOUNCE_E, (float)k);
    }
    float cycle_s = BOUNCE_DROP_S + bounces_s + BOUNCE_REST_S + BOUNCE_LIFT_S + BOUNCE_PAUSE_S;
    float t = fmodf(millis / 1000.0f, cycle_s);
    if (t < BOUNCE_DROP_S) {
        float u = t / BOUNCE_DROP_S;
        return length * u * u;
    }
    t -= BOUNCE_DROP_S;
    for (int k = 1; k <= BOUNCE_COUNT; k++) {
        float d = 2.0f * BOUNCE_DROP_S * powf(BOUNCE_E, (float)k);
        if (t < d) {
            float h = powf(BOUNCE_E, 2.0f * k);           // the bounce's height
            float v = 2.0f * t / d - 1.0f;
            return length * (1.0f - h * (1.0f - v * v));
        }
        t -= d;
    }
    if (t < BOUNCE_REST_S) {
        return length;
    }
    t -= BOUNCE_REST_S;
    if (t < BOUNCE_LIFT_S) {
        return length * (1.0f - ease(t / BOUNCE_LIFT_S));
    }
    return 0.0f;
}

// A plucked spring: pulled out, let go, ringing down to rest
#define SPRING_PLUCK_S    0.35f
#define SPRING_PERIOD_S   0.7f
#define SPRING_RINGS      6.5f     // ends at the bottom of a swing, so the next pluck starts at rest
#define SPRING_DAMPING    1.2f     // per second
static float stroke_spring(uint32_t millis, float length)
{
    const float ring_s = SPRING_RINGS * SPRING_PERIOD_S;
    float t = fmodf(millis / 1000.0f, SPRING_PLUCK_S + ring_s);
    if (t < SPRING_PLUCK_S) {
        return length * ease(t / SPRING_PLUCK_S);
    }
    t -= SPRING_PLUCK_S;
    // (1 + a t) e^(-a t) starts level, so the ring takes over from the pluck without a kink
    float env = (1.0f + SPRING_DAMPING * t) * expf(-SPRING_DAMPING * t);
    return length * env * (1.0f + cosf(2.0f * (float)M_PI * t / SPRING_PERIOD_S)) / 2.0f;
}

// da-da-DUM: two quick shallow strokes, then a full one
static float stroke_gallop(uint32_t millis, float length)
{
    uint32_t t = millis % 1200u;
    if (t < 250u) {
        return length * cosine_stroke(t / 250.0f, 0.4f);
    }
    if (t < 500u) {
        return length * cosine_stroke((t - 250u) / 250.0f, 0.4f);
    }
    return length * cosine_stroke((t - 500u) / 700.0f, 1.0f);
}

// Long-short, like swung music: a full 0.75 s stroke, then a quicker, shallower one
static float stroke_swing(uint32_t millis, float length)
{
    uint32_t t = millis % 1200u;
    if (t < 750u) {
        return length * cosine_stroke(t / 750.0f, 1.0f);
    }
    return length * cosine_stroke((t - 750u) / 450.0f, 0.7f);
}

// The same full stroke, getting faster: 0.5 to 1.4 strokes a second over 20 s (19 strokes, so it
// ends at rest), then a pause before it starts slow again
#define RAMP_S       20.0f
#define RAMP_FROM_HZ 0.5f
#define RAMP_TO_HZ   1.4f
#define RAMP_PAUSE_S 2.0f
static float stroke_ramp_up(uint32_t millis, float length)
{
    float t = fmodf(millis / 1000.0f, RAMP_S + RAMP_PAUSE_S);
    if (t >= RAMP_S) {
        return 0.0f;
    }
    float phase = RAMP_FROM_HZ * t + (RAMP_TO_HZ - RAMP_FROM_HZ) * t * t / (2.0f * RAMP_S);   // strokes so far
    return length * (1.0f - cosf(2.0f * (float)M_PI * phase)) / 2.0f;
}

// Out, a small fast tremble while held at the deepest point, back, and a short rest
#define FLUTTER_OUT_S     0.6f
#define FLUTTER_HOLD_S    1.25f    // 10 trembles at 8 Hz: ends back at full depth
#define FLUTTER_HZ        8.0f
#define FLUTTER_DEPTH     0.08f    // how far each tremble comes back from full depth
#define FLUTTER_REST_S    0.3f
static float stroke_flutter(uint32_t millis, float length)
{
    float t = fmodf(millis / 1000.0f, 2.0f * FLUTTER_OUT_S + FLUTTER_HOLD_S + FLUTTER_REST_S);
    if (t < FLUTTER_OUT_S) {
        return length * ease(t / FLUTTER_OUT_S);
    }
    t -= FLUTTER_OUT_S;
    if (t < FLUTTER_HOLD_S) {
        return length * (1.0f - FLUTTER_DEPTH * (1.0f - cosf(2.0f * (float)M_PI * FLUTTER_HZ * t)) / 2.0f);
    }
    t -= FLUTTER_HOLD_S;
    if (t < FLUTTER_OUT_S) {
        return length * (1.0f - ease(t / FLUTTER_OUT_S));
    }
    return 0.0f;
}

// Ocean swell: 1.6 s waves whose height builds and falls in sets over 16 s, with a light chop
static float stroke_waves(uint32_t millis, float length)
{
    float t = (millis % 16000u) / 1000.0f;
    float set = (1.0f - cosf(2.0f * (float)M_PI * t / 16.0f)) / 2.0f;
    float height = 0.3f + 0.65f * set * set;
    float swell = (1.0f - cosf(2.0f * (float)M_PI * t / 1.6f)) / 2.0f;
    float chop = 0.05f * (1.0f - cosf(2.0f * (float)M_PI * t / 0.4f)) / 2.0f;
    return length * (height * swell + chop);
}

// A stroke: the offset (0..length) at millis into it. Time is already scaled by the Speed setting.
typedef float (*stroke_fn_t)(uint32_t millis, float length);

typedef struct {
    const char *name;
    stroke_fn_t fn;
} stroke_entry_t;

_Static_assert(STROKE_COUNT <= OM_MAX_STROKES, "the link carries at most OM_MAX_STROKES stroke names");

static const stroke_entry_t s_strokes[STROKE_COUNT] = {
    [STROKE_JACK] = {"Jack", stroke_jack},
    [STROKE_PULSE] = {"Pulse", stroke_pulse},
    [STROKE_PUMP] = {"Pump", stroke_pump},
    [STROKE_CLASSIC] = {"Classic", stroke_classic},
    [STROKE_TEASE] = {"Tease", stroke_tease},
    [STROKE_CRESCENDO] = {"Crescendo", stroke_crescendo},
    [STROKE_SLOW_IN] = {"Slow In", stroke_slow_in},
    [STROKE_HEARTBEAT] = {"Heartbeat", stroke_heartbeat},
    [STROKE_BREATHING] = {"Breathing", stroke_breathing},
    [STROKE_WANDER] = {"Wander", stroke_wander},
    [STROKE_PENDULUM] = {"Pendulum", stroke_pendulum},
    [STROKE_BOUNCE] = {"Bounce", stroke_bounce},
    [STROKE_SPRING] = {"Spring", stroke_spring},
    [STROKE_LORENZ] = {"Lorenz", stroke_lorenz},
    [STROKE_GALLOP] = {"Gallop", stroke_gallop},
    [STROKE_SWING] = {"Swing", stroke_swing},
    [STROKE_RAMP_UP] = {"Ramp Up", stroke_ramp_up},
    [STROKE_FLUTTER] = {"Flutter", stroke_flutter},
    [STROKE_WAVES] = {"Waves", stroke_waves},
};

float stroke_eval(stroke_id_t stroke, uint32_t millis, float length)
{
    if (stroke >= STROKE_COUNT) {
        return 0.0f;
    }
    return s_strokes[stroke].fn(millis, length);
}

const char *stroke_name(stroke_id_t stroke)
{
    return stroke < STROKE_COUNT ? s_strokes[stroke].name : "?";
}
