#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 MattEdm
# SPDX-License-Identifier: GPL-3.0-or-later
"""Draw every auto stroke into docs/strokes.png, one graph each.

It compiles the controller's own main/strokes.c for this computer, so the graphs are exactly what
the firmware plays: each stroke's position over 30 s at Speed 1x (the Speed setting 19), with its
pacing (stroke_pace()), as a share of the stroke's length. New strokes appear by themselves.

It also prints each stroke's own top speed and its pace factor (shown in its graph's title too),
and suggests STROKE_TOP_SPEED for main/strokes.h: the median of the strokes' top speeds, which
keeps the middle stroke playing as written. Change it there if the suggestion differs.

Needs a C compiler (gcc or cc) and Python with numpy and matplotlib. Run from anywhere:

    python3 controller/tools/plot_strokes.py
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

CONTROLLER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(CONTROLLER, 'main')
OUT = os.path.join(os.path.dirname(CONTROLLER), 'docs', 'strokes.png')
SECONDS = 30
COLUMNS = 4

# Prints "TOP,<STROKE_TOP_SPEED>", then each stroke as "name,pace,p0,p1,...": its pace factor, then
# its position (0..1) every millisecond of real time at Speed 1x, played at its pace, as the
# controller does.
HARNESS = r'''
#include <stdio.h>
#include "strokes.h"
int main(void)
{
    strokes_init();
    printf("TOP,%.4f\n", (double)STROKE_TOP_SPEED);
    for (int i = 0; i < STROKE_COUNT; i++) {
        float pace = stroke_pace((stroke_id_t)i);
        printf("%s,%.6f", stroke_name((stroke_id_t)i), (double)pace);
        for (unsigned ms = 0; ms <= SECONDS_MS; ms++) {
            printf(",%.4f", stroke_eval((stroke_id_t)i, (uint32_t)(ms * pace), 1.0f));
        }
        printf("\n");
    }
    return 0;
}
'''


def sample_strokes():
    cc = shutil.which('gcc') or shutil.which('cc')
    if cc is None:
        sys.exit('No C compiler found: install gcc (it compiles main/strokes.c to sample the strokes).')
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'harness.c')
        exe = os.path.join(tmp, 'harness')
        with open(src, 'w') as f:
            f.write(HARNESS)
        subprocess.run([cc, '-O2', f'-DSECONDS_MS={SECONDS * 1000}', '-I', MAIN, '-o', exe, src,
                        os.path.join(MAIN, 'strokes.c'), '-lm'], check=True)
        out = subprocess.run([exe], check=True, capture_output=True, text=True).stdout
    lines = out.splitlines()
    top = float(lines[0].split(',')[1])
    strokes = []
    for line in lines[1:]:
        name, pace, *values = line.split(',')
        strokes.append((name, float(pace), np.array(values, dtype=float)))
    return top, strokes


def report(top, strokes):
    """Each stroke's own top speed and pace, and the suggested STROKE_TOP_SPEED."""
    print(f"{'Stroke':<16}{'own top speed':>16}{'pace':>8}")
    own = []
    for name, pace, _ in strokes:
        speed = top / pace   # pace = STROKE_TOP_SPEED / the stroke's own top speed
        own.append(speed)
        print(f'{name:<16}{speed:>10.2f} len/s{pace:>8.2f}x')
    suggested = float(np.median(own))
    print(f'\nSTROKE_TOP_SPEED is {top:g} lengths/s (main/strokes.h); '
          f'the median of the strokes is {suggested:.2f}.')
    if abs(suggested - top) > 0.05 * top:
        print(f'Suggested: #define STROKE_TOP_SPEED {suggested:.1f}f  '
              '(so the middle stroke plays as written; strokes then run at the same top speed as each other either way)')
    else:
        print('That matches: no change needed.')


def plot(top, strokes):
    ink, ink2, grid, blue, paper = '#0b0b0b', '#52514e', '#e4e3df', '#2a78d6', '#fcfcfb'
    rows = math.ceil(len(strokes) / COLUMNS)
    fig, axs = plt.subplots(rows, COLUMNS, figsize=(32, 5.6 * rows), sharex=True, sharey=True,
                            facecolor=paper, squeeze=False)
    t = np.arange(SECONDS * 1000 + 1) / 1000.0
    for ax, (name, pace, pos) in zip(axs.flat, strokes):
        ax.set_facecolor(paper)
        ax.plot(t, pos * 100, color=blue, lw=1.6)
        ax.set_title(name, loc='left', color=ink, fontsize=22, fontweight='bold')
        ax.set_title(f'pace \u00d7{pace:.2f}', loc='right', color=ink2, fontsize=18)
        ax.set_xlim(0, SECONDS)
        ax.set_ylim(-3, 103)
        ax.set_yticks([0, 50, 100])
        ax.set_xticks(range(0, SECONDS + 1, 5))
        ax.grid(color=grid, lw=0.6)
        ax.tick_params(colors=ink2, labelsize=16, length=0)
        for spine in ax.spines.values():
            spine.set_visible(False)
    for ax in list(axs.flat)[len(strokes):]:
        ax.axis('off')
    # time labels under the last graph in each column
    for col in range(COLUMNS):
        filled = [r for r in range(rows) if r * COLUMNS + col < len(strokes)]
        if filled:
            ax = axs[filled[-1], col]
            ax.tick_params(labelbottom=True)
            ax.set_xlabel('time (s)', color=ink2, fontsize=16)
    for ax in axs[:, 0]:
        ax.set_ylabel('% of stroke length', color=ink2, fontsize=16)
    fig.suptitle(f'omachine strokes: position over 30 s at Speed 1x, paced to a top speed of '
                 f'{top:g} stroke lengths/s', x=0.06, ha='left',
                 color=ink, fontsize=30, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print(f'{OUT}: {len(strokes)} strokes')


if __name__ == '__main__':
    top, strokes = sample_strokes()
    report(top, strokes)
    plot(top, strokes)
