# omachine hardware

The physical design of the omachine: the circuit board (KiCad) and the mechanical parts (FreeCAD). The firmware is in [`controller/`](../controller) and [`manipulator/`](../manipulator).

| Folder | What | Tool |
|---|---|---|
| [`pcb/manipulator/`](pcb/manipulator) | The handle's board: ESP32-S3-MINI-1, DRV8311 gimbal motor driver, 1S battery charger, connectors for the display, encoder, knob and button | KiCad 9 |
| [`pcb/encoder-breakouts/`](pcb/encoder-breakouts) | The handle's two small boards, on one breakaway panel: the MT6701 magnetic encoder (the handle motor's angle) and the EC11 rotary encoder (the knob) | KiCad 9 |
| [`cad/manipulator/`](cad/manipulator) | The handle: `manipulator.FCStd` | FreeCAD 1.1 |
| [`cad/machine/`](cad/machine) | The main machine: `omachine.FCStd` | FreeCAD 1.1 |

The FreeCAD files are stored with [Git LFS](https://git-lfs.com/). Install it (and run
`git lfs install` once) before cloning, or the `.FCStd` files arrive as small text pointers
instead of models; in a clone made without it, `git lfs pull` fetches them.

## The manipulator board

Open `pcb/manipulator/omachine-manipulator.kicad_pro` in KiCad 9. Its own footprint (USB-C) and
symbols (`jlcpcb.kicad_sym`) are in the project; nearly all 3D models are embedded in the board file.

`pcb/manipulator/production/` has the files to order the current revision from JLCPCB, made with
the Fabrication Toolkit plugin: `omachine_Manipulator_Board_1.2.zip` (Gerbers and drill files),
`bom.csv` and `positions.csv` (for assembly). After changing the board, make them again from the
plugin, so they match.

### Revision 1.2

- R19 and R21 (the main button's sense transistor, Q1) are 10k, not 100k. With 100k, a few
  microamps of leakage from D1 when the board was warm turned Q1 on, and the button read as held.
- Named the omachine manipulator board (was a DRV8311 breakout), with the source link
  github.com/mattedm/omachine on the back silkscreen.

## The encoder boards

Open `pcb/encoder-breakouts/encoder-breakouts.kicad_pro`. The two boards are separate sheets
(`encoder.kicad_sch`, `rotary.kicad_sch`) laid out on one panel with a breakaway rail, and are
ordered together from `pcb/encoder-breakouts/production/` (revision 1.0). They plug into the
manipulator board's ENCODER and ROTARY connectors.

## License

Copyright 2026 MattEdm. The hardware designs in this folder are licensed under the CERN Open
Hardware Licence Version 2 - Strongly Reciprocal (CERN-OHL-S-2.0): see [LICENSE](LICENSE). The
source location is https://github.com/mattedm/omachine. The firmware is under the GPL; see the
top-level [README](../README.md).
