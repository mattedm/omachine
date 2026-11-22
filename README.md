# omachine

Firmware for the omachine: a linear-rail machine driven by an ODrive S1, and a handheld haptic
handle that controls it. Both boards are ESP32-S3s, linked over ESP-NOW.

| Folder | Board | Does |
|---|---|---|
| [`controller/`](controller) | ESP32-S3 dev board, CAN to the ODrive S1 | Runs the machine's states (homing, manual, auto strokes), drives the ODrive and keeps the settings |
| [`manipulator/`](manipulator) | ESP32-S3 with a DRV8311 and a gimbal motor | The haptic handle: round display, knob, buttons and menu |
| [`hardware/`](hardware) | | The physical design: the handle's circuit board (KiCad), and the mechanical parts (FreeCAD) to come |

`controller/` and `manipulator/` are each their own ESP-IDF (v6.1) project, with its own README on
building and flashing. In VS Code, open `omachine.code-workspace` rather than the folder.
`omachine_protocol.h` is shared by the two and must stay identical in both.

## Firmware updates

After the first flash over USB, both boards update over the air from the manipulator's
**Update firmware** menu item. It downloads the latest release of this repository
(`manipulator.bin` and `controller.bin`) and relays the controller's image to it over ESP-NOW.
To make a release, tag the repository and run `manipulator/tools/make_release.sh` (see the
script for the `gh release create` line).

## The ODrive

The ODrive S1's configuration is in [`controller/odrive/`](controller/odrive). The manipulator's
**Set up ODrive** menu item writes it to the ODrive over CAN, calibrates the motor (including
anticogging) and saves it.

## License

Copyright 2026 MattEdm. The firmware is GPL-3.0-or-later: see [LICENSE](LICENSE). Some files
carry their own notices; see each project's README. The hardware designs in [`hardware/`](hardware)
are under CERN-OHL-S-2.0: see [hardware/LICENSE](hardware/LICENSE).
