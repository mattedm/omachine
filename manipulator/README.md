# omachine manipulator

Firmware for the omachine's manipulator: the handheld haptic handle that drives the
[controller](../controller) over ESP-NOW. A gimbal motor under SimpleFOC gives the handle its
feel, a round display shows the settings, and a knob and an illuminated button control the
machine. It runs from a 1S battery or USB. See the [top-level README](../README.md) for the whole
machine.

## Hardware

The board is in [`../hardware/pcb/manipulator/`](../hardware/pcb/manipulator) and the encoder
boards in [`../hardware/pcb/encoder-breakouts/`](../hardware/pcb/encoder-breakouts). The pins are
in [`main/board_pins.h`](main/board_pins.h).

| | |
|---|---|
| Processor | ESP32-S3-MINI-1 |
| Motor | gimbal motor on a DRV8311 driver, SimpleFOC (estimated current), about 19.5 kHz loop on core 1 |
| Angle sensor | AS5048A or MT6701 magnetic encoder board on J5 (`ENCODER_MT6701` in `main/motor_task.h`) |
| Display | 240 x 240 round GC9A01 (LovyanGFX) |
| Inputs | EC11 rotary encoder with push switch (the knob); illuminated main button (RGB) |
| Power | 1S battery through a BQ24075 charger, or USB |

## Building

ESP-IDF v6.1, from this folder:

```
idf.py build
idf.py -p /dev/ttyACM0 flash
```

`main/main.cpp` has three debugging builds, all off normally: `ENCODER_TEST` (the knob),
`OPEN_LOOP_TEST` (spins the motor without its sensor) and `SENSOR_TEST` (the magnetic encoder).

## Using it

- **Power:** hold the main button for 3 s to turn it on from the battery, and 3 s to turn it off
  (a countdown shows). On USB it shows the charging screen when off.
- **Main button:** switches between Manual and Auto.
- **Knob:** pressing it steps through Speed, Length and Stroke; turning it changes the
  highlighted one.
- **Menu:** hold the knob for 3 s. It has Power (the ODrive's current limit), Save settings (as
  the start-up defaults), Calibrate motor, Set up ODrive, Wi-Fi setup, Update firmware and Exit.
- **Faults:** if the ODrive stops, the screen shows the cause, what to check, any other errors,
  the supply voltage, and "Retrying..." or "Press to retry".

## Firmware updates

**Wi-Fi setup** starts an `omachine-setup` network; join it from a phone and enter your Wi-Fi's
name and password. **Update firmware** then downloads both boards' images from this repository's
latest release, installs its own and passes the controller's on over ESP-NOW. The address is
`OM_UPDATE_URL`, under "omachine manipulator" in `idf.py menuconfig`; in a fork, point it at your
own releases.

To make a release, commit and tag the repository (e.g. `git tag v1.4`), then, with ESP-IDF
activated, run [`tools/make_release.sh`](tools/make_release.sh). It builds this project and the
controller into `release/`, and the version both boards compare is the tag. Attach both files to
a GitHub release (the script shows the `gh release create` line).

## How it works

| File | Does |
|---|---|
| `main/motor_task.cpp` | The haptic motor: finds the handle's end stops at start-up, then follows the ODrive's position |
| `main/link_task.cpp` | The ESP-NOW link: replies to the controller's polls with the handle's position and the inputs, and keeps the settings, state, fault and stroke names it sends |
| `main/display_task.cpp` | The screens and the menu |
| `main/inputs.cpp` | The knob and buttons, sampled every 1 ms |
| `main/power.cpp` | Power on and off, the battery and charging, and the power-off hold |
| `main/firmware_update.cpp` | Wi-Fi setup and over-the-air updates for both boards |
| `main/omachine_protocol.h` | The link's frames. It must be identical to the controller's copy |

## License

Copyright 2026 MattEdm. This firmware is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version. See [LICENSE](LICENSE).

`main/rotary.cpp` and `main/rotary.h` are a port of Ben Buxton's rotary encoder library
(Copyright 2011 Ben Buxton), licensed under the GNU GPL version 3.

### Third-party components

Fetched by the ESP-IDF component manager into `managed_components/`, each under its own license:

| Component | License |
|---|---|
| [Arduino-FOC (SimpleFOC)](https://github.com/simplefoc/Arduino-FOC), Copyright 2020 Antun Skuric | MIT |
| espressif/esp_simplefoc, espressif/i2c_bus, espressif/cmake_utilities | Apache-2.0 |
| espressif/iqmath, Copyright 2023 Texas Instruments | BSD-3-Clause |
| [LovyanGFX](https://github.com/lovyan03/LovyanGFX) (with parts from Adafruit_GFX and TFT_eSPI) | FreeBSD / BSD / MIT |
| [ESP-IDF](https://github.com/espressif/esp-idf) | Apache-2.0 |

The fonts in `main/fonts/` are bitmaps of Google's Roboto Light, made with
[squix.ch's font converter](http://oleddisplay.squix.ch/); Roboto is licensed under the
SIL Open Font License 1.1 (earlier releases: Apache-2.0).
