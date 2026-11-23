# omachine controller

Firmware for the omachine's controller: an ESP32-S3 that drives the ODrive S1 over CAN and links
to the [manipulator](../manipulator) (the handheld handle) over ESP-NOW. It runs the machine's
states, homes the carriage, follows the handle, plays the auto strokes and keeps the settings.
See the [top-level README](../README.md) for the whole machine.

## Hardware

| | |
|---|---|
| Board | ESP32-S3 dev board |
| CAN (TWAI) to the ODrive | TX GPIO 7, RX GPIO 15, through a CAN transceiver; 1 Mbit/s, ODrive node 0 |
| Status LED | the dev board's RGB LED (WS2812, GPIO 48): the machine's state in the same colours as the handle's button (blue starting up, yellow syncing, green Manual, red Auto), blinking red on an ODrive fault |

## Building

ESP-IDF v6.1, from this folder:

```
idf.py build
idf.py -p /dev/ttyACM0 flash
```

After the first USB flash, it updates over the air with the handle: the handle's
**Update firmware** downloads both boards' images and passes this one on over ESP-NOW. Releases
are made with [`../manipulator/tools/make_release.sh`](../manipulator/tools/make_release.sh).

## How it works

| File | Does |
|---|---|
| `main/twai_odrive_control.c` | The ODrive over CAN: start-up and homing, then a position setpoint for every handle reading (about 200 a second). Manual (with the soft carriage when the handle is free), Auto strokes, the Power current limit, and fault detection and retries |
| `main/machine_state.c` | The machine's states (Initializing, Homing, Syncing, Manual, Starting, Auto, Link lost, Setting up, ODrive fault), the settings (Power, Speed, Length, Stroke) and saving them to flash |
| `main/strokes.c`, `strokes.h` | The auto strokes |
| `main/espnow_manipulator.cpp` | The link to the handle: a frame every 5 ms with the ODrive's position, the settings, the state, any fault and one stroke name |
| `main/odrive_setup.c` | The menu's **Set up ODrive**: writes the configuration over CAN, calibrates and saves |
| `main/firmware_receiver.c` | Receives an update from the handle and installs it |
| `main/status_led.c` | The RGB LED |
| `main/omachine_protocol.h` | The link's frames. It must be identical to the manipulator's copy |

The main settings are `#define`s at the top of `twai_odrive_control.c`: `MAX_CURRENT_A` (Power
100%, 40 A), `VEL_LIMIT` (15 turns/s), the `HOMING_*` and the `COMPLIANCE`/`SOFT_*` values.

### Homing

At start-up, once the ODrive is in closed loop, the carriage drives at 1 turn/s with an 8 A limit
into the contracted end of the rail and then the extended end, stopping where it stops moving.
The stroke's span is set 0.1 turns inside each end, and the carriage waits at the extended end
(fully retracted). If no end turns up within 40 turns (carriage off the rail, or belt off), or
the ends are under a turn apart, it falls back to a one-turn span around where it is. The log
reports what it found.

### Faults

If the ODrive leaves closed loop or reports an error, the controller asks it for its active
errors and the reason it disarmed (Get_Error), and sends them to the handle with the supply
voltage. It clears the errors and retries 3 times, 2 s apart; after that, the handle's main
button retries.

## Strokes

The auto strokes are all in [`main/strokes.c`](main/strokes.c), one function each, and the top of
[`main/strokes.h`](main/strokes.h) explains how to change one or add one. The handle gets the
names from the controller, so only the controller changes. Strokes can be written at any pace:
each is played at its own rate (`stroke_pace()`) so that all of them reach the same top speed at a
given Speed setting.

[`docs/strokes.png`](../docs/strokes.png) graphs them all as the machine plays them. After
changing the strokes, redraw it with `python3 tools/plot_strokes.py` (it needs gcc, numpy and
matplotlib; it compiles `main/strokes.c` for your computer, so new strokes appear by themselves).

Pendulum and Lorenz play back tables made from simulations by
[`tools/gen_loop_strokes.py`](tools/gen_loop_strokes.py).

## ODrive configuration

The ODrive S1's configuration is in [`odrive/odrive-s1-config.json`](odrive/), with how to
restore it and what the firmware needs from it. After changing it, run
`python3 tools/gen_odrive_setup.py` to update the table that **Set up ODrive** writes.

## License

Copyright 2026 MattEdm. This firmware is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version. See [LICENSE](LICENSE).

`main/twai_odrive_control.c` started from Espressif's TWAI example (Apache-2.0) and
`main/serial_manipulator.cpp` from Espressif's USB host CDC-ACM example (CC0-1.0); both keep
Espressif's notices. (`serial_manipulator.cpp`, an earlier wired link to the handle, isn't built.)

### Third-party components

Fetched by the ESP-IDF component manager into `managed_components/`, each under its own license:

| Component | License |
|---|---|
| espressif/usb, usb_host_cdc_acm, usb_host_ch34x_vcp, usb_host_vcp | Apache-2.0 |
| [ESP-IDF](https://github.com/espressif/esp-idf) | Apache-2.0 |
