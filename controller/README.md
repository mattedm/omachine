# omachine-controller

Firmware for the omachine controller: an ESP32-S3 that drives an ODrive S1 over CAN (TWAI) and
links to the [omachine-manipulator](../manipulator) handle over ESP-NOW. It runs the
machine's states (homing, manual, auto strokes) and keeps the Power, Speed, Length and Stroke
settings, which can be saved as start-up defaults from the manipulator's menu.

## Building

ESP-IDF v6.1:

```
idf.py build
idf.py -p /dev/ttyACM0 flash
```

After the first USB flash, updates can come over the air through the manipulator's
**Update firmware** menu item (see omachine-manipulator's `tools/make_release.sh`).

## Strokes

The auto strokes are all in [`main/strokes.c`](main/strokes.c), one function each, with how
to change or add one at the top of [`main/strokes.h`](main/strokes.h). A new stroke's name also
goes in the manipulator's list (`strokes[]` in `manipulator/main/display_task.cpp`), in the same
place. [`docs/strokes.png`](../docs/strokes.png) graphs them all.

## ODrive configuration

The ODrive S1's configuration is in [`odrive/odrive-s1-config.json`](odrive/), with how to
restore it and what the firmware needs from it.

## License

Copyright 2026 MattEdm. This firmware is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version. See [LICENSE](LICENSE).

`main/twai_odrive_control.c` started from Espressif's TWAI example (Apache-2.0) and
`main/serial_manipulator.cpp` from Espressif's USB host CDC-ACM example (CC0-1.0); both keep
Espressif's notices.

### Third-party components

Fetched by the ESP-IDF component manager into `managed_components/`, each under its own license:

| Component | License |
|---|---|
| espressif/usb, usb_host_cdc_acm, usb_host_ch34x_vcp, usb_host_vcp | Apache-2.0 |
| [ESP-IDF](https://github.com/espressif/esp-idf) | Apache-2.0 |
