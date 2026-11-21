# omachine-manipulator

Firmware for the omachine manipulator: the handheld haptic handle and round display that drive
[omachine-controller](../controller) over ESP-NOW. It runs on an ESP32-S3 with a DRV8311
motor driver and a gimbal motor under SimpleFOC, a GC9A01 display, a rotary knob and a 1S battery.

## Building

ESP-IDF v6.1:

```
idf.py build
idf.py -p /dev/ttyACM0 flash
```

`tools/make_release.sh` builds this project and [the controller](../controller) into `release/`
for a GitHub release; the menu's **Update firmware** installs them over the air. It downloads
from this repository's latest release (`OM_UPDATE_URL`, under "omachine manipulator" in
`idf.py menuconfig`; point it at your own repository's releases in a fork).

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
