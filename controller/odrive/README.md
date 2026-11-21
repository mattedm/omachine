# ODrive S1 configuration

`odrive-s1-config.json` is the ODrive S1's configuration for this machine, saved with
`odrivetool backup-config` (it uses odrivetool's format, with bare `Infinity` and `NaN`, which
strict JSON parsers reject).

To load it onto an ODrive:

```
odrivetool restore-config odrive-s1-config.json
```

If your odrivetool version doesn't save it to the ODrive for you, run
`odrv0.save_configuration()` in `odrivetool` afterwards. To save a fresh copy after changing
settings:

```
odrivetool backup-config odrive-s1-config.json
```

## Setting the ODrive up from the manipulator

The manipulator's menu item **Set up ODrive** has the controller configure the ODrive over CAN.
Take the motor off the linear rail first: the anticogging calibration turns it about 40 turns
each way. It takes about 12 minutes:

1. **Checking:** the ODrive's firmware and hardware must match the endpoint file below, and its
   CAN baud rate and node ID must read back as the controller expects.
2. **Configuring:** every setting in `odrive-s1-config.json` is written and read back. The CAN
   identity settings and the few values that don't fit a CAN frame are left as they are.
3. **Calibrating:** the motor and encoder calibration (resistance, inductance, encoder offset).
4. **Anticogging:** the anticogging calibration, with stiff gains, watched for the motor stalling.
5. **Saving:** the operating gains and control mode go back from the file, then the ODrive saves
   and reboots.

Nothing is saved until the last step, so stopping it from the menu, or any failure, reboots the
ODrive without saving and it keeps its old settings. The controller restarts afterwards, whatever
the result.

The settings come from `main/odrive_setup_table.h`, which `tools/gen_odrive_setup.py` generates
from `odrive-s1-config.json` and `odrive-s1-endpoints-0.6.11.json`. After changing the config
file, regenerate it from the repository root:

```
python3 tools/gen_odrive_setup.py
```

Endpoint IDs change between ODrive firmware versions. The endpoint file was flattened from the
device descriptor that odrivetool caches (`~/.cache/odrivetool/device_json/`). After an ODrive
firmware update, make a new one for that version (and point the generator at it), or the setup
stops at **Checking**.

## What the firmware relies on

`main/twai_odrive_control.c` expects these settings:

- CAN at 1 Mbit/s (`can.config.baud_rate`), node ID 0 (`axis0.config.can.node_id`)
- Encoder estimates every 5 ms, heartbeats every 1000 ms and bus voltage every 2000 ms
  (`axis0.config.can.*_msg_rate_ms`)
- Position control (`control_mode` 3) with the input filter (`input_mode` 3)

## Specific to this motor

The motor's measured phase resistance and inductance and the encoder's commutation offset
(`axis0.commutation_mapper.config.offset`) come from calibrating this motor on this ODrive.
The anticogging map isn't in the file, only its settings. On a different motor or ODrive, run
**Set up ODrive** from the manipulator's menu, which calibrates all of these afresh.
