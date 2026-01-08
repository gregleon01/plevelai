# Arduino UNO R4 DM556 Controller

This directory contains the production firmware for the pan/tilt head when it is driven by DM556 stepper drivers tied to an Arduino UNO R4 WiFi. The sketch consumes JSON commands from the host runtime, performs queued motion with homing, and gates the laser output once both axes are stable.

## Key capabilities
- Parses `move`, `home`, `config`, and `ping` JSON commands over USB serial (115200 baud).
- Maintains a FIFO motion queue with drop-oldest behaviour if detections arrive faster than dispatch.
- Supports homing sequences with normally-closed limit switches (set the limit pin to `0xFF` if absent).
- Laser control honours default pulse/settle timing, confidence thresholds, and per-command overrides.
- DM556-safe pulse generation: 15 microsecond step-high/low timing and a 20 microsecond DIR settle window when direction changes.
- Axis calibration may be tweaked at runtime via `{"cmd":"config","axis":{...}}`; any change invalidates the home state so the controller re-homes cleanly.

## Wiring quick reference
- PSU V+ -> DM556 +V
- PSU COM -> DM556 GND and linked to Arduino GND (shared reference)
- Arduino 5 V -> breadboard red rail feeding both drivers' PUL+, DIR+, (optional) ENA+
- Breadboard blue rail -> common ground return for Arduino GND, PSU COM, and both drivers' PUL-/DIR-
- PAN driver: PUL- -> D2, DIR- -> D3, ENA floating
- TILT driver: PUL- -> D5, DIR- -> D6, ENA floating
- Laser TTL: pin 10 (active-high by default; configurable via `config`)

## Timing highlights
- `STEP_PULSE_HIGH_US` / `STEP_PULSE_LOW_US` = 15 microseconds.
- `DIR_SETUP_DELAY_US` = 20 microseconds before the first pulse after a direction change.
- Default kinematics assume 3200 microsteps per revolution (~8.89 steps/deg); adjust with `axis.pan.steps_per_deg` / `axis.tilt.steps_per_deg` as needed.

## Building and uploading
```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi firmware/uno_r4
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi --port /dev/ttyACM0 firmware/uno_r4
```

Monitor the serial port at 115200 baud for telemetry (`{"status":"telemetry",...}`) and acknowledgements (`{"status":"queued"}`, `{"status":"dispatch"}`, etc.).

## Runtime tips
- Issue a `{"cmd":"home"}` if you connect with homing disabled on the host. The firmware auto-homes once after upload because `g_homeRequested` is `true` at boot.
- Use `{"cmd":"config","axis":{"reset":true}}` to revert steps/deg to defaults; the controller will clear its queue and re-home automatically.
- Run `{"cmd":"motors_check"}` to sweep both axes and fire two laser pulses for a quick hardware sanity check without the Jetson runtime.
- Keep PUL+/DIR+ tied to the UNO's 5 V rail and ensure PSU COM is bonded to Arduino GND so the 15 microsecond pulses reference the same logic ground.

Latest known-good snapshot: `working_sketch_2025-10-15.ino`.
