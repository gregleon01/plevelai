# IK + Control Pipeline

This is the plan for taking YOLO detections and driving the steppers via joint angles.

## Flow
1. YOLO app logs detections to `detections.log` (`apps/yolo/yolo_to_log.py` / `launch`).
2. `apps/weeder_runtime/runtime.py` tails the log, converts pixel centers via homography, merges duplicates, queues candidates, and solves pan/tilt angles using `plevelai.kinematics.pan_tilt.PanTiltRig`.
3. Joint angles are streamed as JSON lines over serial using `control.host.serial_bridge.ArduinoBridge`.
4. Arduino UNO R4 WiFi firmware consumes `{ "cmd": "move", "joints": {"joint_1": θ1_deg, "joint_2": θ2_deg}, ... }` and performs motion + laser gating.

## What you must measure / fill
- `configs/robot.yaml`
  - `pan_tilt.axis_height_m`: ground-to-tilt-axis height (meters).
  - `pan_tilt.tilt_offset_deg`: add/subtract so 90° aims straight down (adjust for mechanics).
  - `pan_tilt.tilt_direction`: `1` if positive tilt points downwards, `-1` to flip.
  - `pan_tilt.joint_limits_deg`: mechanical safe range for `pan` (±) and `tilt` (0–180° hemisphere).
  - `camera_to_arm.rotation_deg` / `translation_m`: rotation + XY offset from camera ground frame to arm base frame.
  - `arduino.port`: serial device path.
  - `min_confidence` / `min_bbox_area_px`: detection filters.
- `calibration/H_img_to_ground.npy`: run the steps in `docs/CALIBRATION_GUIDE.md`.

## Arduino firmware outline
- Use `Serial.begin(baudrate)` matching `configs/robot.yaml`.
- Parse incoming JSON lines (`cmd == "move"`). Lightweight option: use `ArduinoJson` or manual parsing.
- For each joint (`pan`, `tilt`):
  - Convert commanded degrees to motor steps: `steps = deg * steps_per_deg[joint]`.
  - Run trapezoidal move profile (or reuse existing stepper lib) to hit target without overshoot.
  - Add homing + soft-limit checks before enabling motion.
- Expect normally-closed limit switches for homing; once triggered, back off a few steps so the switch re-opens before entering `Idle`.
- Once arm is in position, enable the laser gate if confidence exceeds threshold (value is in the JSON payload).
- Return status lines back over serial (optional, but helps debugging): `{"ok":true,"joints":[...],"ts":123.4}`.

## Runtime queue, homing, and telemetry
- The host runtime issues a `home` command on startup unless `--skip-home` (or `SKIP_HOME=1`) is provided. Use `--home-once` to force an extra homing cycle after reconnects.
- Queue controls: `--queue-len`, `--queue-stale-sec`, `--queue-merge-dist`. Detections within the merge distance are treated as duplicates.
- Set `--telemetry-log <path>` (or `TELEMETRY_LOG=...`) to write a CSV containing `sent_ts,det_ts,confidence,pan_deg,tilt_deg,ground_x,ground_y,image_v,queue_after,target_age_s` for each dispatch.

## Running it today
```bash
# 1. Launch YOLO (existing flow)
launch  # or `make run`

# 2. In another shell, dry-run the IK/controller loop
python -m apps.weeder_runtime.runtime --dry-run --verbose

# 3. Once geometry + firmware are ready
python -m apps.weeder_runtime.runtime --serial-port /dev/ttyACM0
```

## Next steps
- Validate homography by projecting a grid of test points.
- Tune the arm geometry until `--dry-run` outputs sensible angles.
- Extend the Arduino UNO R4 WiFi firmware (in `firmware/uno_r4`) with additional safeties and limit handling as the mechanical design firms up.
- Add motion queueing / time synchronization if you need continuous scanning instead of single-shot moves.
