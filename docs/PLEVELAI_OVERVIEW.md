# PlevelAI Overview

## Mission
PlevelAI is the AI-guided pan/tilt laser module for Robotnik, an autonomous weeding robot aimed at making precision weed control affordable for small and medium farms. The module spots weeds with a camera + YOLO detector, aims a pair of NEMA 17 stepper motors, and will eventually fire a laser to zap the target.

## Hardware Snapshot
- **Pan/Tilt Head:** Two NEMA 17 steppers with a pan axis and a tilt axis. (In code these still appear as "joints" because the kinematics module outputs angles; the firmware translates those angles into step counts for the steppers.)
- **Controller:** Arduino UNO R4 WiFi that receives JSON commands over USB serial, drives the stepper drivers, and will gate the laser.
- **Sensing:** Jetson (or laptop) running a CSI/USB camera that feeds YOLO for weed detection.

## Software Flow
1. **Detection:** `apps/yolo/yolo_log_and_stream.py` (invoked via `make run` or `./launch`) captures frames, runs YOLO, draws bounding boxes, and logs detections to `detections.log`.
2. **Target Selection:** Each detection entry contains bounding-box centers, dimensions, class IDs, and confidences.
3. **Ground Projection:** `apps/weeder_runtime/runtime.py` tails the log, projects pixels to ground coordinates through the homography + extrinsics in `configs/robot.yaml`.
4. **Target Prioritisation & Queue:** The runtime filters detections against confidence/area thresholds, keeps a small queue of candidates, and always prefers the weed lowest in the image. Duplicate hits within a configurable ground-plane distance are merged, and stale targets age out automatically.
5. **Pan/Tilt Solve:** `src/plevelai/kinematics/pan_tilt.py` converts ground positions into pan/tilt angles for the head.
6. **Stepper Command & Telemetry:** The runtime streams JSON like `{ "cmd": "move", "joints": {"pan": θ_pan_deg, "tilt": θ_tilt_deg}, ... }` over USB serial. The Arduino UNO R4 WiFi firmware at `firmware/uno_r4/nano_r4.ino` converts degrees into step pulses for the NEMA 17 motors, keeps timing at 15 micros/20 micros for DM556 drivers, and will handle laser gating as interlocks come online. Each dispatch also records telemetry (timestamp, confidence, queue depth) when `--telemetry-log` is enabled.

## Repository Layout (key pieces)
- `src/plevelai/vision/` – camera drivers, calibration data, YOLO wrappers.
- `apps/weeder_runtime/` – the Python runtime that bridges detections to the controller.
- `src/plevelai/control/` – host serial bridge for the UNO controller.
- `firmware/uno_r4/` – UNO R4 DM556 controller firmware.
- `src/plevelai/kinematics/` – inverse kinematics helpers for the pan/tilt rig.
- `web/dashboard/` – dashboard frontend assets.
- `docs/` – setup and calibration guides (this file included).

## Typical Bring-up
1. Place YOLO weights at `models/best.pt` (or point `MODEL=...`).
2. Run `./launch` (or `make run`). This starts the camera + YOLO stream and the runtime; the annotated video is available at `http://<host>:8080/video`.
3. The runtime automatically sends a `home` command once at startup (limit switches should be normally closed). Override with `SKIP_HOME=1 ./launch` if you need to bypass homing, or `HOME_ONCE=1` to force an extra home.
4. Ensure the Arduino UNO R4 WiFi shows up at `/dev/ttyACM0` (override with `SERIAL_PORT=...` if needed).
5. Verify that the steppers track detections. Use `DRY_RUN=1 ./launch` to dry-test without actuating hardware. Set `TELEMETRY_LOG=telemetry.csv` if you want a CSV trail of every dispatch.

## Next Milestones
- Extend the Arduino UNO R4 WiFi firmware against the DM556 hardware: add soft-limit logic, richer telemetry, and safety interlocks as the system matures.
- Integrate the laser safety controls (interlocks, watchdogs, timing gates).
- Fold the module into the larger Robotnik autonomy stack (navigation, task scheduling, farm UX).
