#!/usr/bin/env bash
set -euo pipefail

# Offline smoke test: runs the runtime against a tiny sample log with a dummy homography.
# No camera, GPU, or serial hardware required.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${ROOT_DIR}/src:${ROOT_DIR}:${PYTHONPATH:-}"

TMP_DIR="$(mktemp -d /tmp/plevelai_smoke.XXXXXX)"
cleanup() { rm -rf "${TMP_DIR}"; }
trap cleanup EXIT

H_FILE="${TMP_DIR}/H_img_to_ground.npy"
CONFIG_FILE="${TMP_DIR}/config.yaml"
LOG_FILE="${TMP_DIR}/detections.log"

python3 - <<PY
import numpy as np, pathlib, yaml, time, json
tmp = pathlib.Path("${TMP_DIR}")

# Identity homography (pixels -> same coordinates) for smoke purposes only.
np.save(tmp / "H_img_to_ground.npy", np.eye(3, dtype=float))

# Minimal config pointing at the temp homography and no serial port.
cfg = {
    "homography_path": str(tmp / "H_img_to_ground.npy"),
    "pan_tilt": {
        "axis_height_m": 0.3,
        "tilt_offset_deg": 90.0,
        "tilt_direction": 1,
        "joint_limits_deg": {"pan": [-180, 180], "tilt": [0, 180]},
    },
    "camera_to_arm": {"rotation_deg": 0.0, "translation_m": [0.0, 0.0]},
    "target_plane_z_m": 0.0,
    "arduino": {"port": None, "baudrate": 115200},
    "min_confidence": 0.5,
    "min_bbox_area_px": 20,
}
with open(tmp / "config.yaml", "w") as fh:
    yaml.safe_dump(cfg, fh)

# Single detection entry for a weed-ish target.
sample = {
    "ts": time.time(),
    "detections": [
        {"u": 640.0, "v": 360.0, "w": 40.0, "h": 50.0, "cls": 1, "conf": 0.9}
    ],
}
with open(tmp / "detections.log", "w") as fh:
    fh.write(json.dumps(sample) + "\n")
PY

echo "Running offline smoke test (dry-run)…"
python3 -m apps.weeder_runtime.runtime \
    --config "${CONFIG_FILE}" \
    --log "${LOG_FILE}" \
    --once \
    --dry-run \
    --verbose

echo "✅ Offline smoke test completed."
