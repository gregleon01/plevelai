#!/usr/bin/env bash
set -euo pipefail

# Quickstart: from fresh clone to live window on Jetson.
# Usage:
#   ./scripts/quickstart.sh [optional_model_path]
# Env:
#   CAM=csi|usb   (default csi)
#   USB_INDEX=0
#   SHOW=1        (1=window on Jetson monitor; 0=headless)
#   RESTART_ARGUS=0  (set 1 to auto restart nvargus-daemon)

MODEL_ARG="${1:-}"
CAM="${CAM:-csi}"
USB_INDEX="${USB_INDEX:-0}"
SHOW="${SHOW:-1}"
RESTART_ARGUS="${RESTART_ARGUS:-0}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 1) Detect Jetson
if [[ ! -f /etc/nv_tegra_release ]]; then
  echo "❌ This quickstart is intended for NVIDIA Jetson (JetPack)."
  echo "   You can still run the app if deps are installed:"
  echo "   cd apps/yolo_live && ./run_yolo_live.sh ../../models/best.pt"
  exit 1
fi

# 2) Deps check (python, cv2, numpy, ultralytics). Install if missing.
NEED_INSTALL=0
python3 - <<'PY' >/dev/null 2>&1 || NEED_INSTALL=1
import cv2, numpy
from ultralytics import YOLO
PY

if [[ "$NEED_INSTALL" == "1" ]]; then
  echo "ℹ️ Installing minimal dependencies (JetPack 6.2)…"
  "${ROOT_DIR}/scripts/install_jp62_min.sh"
fi

# 3) Pick a model (priority: CLI arg → repo → common NVMe paths)
choose_model() {
  local c
  for c in \
    "${MODEL_ARG}" \
    "${ROOT_DIR}/models/best.engine" \
    "${ROOT_DIR}/models/best.pt" \
    "/ssd/yolo/best.engine" "/ssd/yolo/best_1.pt" "/ssd/yolo/best.pt" \
    "/mnt/nvme/yolo/best.engine" "/mnt/nvme/yolo/best_1.pt" "/mnt/nvme/yolo/best.pt"
  do
    [[ -n "${c}" && -f "${c}" ]] && { echo "${c}"; return; }
  done
  echo ""
}
MODEL="$(choose_model)"

if [[ -z "${MODEL}" ]]; then
  echo "❌ No model found."
  echo "Place weights at one of:"
  echo "  - ${ROOT_DIR}/models/best.pt  (recommended symlink/copy)"
  echo "  - /ssd/yolo/best_1.pt or /ssd/yolo/best.pt"
  echo "  - /mnt/nvme/yolo/best_1.pt or /mnt/nvme/yolo/best.pt"
  echo "Or pass a path:  ./scripts/quickstart.sh /path/to/your.pt"
  exit 1
fi

# 4) Display + Argus housekeeping
if [[ "${SHOW}" == "1" ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  command -v xhost >/dev/null 2>&1 && xhost +SI:localuser:"$USER" >/dev/null 2>&1 || true
fi
if [[ "${CAM}" == "csi" && "${RESTART_ARGUS}" == "1" ]]; then
  sudo systemctl restart nvargus-daemon || true
fi

# 5) Run the app
echo "▶️  Running YOLO live | CAM=${CAM} USB_INDEX=${USB_INDEX} SHOW=${SHOW}"
cd "${ROOT_DIR}/apps/yolo_live"
CAM="${CAM}" USB_INDEX="${USB_INDEX}" SHOW="${SHOW}" ./run_yolo_live.sh "${MODEL}"
