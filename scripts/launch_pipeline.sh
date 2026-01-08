#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_PATH="${LOG:-${ROOT_DIR}/detections.log}"
CONFIG_PATH="${CONFIG:-${ROOT_DIR}/configs/robot.yaml}"
SERIAL_PORT="${SERIAL_PORT:-}"
BAUDRATE="${BAUDRATE:-}"
DRY_RUN="${DRY_RUN:-0}"
export PYTHONPATH="${ROOT_DIR}/src:${ROOT_DIR}:${PYTHONPATH:-}"

mkdir -p "$(dirname "${LOG_PATH}")"
: >"${LOG_PATH}"

cleanup() {
    local status=$?
    if [[ -n "${YOLO_PID:-}" ]]; then
        kill "${YOLO_PID}" 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    return ${status}
}
trap cleanup EXIT INT TERM

YOLO_ENV=(
    MODEL="${MODEL:-${ROOT_DIR}/models/best.pt}"
    CAM="${CAM:-csi}"
    SENSOR_ID="${SENSOR_ID:-0}"
    IMGSZ="${IMGSZ:-640}"
    CONF="${CONF:-0.25}"
    LOG="${LOG_PATH}"
    PORT="${PORT:-8080}"
)

( export "${YOLO_ENV[@]}"; python3 -m apps.yolo.yolo_log_and_stream ) &
YOLO_PID=$!

RUNTIME_CMD=(python3 -m apps.weeder_runtime.runtime --config "${CONFIG_PATH}" --log "${LOG_PATH}")
if [[ -n "${SERIAL_PORT}" ]]; then
    RUNTIME_CMD+=(--serial-port "${SERIAL_PORT}")
fi
if [[ -n "${BAUDRATE}" ]]; then
    RUNTIME_CMD+=(--baudrate "${BAUDRATE}")
fi
if [[ "${DRY_RUN}" == "1" ]]; then
    RUNTIME_CMD+=(--dry-run)
fi
if [[ -n "${QUEUE_LEN:-}" ]]; then
    RUNTIME_CMD+=(--queue-len "${QUEUE_LEN}")
fi
if [[ -n "${QUEUE_STALE_SEC:-}" ]]; then
    RUNTIME_CMD+=(--queue-stale-sec "${QUEUE_STALE_SEC}")
fi
if [[ -n "${QUEUE_MERGE_DIST:-}" ]]; then
    RUNTIME_CMD+=(--queue-merge-dist "${QUEUE_MERGE_DIST}")
fi
if [[ -n "${TELEMETRY_LOG:-}" ]]; then
    RUNTIME_CMD+=(--telemetry-log "${TELEMETRY_LOG}")
fi
if [[ "${HOME_ONCE:-0}" == "1" ]]; then
    RUNTIME_CMD+=(--home-once)
fi
if [[ "${SKIP_HOME:-0}" == "1" ]]; then
    RUNTIME_CMD+=(--skip-home)
fi

"${RUNTIME_CMD[@]}"
