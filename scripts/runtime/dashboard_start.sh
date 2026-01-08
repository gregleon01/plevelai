#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "${ROOT_DIR}"
export PYTHONPATH="${PYTHONPATH:-}:${ROOT_DIR}/src:${ROOT_DIR}:/mnt/nvme/yolo"
python -m plevelai.dashboard.run "$@"
