#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="${PYTHONPATH:-}:${PWD}/src:${PWD}:/mnt/nvme/yolo"
python -m plevelai.dashboard.run "$@"
