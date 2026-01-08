#!/usr/bin/env bash
set -euo pipefail
PT="${1:-models/best.pt}"
python3 - <<PY
from ultralytics import YOLO
m = YOLO("${PT}")
f = m.export(format="engine", imgsz=640, half=True, device=0, simplify=True, opset=19, workspace=4)
print("✅ Exported:", f)
PY
