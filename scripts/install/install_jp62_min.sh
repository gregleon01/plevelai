#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y python3-opencv v4l-utils git
python3 -m pip install --user --upgrade pip wheel setuptools
python3 -m pip install --user "numpy==1.26.4" "ultralytics<9" "onnx>=1.12.0,<1.18.0" "onnxslim>=0.1.65" "protobuf<4"
echo "✅ JetPack 6.2 minimal deps installed."
