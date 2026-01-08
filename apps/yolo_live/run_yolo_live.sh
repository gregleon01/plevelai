#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./run_yolo_live.sh [MODEL_PATH]
# Env (optional):
#   CAM=csi|usb   (default csi)
#   USB_INDEX=0   (when CAM=usb)
#   W=1280 H=720 FPS=30 IMGSZ=640 CONF=0.25 SHOW=1

MODEL="${1:-../../models/best.pt}"
W="${W:-1280}"; H="${H:-720}"; FPS="${FPS:-30}"
IMGSZ="${IMGSZ:-640}"; CONF="${CONF:-0.25}"
CAM="${CAM:-csi}"           # csi or usb
USB_INDEX="${USB_INDEX:-0}" # for CAM=usb
SHOW="${SHOW:-1}"           # 1=show window, 0=headless

python3 - <<PY
import os, sys, time, cv2
from ultralytics import YOLO

model_path = "${MODEL}"
if not os.path.exists(model_path):
    sys.exit(f"❌ Model not found: {model_path}")

# Prefer TensorRT engine if next to the .pt
if model_path.endswith(".pt"):
    eng = os.path.splitext(model_path)[0] + ".engine"
    if os.path.exists(eng):
        print(f"🔁 Using engine: {eng}")
        model_path = eng

m = YOLO(model_path)

W, H, FPS = int("${W}"), int("${H}"), int("${FPS}")
IMGSZ, CONF = int("${IMGSZ}"), float("${CONF}")
CAM = "${CAM}"
USB_INDEX = int("${USB_INDEX}")
SHOW = int("${SHOW}") == 1

# Open camera
if CAM == "csi":
    gst = ("nvarguscamerasrc ! "
           f"video/x-raw(memory:NVMM), width={W}, height={H}, framerate={FPS}/1 ! "
           "nvvidconv ! video/x-raw, format=BGRx ! "
           "videoconvert ! video/x-raw, format=BGR ! appsink")
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
else:
    cap = cv2.VideoCapture(USB_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    sys.exit("❌ Failed to open camera (for CSI, try: sudo systemctl restart nvargus-daemon)")

# UI window
win = "YOLO (q to quit)"
if SHOW:
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, W, H)
    except Exception:
        print("⚠️ Could not create window; running headless.")
        SHOW = False

print(f"==> Capture: {W}x{H}@{FPS} | imgsz={IMGSZ} conf={CONF} | CAM={CAM}")
t0 = time.time(); n = 0
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("⚠️ Empty frame; stopping.")
            break
        r = m.predict(source=frame, device=0, imgsz=IMGSZ, conf=CONF, verbose=False)
        out = r[0].plot()
        if SHOW:
            cv2.imshow(win, out)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
        n += 1
finally:
    cap.release()
    if SHOW:
        cv2.destroyAllWindows()

dt = time.time() - t0
print(f"✅ Done. Frames={n}, Avg FPS={n/dt:.2f}" if dt > 0 and n > 0 else "✅ Done.")
PY
