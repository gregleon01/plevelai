# Logs bbox center pixel coords (u,v) to JSONL and optionally shows live video.
# Env vars:
#   MODEL=/path/to/best.pt | CAM=usb|csi | IMGSZ=640 | CONF=0.25 | LOG=./detections.log | SHOW=0|1
import os, time, json
import cv2
from ultralytics import YOLO

MODEL = os.environ.get("MODEL", "best.pt")
CAM   = os.environ.get("CAM", "usb")      # "usb" or "csi"
IMGSZ = int(os.environ.get("IMGSZ", "640"))
CONF  = float(os.environ.get("CONF", "0.25"))
LOG   = os.path.abspath(os.environ.get("LOG", "./detections.log"))
SHOW  = int(os.environ.get("SHOW", "0"))

def csi_gst(width=1280, height=720, fps=30):
    return (
        f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"framerate={fps}/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, "
        f"width={width}, height={height}, format=BGRx ! videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )
    if show:
        # direct to Jetson HDMI monitor (no GTK needed)
        pipeline += " ! nvvidconv ! nvoverlaysink sync=false"
    else:
        pipeline += " ! appsink drop=1"
    return pipeline

cap = cv2.VideoCapture(0 if CAM!="csi" else csi_gst(),
                       cv2.CAP_GSTREAMER if CAM=="csi" else 0)
if not cap.isOpened():
    raise SystemExit("Camera open failed (check CAM=usb|csi and device).")

model = YOLO(MODEL)

with open(LOG, "a") as f:
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        res = model.predict(source=frame, imgsz=IMGSZ, conf=CONF, verbose=False)
        dets = []
        if len(res):
            for b in res[0].boxes:
                x1,y1,x2,y2 = map(float, b.xyxy[0])
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                dets.append({
                    "u": u, "v": v,
                    "w": x2-x1, "h": y2-y1,
                    "cls": int(b.cls[0]) if hasattr(b,'cls') else -1,
                    "conf": float(b.conf[0]) if hasattr(b,'conf') else 0.0
                })

        f.write(json.dumps({"ts": time.time(), "detections": dets}) + "\n")
        f.flush()
