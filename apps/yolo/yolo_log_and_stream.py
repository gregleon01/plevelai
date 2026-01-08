# YOLO on CSI/USB, log pixel coords to JSONL, and stream annotated frames over HTTP (MJPEG).
# Env: MODEL=/path/best.pt | CAM=csi|usb | SENSOR_ID=0 | IMGSZ=640 | CONF=0.25 | LOG=./detections.log | PORT=8080
import os, time, json, threading
import cv2
from ultralytics import YOLO
from flask import Flask, Response

MODEL = os.environ.get("MODEL", "best.pt")
CAM   = os.environ.get("CAM", "csi")          # "csi" or "usb"
SENS  = int(os.environ.get("SENSOR_ID", "0")) # CSI slot index
IMGSZ = int(os.environ.get("IMGSZ", "640"))
CONF  = float(os.environ.get("CONF", "0.25"))
LOG   = os.path.abspath(os.environ.get("LOG", "./detections.log"))
PORT  = int(os.environ.get("PORT", "8080"))

def csi_gst(width=1280, height=720, fps=30):
    # Use sensor-id (some boards have multiple CSI lanes)
    return (
        f"nvarguscamerasrc sensor-id={SENS} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, framerate={fps}/1, format=NV12 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink"
    )

# Open camera via OpenCV
cap = cv2.VideoCapture(
    0 if CAM != "csi" else csi_gst(),
    cv2.CAP_GSTREAMER if CAM == "csi" else 0
)
if not cap.isOpened():
    raise SystemExit("Camera open failed (check CAM=csi|usb, SENSOR_ID, and that no other process holds CSI).")

model = YOLO(MODEL)

last_frame = None
last_lock  = threading.Lock()
stop_flag  = False

def infer_and_log():
    global last_frame, stop_flag
    with open(LOG, "a") as f:
        while not stop_flag:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02); continue

            # YOLO inference
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
                    # draw overlays for the stream
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                    cv2.circle(frame, (int(u), int(v)), 4, (0,0,255), -1)

            # append JSONL
            f.write(json.dumps({"ts": time.time(), "detections": dets}) + "\n")
            f.flush()

            # publish frame for HTTP stream
            with last_lock:
                last_frame = frame

# start worker thread
t = threading.Thread(target=infer_and_log, daemon=True)
t.start()

# Minimal Flask app for MJPEG
from flask import Flask
app = Flask(__name__)

@app.route("/")
def root():
    return f"OK. Stream at /video (MJPEG). Log at {LOG}"

@app.route("/video")
def video():
    def gen():
        while True:
            with last_lock:
                frm = None if last_frame is None else last_frame.copy()
            if frm is None:
                time.sleep(0.02); continue
            ok, jpg = cv2.imencode(".jpg", frm, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + bytearray(jpg) + b"\r\n")
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    try:
        # Run HTTP server; capture thread keeps feeding frames
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        stop_flag = True
        cap.release()
