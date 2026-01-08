from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

import cv2
import numpy as np
from ultralytics import YOLO

import yolo_launch as yl


class DetectionService:
    """Background worker that runs YOLO, projects targets, and exposes live state."""

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        args = yl.parse_args([])  # reuse CLI defaults/env parsing
        resolved = yl.resolve_settings(args)
        if config:
            resolved.update(config)

        self._settings = resolved
        self._model = yl.load_model(resolved["model"])
        self._cap = yl.open_capture(
            resolved["cam"],
            resolved["usb_index"],
            resolved["width"],
            resolved["height"],
            resolved["fps"],
        )
        self._names = getattr(self._model, "names", {}) or {}

        robot_cfg = yl.load_robot_config()
        self._projector = yl.PixelProjector(robot_cfg.get("projection", {}))
        self._rig = yl.build_pan_tilt(robot_cfg)
        self._extrinsics = robot_cfg.get("camera_to_arm", {})
        self._plane_z = float(robot_cfg.get("target_plane_z_m", 0.0))

        # serial connection is optional
        self._serial = yl.open_serial_connection(resolved["serial_port"], resolved["serial_baud"])
        self._last_serial_payload: Optional[str] = None

        self._frame_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._status: Dict[str, Any] = {
            "last_update": None,
            "has_target": False,
            "target": None,
            "fps": 0.0,
            "serial_connected": bool(self._serial),
            "serial_port": resolved["serial_port"],
        }
        self._events: Deque[Dict[str, Any]] = deque(maxlen=256)

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    def _loop(self) -> None:
        conf = self._settings["conf"]
        imgsz = self._settings["imgsz"]
        target_name = self._settings["target_name"]
        conf_min = self._settings["conf_min"]

        frame_count = 0
        start_time = time.time()
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.02)
                continue

            result = self._model.predict(
                source=frame,
                device=0,
                imgsz=imgsz,
                conf=conf,
                verbose=False,
            )[0]

            target = yl.pick_target(result, self._names, target_name, conf_min, conf)
            event: Dict[str, Any]
            payload = "NO_TARGET\n"
            serial_sent = False
            x_ground = y_ground = None
            angles: Optional[Dict[str, float]] = None

            if target:
                u, v, score = target
                try:
                    x_ground, y_ground = self._projector.map(u, v, frame.shape[1], frame.shape[0])
                    x_rig, y_rig = yl.transform_camera_to_rig(float(x_ground), float(y_ground), self._extrinsics)
                    if self._rig is not None:
                        angles = self._rig.solve(x_rig, y_rig, self._plane_z)
                except Exception as exc:  # pragma: no cover - depends on calibration/hardware
                    event = {
                        "timestamp": time.time(),
                        "message": f"IK failure: {exc}",
                        "has_target": False,
                        "target": None,
                        "serial_sent": False,
                    }
                else:
                    if angles:
                        payload_dict = {
                            "cmd": "move",
                            "joints": {joint: float(value) for joint, value in angles.items()},
                            "conf": float(score),
                            "pixel": {"u": float(u), "v": float(v)},
                            "target_ground": [float(x_ground), float(y_ground), self._plane_z],
                            "timestamp": time.time(),
                        }
                        payload = json.dumps(payload_dict) + "\n"
                        if self._serial and payload != self._last_serial_payload:
                            try:
                                self._serial.write(payload.encode("ascii"))
                                self._serial.flush()
                                self._last_serial_payload = payload
                                serial_sent = True
                            except Exception:
                                serial_sent = False
                                self._serial = None
                        event = {
                            "timestamp": time.time(),
                            "message": "target",
                            "has_target": True,
                            "score": float(score),
                            "pixel": {"u": float(u), "v": float(v)},
                            "target_ground": [float(x_ground), float(y_ground), self._plane_z],
                            "joints": angles,
                            "serial_sent": serial_sent,
                        }
                    else:
                        event = {
                            "timestamp": time.time(),
                            "message": "ik_unavailable",
                            "has_target": True,
                            "pixel": {"u": float(u), "v": float(v)},
                            "serial_sent": False,
                        }
            else:
                event = {
                    "timestamp": time.time(),
                    "message": "no_target",
                    "has_target": False,
                    "serial_sent": False,
                }

            output = result.plot()
            yl.annotate_output(output, target, target_name)

            with self._frame_lock:
                self._latest_frame = output

            with self._status_lock:
                fps = 0.0
                frame_count += 1
                elapsed = time.time() - start_time
                if elapsed > 0:
                    fps = frame_count / elapsed
                status = {
                    "last_update": event["timestamp"],
                    "has_target": event.get("has_target", False),
                    "target": event.get("target_ground"),
                    "pixel": event.get("pixel"),
                    "score": event.get("score"),
                    "joints": event.get("joints"),
                    "serial_sent": event.get("serial_sent", False),
                    "serial_connected": bool(self._serial),
                    "serial_port": self._settings["serial_port"],
                    "fps": fps,
                }
                self._status = status
                self._events.appendleft(event)

        self._cap.release()
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass

    def stop(self) -> None:
        self._stop.set()
        self._worker.join(timeout=2.0)

    def latest_frame(self) -> Optional[np.ndarray]:
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def latest_jpeg(self) -> Optional[bytes]:
        frame = self.latest_frame()
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return None
        return buf.tobytes()

    def status(self) -> Dict[str, Any]:
        with self._status_lock:
            return dict(self._status)

    def events(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._status_lock:
            snapshot = list(self._events)
        return list(reversed(snapshot[:limit]))
