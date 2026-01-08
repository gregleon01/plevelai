"""Runtime that converts YOLO detections into joint angles + Arduino commands."""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Iterable, Iterator, Optional

try:
    import yaml
except ImportError as exc:  # pragma: no cover - user must install dependency
    raise SystemExit("PyYAML is required: pip install pyyaml") from exc

from plevelai.control.host.serial_bridge import ArduinoBridge
from plevelai.kinematics.planar_arm import JointLimits
from plevelai.kinematics.pan_tilt import PanTiltRig
from plevelai.vision.calibration.homography import Homography


@dataclass
class Target:
    timestamp: float
    enqueued_at: float
    conf: float
    u: float
    v: float
    w: float
    h: float
    x_ground: float
    y_ground: float
    x_arm: float
    y_arm: float

    def age(self, now: float) -> float:
        return now - self.enqueued_at


def load_config(path: Path) -> Dict:
    with path.open() as fh:
        return yaml.safe_load(fh)


def build_rig(cfg: Dict) -> PanTiltRig:
    rig_cfg = cfg.get("pan_tilt") or {}

    pan_limits = None
    tilt_limits = None
    limits = rig_cfg.get("joint_limits_deg", {})
    if "pan" in limits:
        pan_limits = JointLimits(*limits["pan"])
    if "tilt" in limits:
        tilt_limits = JointLimits(*limits["tilt"])

    return PanTiltRig(
        axis_height=float(rig_cfg.get("axis_height_m", 0.3)),
        pan_limits=pan_limits,
        tilt_limits=tilt_limits,
        tilt_offset_deg=float(rig_cfg.get("tilt_offset_deg", 0.0)),
        tilt_direction=int(rig_cfg.get("tilt_direction", 1)),
    )


def transform_camera_to_arm(x: float, y: float, extrinsics: Dict) -> tuple[float, float]:
    rot_deg = float(extrinsics.get("rotation_deg", 0.0))
    tx, ty = extrinsics.get("translation_m", [0.0, 0.0])
    rot_rad = math.radians(rot_deg)
    cos_r = math.cos(rot_rad)
    sin_r = math.sin(rot_rad)
    xr = cos_r * x - sin_r * y
    yr = sin_r * x + cos_r * y
    return xr + tx, yr + ty


def detection_stream(path: Path, follow: bool = True) -> Iterator[Dict]:
    with path.open() as fh:
        if follow:
            fh.seek(0, 2)
        while True:
            line = fh.readline()
            if not line:
                if follow:
                    time.sleep(0.05)
                    continue
                break
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue
def prioritized_detections(
    dets: Iterable[Dict], min_conf: float, min_area: float
) -> list[Dict]:
    filtered = []
    for det in dets:
        conf = float(det.get("conf", 0.0))
        area = float(det.get("w", 0.0)) * float(det.get("h", 0.0))
        if conf < min_conf or area < min_area:
            continue
        filtered.append(det)
    filtered.sort(
        key=lambda d: (float(d.get("v", 0.0)), float(d.get("conf", 0.0))),
        reverse=True,
    )
    return filtered


def select_target(queue: Deque[Target]) -> Optional[Target]:
    best: Optional[Target] = None
    best_v = float("-inf")
    best_conf = -1.0
    for tgt in queue:
        if tgt.v > best_v or (math.isclose(tgt.v, best_v) and tgt.conf > best_conf):
            best = tgt
            best_v = tgt.v
            best_conf = tgt.conf
    return best


def prune_queue(queue: Deque[Target], max_age_s: float, now: float) -> None:
    if not queue or max_age_s <= 0:
        return
    keep = [t for t in queue if now - t.enqueued_at <= max_age_s]
    if len(keep) != len(queue):
        queue.clear()
        queue.extend(keep)


def is_duplicate(queue: Deque[Target], candidate: Target, merge_dist_m: float) -> bool:
    if merge_dist_m <= 0:
        return False
    for tgt in queue:
        if math.hypot(tgt.x_ground - candidate.x_ground, tgt.y_ground - candidate.y_ground) <= merge_dist_m:
            return True
    return False


def run(args: argparse.Namespace) -> None:
    cfg = load_config(args.config)
    homography = Homography.load(cfg.get("homography_path", None))
    rig = build_rig(cfg)

    extrinsics = cfg.get("camera_to_arm", {})
    plane_z = float(cfg.get("target_plane_z_m", 0.0))
    min_conf = float(cfg.get("min_confidence", args.min_conf))
    min_area = float(cfg.get("min_bbox_area_px", args.min_area))

    queue_cfg = cfg.get("runtime_queue", {})
    queue_len = int(queue_cfg.get("max_len", args.queue_len))
    queue_len = max(queue_len, 1)
    queue_stale = float(queue_cfg.get("stale_seconds", args.queue_stale_sec))
    queue_merge = float(queue_cfg.get("merge_distance_m", args.queue_merge_dist))

    telemetry_path = queue_cfg.get("telemetry_log")
    if args.telemetry_log is not None:
        telemetry_path = args.telemetry_log
    telemetry_writer = None
    telemetry_file = None
    if telemetry_path:
        telemetry_path = Path(telemetry_path)
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not telemetry_path.exists()
        telemetry_file = telemetry_path.open("a", newline="")
        telemetry_writer = csv.writer(telemetry_file)
        if new_file:
            telemetry_writer.writerow(
                [
                    "sent_ts",
                    "det_ts",
                    "confidence",
                    "pan_deg",
                    "tilt_deg",
                    "ground_x",
                    "ground_y",
                    "image_v",
                    "queue_after",
                    "target_age_s",
                ]
            )

    bridge_cfg = cfg.get("arduino", {})
    serial_port = args.serial_port or bridge_cfg.get("port")
    baudrate = int(args.baudrate or bridge_cfg.get("baudrate", 115200))

    if not serial_port and not args.dry_run:
        raise ValueError("Serial port not provided. Use --serial-port or configs/robot.yaml")

    default_home = bool(queue_cfg.get("home_on_start", True))
    if args.skip_home:
        home_on_start = False
    else:
        home_on_start = default_home or args.home_once
    target_queue: Deque[Target] = deque(maxlen=queue_len)

    bridge: Optional[ArduinoBridge] = None
    try:
        bridge = ArduinoBridge(port=serial_port, baudrate=baudrate, dry_run=args.dry_run)
        if home_on_start:
            bridge.send_home()

        for entry in detection_stream(args.log, follow=not args.once):
            now = time.time()
            prune_queue(target_queue, queue_stale, now)

            entry_ts = float(entry.get("ts", now))
            for det in prioritized_detections(entry.get("detections", []), min_conf, min_area):
                u = float(det.get("u", 0.0))
                v = float(det.get("v", 0.0))
                w = float(det.get("w", 0.0))
                h = float(det.get("h", 0.0))
                x_ground, y_ground = homography.image_to_ground(u, v)
                x_arm, y_arm = transform_camera_to_arm(x_ground, y_ground, extrinsics)
                candidate = Target(
                    timestamp=entry_ts,
                    enqueued_at=now,
                    conf=float(det.get("conf", 0.0)),
                    u=u,
                    v=v,
                    w=w,
                    h=h,
                    x_ground=x_ground,
                    y_ground=y_ground,
                    x_arm=x_arm,
                    y_arm=y_arm,
                )
                if is_duplicate(target_queue, candidate, queue_merge):
                    continue
                target_queue.append(candidate)

            target = select_target(target_queue)
            if target is None:
                continue

            try:
                joint_angles = rig.solve(target.x_arm, target.y_arm, plane_z)
            except ValueError as err:
                try:
                    target_queue.remove(target)
                except ValueError:
                    pass
                if args.verbose:
                    print(f"Skipping target {(target.x_arm, target.y_arm)}: {err}")
                continue

            queue_depth_before = len(target_queue)
            target_age = target.age(now)
            try:
                target_queue.remove(target)
            except ValueError:
                queue_depth_after = len(target_queue)
            else:
                queue_depth_after = len(target_queue)

            metadata = {
                "conf": target.conf,
                "target_ground": [target.x_ground, target.y_ground, plane_z],
                "timestamp": target.timestamp,
                "queue_depth": queue_depth_before,
                "queue_depth_after": queue_depth_after,
                "queue_age_s": target_age,
            }
            bridge.send_move(joint_angles, metadata=metadata)

            if telemetry_writer:
                telemetry_writer.writerow(
                    [
                        now,
                        target.timestamp,
                        target.conf,
                        joint_angles.get("pan"),
                        joint_angles.get("tilt"),
                        target.x_ground,
                        target.y_ground,
                        target.v,
                        queue_depth_after,
                        target_age,
                    ]
                )
                telemetry_file.flush()

            if args.once:
                break
    finally:
        if telemetry_file:
            telemetry_file.close()
        if bridge:
            bridge.close()


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--log", type=Path, default=Path("detections.log"))
    p.add_argument("--config", type=Path, default=Path("configs/robot.yaml"))
    p.add_argument("--serial-port", type=str, default=None)
    p.add_argument("--baudrate", type=int, default=None)
    p.add_argument("--dry-run", action="store_true", help="Do not open serial; print commands")
    p.add_argument("--once", action="store_true", help="Process existing log and exit")
    p.add_argument("--min-conf", type=float, default=0.5)
    p.add_argument("--min-area", type=float, default=20)
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--queue-len", type=int, default=5, help="Maximum queued targets")
    p.add_argument(
        "--queue-stale-sec",
        type=float,
        default=1.0,
        help="Drop queued detections older than this many seconds",
    )
    p.add_argument(
        "--queue-merge-dist",
        type=float,
        default=0.05,
        help="Merge detections within this ground-plane distance (meters)",
    )
    p.add_argument(
        "--telemetry-log",
        type=Path,
        default=None,
        help="Optional CSV log for dispatched commands",
    )
    p.add_argument(
        "--home-once",
        action="store_true",
        help="Send a home command when the controller starts",
    )
    p.add_argument(
        "--skip-home",
        action="store_true",
        help="Suppress automatic homing at startup",
    )
    return p


def main() -> None:
    args = build_argparser().parse_args()
    run(args)


if __name__ == "__main__":
    main()
