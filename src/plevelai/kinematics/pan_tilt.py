"""Pan/tilt rig kinematics utilities."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

from .planar_arm import JointLimits

_EPS = 1e-9


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class PanTiltRig:
    """Simple pan/tilt head for pointing at a ground target."""

    axis_height: float = 0.0
    pan_limits: Optional[JointLimits] = None
    tilt_limits: Optional[JointLimits] = None
    tilt_offset_deg: float = 0.0
    tilt_direction: int = 1  # +1 keeps atan2 sign, -1 flips it if mechanics invert it

    def solve(self, x: float, y: float, target_z: float) -> Dict[str, float]:
        horizontal = math.hypot(x, y)
        if horizontal < _EPS:
            raise ValueError("Target sits on the pan axis; undefined azimuth.")

        pan_deg = math.degrees(math.atan2(y, x))

        raw_tilt = math.degrees(math.atan2(target_z - self.axis_height, horizontal))
        tilt_deg = self.tilt_direction * raw_tilt + self.tilt_offset_deg

        if self.pan_limits and not self.pan_limits.contains(pan_deg):
            raise ValueError(
                f"Pan angle {pan_deg:.2f}° outside limits {self.pan_limits.min_deg}..{self.pan_limits.max_deg}"
            )
        if self.tilt_limits:
            tilt_deg = _clamp(tilt_deg, self.tilt_limits.min_deg, self.tilt_limits.max_deg)

        return {"pan": pan_deg, "tilt": tilt_deg}


__all__ = ["PanTiltRig"]
