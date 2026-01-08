"""Basic inverse kinematics for a planar 2-link arm."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class JointLimits:
    min_deg: float
    max_deg: float

    def contains(self, angle_deg: float) -> bool:
        return self.min_deg <= angle_deg <= self.max_deg


@dataclass
class PlanarTwoLinkArm:
    """Planar arm with two revolute joints (shoulder + elbow)."""

    link_1: float
    link_2: float
    base_height: float = 0.0
    tool_z_offset: float = 0.0
    joint_1_limits: Optional[JointLimits] = None
    joint_2_limits: Optional[JointLimits] = None

    def solve(self, x: float, y: float, *, elbow_up: bool = True) -> Dict[str, float]:
        """Return joint angles (degrees) that place the tool center at (x, y)."""
        r_sq = x * x + y * y
        l1, l2 = self.link_1, self.link_2
        if r_sq < 1e-12:
            raise ValueError("Target is too close to arm base; singular configuration")

        cos_theta2 = _clamp((r_sq - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1.0, 1.0)
        theta2 = math.acos(cos_theta2)
        if elbow_up:
            theta2 = -theta2

        k1 = l1 + l2 * math.cos(theta2)
        k2 = l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        if self.joint_1_limits and not self.joint_1_limits.contains(theta1_deg):
            raise ValueError(
                f"Shoulder angle {theta1_deg:.2f}° is outside limits {self.joint_1_limits}"
            )
        if self.joint_2_limits and not self.joint_2_limits.contains(theta2_deg):
            raise ValueError(
                f"Elbow angle {theta2_deg:.2f}° is outside limits {self.joint_2_limits}"
            )

        return {"joint_1": theta1_deg, "joint_2": theta2_deg}

    def reachable(self, x: float, y: float) -> bool:
        dist = math.hypot(x, y)
        return abs(self.link_1 - self.link_2) <= dist <= (self.link_1 + self.link_2)

    def z_height(self) -> float:
        return self.base_height + self.tool_z_offset


__all__ = ["PlanarTwoLinkArm", "JointLimits"]
