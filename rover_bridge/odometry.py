# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Host-side differential-drive wheel odometry.

The rover's encoders work and `tel/wheel` carries *cumulative* signed tick
counts (network-jitter-independent — the host just differences successive
samples). We integrate those into a planar pose ``(x, y, yaw)`` and feed the
waypoint follower, which needs pose for advance + twist recomputation.

Geometry must match the firmware's Kconfig so the host pose agrees with the
rover's own estimate:

    UGV_WHEEL_DIAMETER_MM   default 80
    UGV_TRACK_WIDTH_MM      default 172   (wheel separation)
    UGV_ENCODER_PPR         default 1650  (post-gearing, 2x quadrature)

Distance per tick = pi * wheel_diameter / PPR. Standard exact-arc integration:
heading advances by ``(d_right - d_left) / track_width`` per step, position by
the mean wheel distance along the mid-step heading.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from . import wire
from .logging_util import get_logger

log = get_logger("odometry")


def pose_stamped_dict(x: float, y: float, yaw: float, frame_id: str = "odom",
                      seq: int = 0) -> dict:
    """Build a ``geometry_msgs/PoseStamped``-shaped dict from a planar pose.

    Matches the JSON ros_ws publishes when it forwards a PoseStamped pose topic
    to MQTT (``ros_msg_to_dict``): ``{header, pose:{position, orientation}}``,
    z = 0 and a yaw-only quaternion (roll = pitch = 0). Consumers walk
    ``["pose"]["position"]`` / ``["pose"]["orientation"]`` exactly as they would
    for the Spot feed. Yaw is the standard CCW math convention; consumers apply
    their own heading convention (as the data logger's ``quat_to_rpy`` does).
    """
    ns = time.time_ns()
    return {
        "header": {
            "seq": seq,
            "stamp": {"secs": ns // 1_000_000_000, "nsecs": ns % 1_000_000_000},
            "frame_id": frame_id,
        },
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {
                "x": 0.0, "y": 0.0,
                "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0),
            },
        },
    }


def pose_from_stamped_dict(d: dict) -> tuple:
    """Inverse of ``pose_stamped_dict``: extract ``(x, y, yaw)`` from a
    PoseStamped-shaped dict.

    Reads ``pose.position.x/y`` and the ZYX yaw of ``pose.orientation``. The
    orientation may be a full 3D quaternion (rover_vio's VIO pose) rather than
    the yaw-only one this module emits; on a ground rover roll/pitch are ~0, so
    the extracted yaw is the heading. Used by the bridge when ``pose_source:
    vio`` to consume rover_vio's pose in place of wheel odometry.
    """
    p = d["pose"]["position"]
    o = d["pose"]["orientation"]
    qx, qy, qz, qw = float(o["x"]), float(o["y"]), float(o["z"]), float(o["w"])
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return float(p["x"]), float(p["y"]), yaw


class WheelOdometry:
    def __init__(self, wheel_diameter_m: float = 0.080,
                 track_width_m: float = 0.172,
                 encoder_ppr: int = 1650,
                 on_pose: Optional[Callable[[float, float, float], None]] = None):
        """
        Args:
            wheel_diameter_m: Drive wheel diameter (m). Firmware default 80 mm.
            track_width_m: Wheel separation (m). Firmware default 172 mm.
            encoder_ppr: Encoder pulses per wheel revolution after gearing /
                quadrature decode. Firmware default 1650.
            on_pose: Called with ``(x, y, yaw)`` after each integration step.
        """
        self._m_per_tick = math.pi * wheel_diameter_m / encoder_ppr
        self._track_width = track_width_m
        self._on_pose = on_pose

        self._lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._prev_left: Optional[int] = None
        self._prev_right: Optional[int] = None

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        with self._lock:
            self.x, self.y, self.yaw = x, y, yaw
            self._prev_left = None
            self._prev_right = None

    @property
    def pose(self) -> tuple[float, float, float]:
        with self._lock:
            return (self.x, self.y, self.yaw)

    def update(self, telem: wire.WheelTelem) -> None:
        """Integrate one wheel-telemetry sample. Wire this to ``on_wheel``."""
        with self._lock:
            if self._prev_left is None:
                # First sample establishes the tick baseline; no motion yet.
                self._prev_left = telem.left_ticks
                self._prev_right = telem.right_ticks
                pose = (self.x, self.y, self.yaw)
            else:
                d_left = (telem.left_ticks - self._prev_left) * self._m_per_tick
                d_right = (telem.right_ticks - self._prev_right) * self._m_per_tick
                self._prev_left = telem.left_ticks
                self._prev_right = telem.right_ticks

                d_center = 0.5 * (d_left + d_right)
                d_yaw = (d_right - d_left) / self._track_width
                mid_yaw = self.yaw + 0.5 * d_yaw
                self.x += d_center * math.cos(mid_yaw)
                self.y += d_center * math.sin(mid_yaw)
                self.yaw = _wrap_pi(self.yaw + d_yaw)
                pose = (self.x, self.y, self.yaw)

        if self._on_pose:
            self._on_pose(*pose)


def _wrap_pi(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
