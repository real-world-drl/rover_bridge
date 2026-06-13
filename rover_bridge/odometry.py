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
from typing import Callable, Optional

from . import wire
from .logging_util import get_logger

log = get_logger("odometry")


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
