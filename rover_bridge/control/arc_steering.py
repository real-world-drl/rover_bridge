# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Arc steering — linear/angular velocities from waypoints via pure pursuit.

Ported from ``ros_ws/src/scripts/arc_steering.py``. The geometry is identical;
the defaults are retuned for a small differential-drive rover:

- The rover can pivot in place freely, so ``turn_in_place_threshold_deg``
  defaults to a real value (45°) instead of being disabled — sharp targets
  are handled by spinning toward them rather than tracing a backwards loop.
- Velocity caps default to rover-scale values (0.5 m/s, 2.0 rad/s) rather than
  Spot-scale. The firmware also clamps (UGV_MAX_LINEAR/ANGULAR), so these are
  a host-side shaping cap, not the hard safety limit.

Waypoint frame: x = forward, y = lateral (negative y = right).
"""

from __future__ import annotations

import math
from typing import Optional


class ArcSteering:
    def __init__(self, waypoint_index: int = 4, actuation_duration: float = 2.0,
                 max_linear_velocity: float = 0.5, max_angular_velocity: float = 2.0,
                 turn_in_place_threshold_deg: float = 45.0,
                 min_angular_velocity: float = 0.0):
        """
        Args:
            waypoint_index: Target waypoint index (0-indexed).
            actuation_duration: Seconds over which the arc is intended to be
                driven (sets the velocity scale). Tie to the publisher buffer.
            max_linear_velocity: Cap on |linear.x| in m/s.
            max_angular_velocity: Cap on |angular.z| in rad/s.
            turn_in_place_threshold_deg: If the bearing to the target exceeds
                this, zero linear velocity and pivot in place toward it. A
                differential rover does this cleanly; set large (e.g. 180) to
                disable and always trace an arc.
            min_angular_velocity: Dead-band on |angular.z| (rad/s); smaller
                magnitudes are snapped to 0 to suppress jittery micro-turns.
        """
        self.waypoint_index = waypoint_index
        self.actuation_duration = actuation_duration
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.turn_in_place_threshold_rad = math.radians(turn_in_place_threshold_deg)
        self.min_angular_velocity = min_angular_velocity

    def compute_velocities(self, waypoints, index: Optional[int] = None):
        """Compute ``(linear, angular)`` from the arc to the target waypoint.

        Uses pure-pursuit curvature ``kappa = 2*ty / (tx^2 + ty^2)``.

        Args:
            waypoints: list of ``[x, y, sin_theta, cos_theta]`` (last two unused).
            index: optional override of ``self.waypoint_index``.

        Returns:
            ``(linear_vel, angular_vel)`` or ``None`` if waypoints are invalid.
        """
        if index is None:
            index = self.waypoint_index

        if not waypoints or len(waypoints) < index + 1:
            return None

        tx, ty = waypoints[index][0], waypoints[index][1]
        dist_sq = tx * tx + ty * ty

        if dist_sq < 1e-12:
            return (0.0, 0.0)

        target_angle = math.atan2(ty, tx)

        if abs(target_angle) > self.turn_in_place_threshold_rad:
            linear_vel = 0.0
            angular_vel = target_angle / self.actuation_duration
        elif abs(ty) < 1e-6:  # straight line
            linear_vel = tx / self.actuation_duration
            angular_vel = 0.0
        else:
            kappa = 2.0 * ty / dist_sq
            R = 1.0 / kappa
            theta = 2.0 * target_angle
            arc_length = abs(R * theta)
            linear_vel = arc_length / self.actuation_duration
            angular_vel = theta / self.actuation_duration

        # Apply the tighter of the two per-axis caps while preserving the
        # linear/angular ratio (so the geometric arc shape is unchanged).
        linear_scale = (self.max_linear_velocity / abs(linear_vel)
                        if abs(linear_vel) > self.max_linear_velocity else 1.0)
        angular_scale = (self.max_angular_velocity / abs(angular_vel)
                         if abs(angular_vel) > self.max_angular_velocity else 1.0)
        scale = min(linear_scale, angular_scale)
        if scale < 1.0:
            linear_vel *= scale
            angular_vel *= scale

        if abs(angular_vel) < self.min_angular_velocity:
            angular_vel = 0.0

        return (linear_vel, angular_vel)
