# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Waypoint follower with pose-driven advance.

Ported from ``ros_ws/src/scripts/waypoint_follower.py``, de-ROS-ified:

- Pose is *pushed in* via :meth:`update_pose` (driven by host wheel odometry in
  :mod:`rover_bridge.odometry`) instead of being pulled from a ROS topic. Yaw
  arrives directly, so the quaternion helpers are gone.
- Commands are :class:`CmdVel` sent through the :class:`RepeatedCmdVelPublisher`
  rather than ROS Twists.

Behavior is otherwise unchanged: it snapshots pose when each trajectory
arrives, projects waypoints from that reference frame, and advances to the
next waypoint as the rover reaches each target — bounded by
``max_waypoint_advance`` and an optional ``max_action_age``.
"""

from __future__ import annotations

import logging
import math
import threading
from typing import Optional

from ..logging_util import get_logger, log_throttle
from .arc_steering import ArcSteering
from .cmd_vel_publisher import CmdVel, RepeatedCmdVelPublisher

log = get_logger("waypoint_follower")


def _waypoint_to_global(tx, ty, ref_pose):
    """Project a waypoint from its action-time robot-local frame to global."""
    rx, ry, ryaw = ref_pose
    gx = rx + tx * math.cos(ryaw) - ty * math.sin(ryaw)
    gy = ry + tx * math.sin(ryaw) + ty * math.cos(ryaw)
    return gx, gy


def _waypoint_in_current_frame(tx, ty, ref_pose, cur_pose):
    """Re-express an action-time-local waypoint in the rover's current frame."""
    gx, gy = _waypoint_to_global(tx, ty, ref_pose)
    cx, cy, cyaw = cur_pose
    dx, dy = gx - cx, gy - cy
    tx_p = dx * math.cos(cyaw) + dy * math.sin(cyaw)
    ty_p = -dx * math.sin(cyaw) + dy * math.cos(cyaw)
    return tx_p, ty_p


class WaypointFollower:
    def __init__(self, publisher: RepeatedCmdVelPublisher, arc_steering: ArcSteering,
                 action_scale: float = 1.0, max_waypoint_advance: int = 0,
                 waypoint_tolerance: float = 0.3, max_action_age: Optional[float] = None,
                 recompute: bool = True):
        """
        Args:
            publisher: RepeatedCmdVelPublisher to push computed commands into.
            arc_steering: ArcSteering for computing velocities from waypoints.
            action_scale: Scale applied to computed linear/angular velocities.
            max_waypoint_advance: How many waypoints past
                ``arc_steering.waypoint_index`` to advance to before stopping.
                0 = no advance.
            waypoint_tolerance: Distance (m) within which a waypoint is "reached".
            max_action_age: Optional safety timer (s); force a stop if no fresh
                action arrives within this window. None disables it.
            recompute: When True, re-aim each buffer-phase command at the current
                target waypoint from the latest pose. Requires pose updates.
        """
        self.publisher = publisher
        self.arc_steering = arc_steering
        self.action_scale = action_scale
        self.max_waypoint_advance = max_waypoint_advance
        self.waypoint_tolerance = waypoint_tolerance
        self.max_action_age = max_action_age
        self.recompute = recompute

        self.lock = threading.Lock()
        self.waypoints = None
        self.ref_pose = None        # (x, y, yaw) snapshot at action arrival
        self.advance_offset = 0
        self.stopped = False

        self.latest_pose = None     # (x, y, yaw), updated by update_pose()
        self.stale_timer: Optional[threading.Timer] = None

    # --- pose feed ----------------------------------------------------------

    def update_pose(self, x: float, y: float, yaw: float) -> None:
        """Feed the latest pose (from wheel odometry). Triggers advance checks."""
        new_pose = (x, y, yaw)
        with self.lock:
            self.latest_pose = new_pose
        self._maybe_advance(new_pose)

    # --- action handling ----------------------------------------------------

    def on_new_action(self, waypoints) -> None:
        """Called when a new trajectory arrives over MQTT."""
        self._cancel_stale_timer()

        cmd = self._make_cmd(waypoints, self.arc_steering.waypoint_index)
        if cmd is None:
            log.warning("empty/invalid waypoints — ignoring action")
            return

        with self.lock:
            self.waypoints = waypoints
            self.advance_offset = 0
            self.stopped = False
            self.ref_pose = self.latest_pose
            if self.max_waypoint_advance > 0 and self.ref_pose is None:
                log_throttle(log, logging.WARNING, 10.0,
                             "action arrived before any pose update — advance "
                             "disabled for this trajectory")

        cb = self._recompute_cmd if self.recompute else None
        self.publisher.publish(cmd, recompute_callback=cb)
        self._arm_stale_timer()

    def force_stop(self) -> None:
        """External stop (e.g. from the ctrl topic). Clears trajectory state."""
        self._cancel_stale_timer()
        with self.lock:
            self.waypoints = None
            self.ref_pose = None
            self.stopped = True
        self.publisher.publish(CmdVel())

    # --- internals ----------------------------------------------------------

    def _maybe_advance(self, current_pose) -> None:
        new_cmd = None
        with_callback = False
        log_msg = None

        with self.lock:
            if (self.waypoints is None or self.stopped
                    or self.max_waypoint_advance <= 0
                    or self.ref_pose is None):
                return

            base_idx = self.arc_steering.waypoint_index
            target_idx = base_idx + self.advance_offset

            if target_idx >= len(self.waypoints):
                self.stopped = True
                new_cmd = CmdVel()
                log_msg = f"trajectory exhausted at index {target_idx} — stopping"
            else:
                tx, ty = self.waypoints[target_idx][0], self.waypoints[target_idx][1]
                wp_gx, wp_gy = _waypoint_to_global(tx, ty, self.ref_pose)
                cx, cy, _ = current_pose
                dist = math.hypot(cx - wp_gx, cy - wp_gy)

                if dist < self.waypoint_tolerance:
                    if self.advance_offset >= self.max_waypoint_advance:
                        self.stopped = True
                        new_cmd = CmdVel()
                        log_msg = (f"reached waypoint {target_idx} but at max advance "
                                   f"budget ({self.max_waypoint_advance}) — stopping")
                    else:
                        self.advance_offset += 1
                        new_idx = base_idx + self.advance_offset
                        new_cmd = self._make_cmd(self.waypoints, new_idx)
                        with_callback = True
                        log_msg = f"reached waypoint {target_idx}, advancing to {new_idx}"

        if log_msg is not None:
            log.info(log_msg)
        if new_cmd is not None:
            cb = self._recompute_cmd if (with_callback and self.recompute) else None
            self.publisher.publish(new_cmd, recompute_callback=cb)

    def _make_cmd(self, waypoints, index) -> Optional[CmdVel]:
        computed = self.arc_steering.compute_velocities(waypoints, index=index)
        if computed is None:
            return None
        return CmdVel(linear_x=computed[0] * self.action_scale,
                      angular_z=computed[1] * self.action_scale)

    def _recompute_cmd(self) -> Optional[CmdVel]:
        """Re-aim the arc at the current target waypoint from the latest pose."""
        with self.lock:
            if (self.waypoints is None or self.stopped
                    or self.ref_pose is None or self.latest_pose is None):
                return None
            waypoints = self.waypoints
            ref_pose = self.ref_pose
            cur_pose = self.latest_pose
            target_idx = self.arc_steering.waypoint_index + self.advance_offset

        if target_idx >= len(waypoints) or target_idx < 0:
            return None

        tx, ty = waypoints[target_idx][0], waypoints[target_idx][1]
        tx_p, ty_p = _waypoint_in_current_frame(tx, ty, ref_pose, cur_pose)
        return self._make_cmd([[tx_p, ty_p, 0.0, 0.0]], 0)

    def _arm_stale_timer(self) -> None:
        if self.max_action_age is None:
            return
        self.stale_timer = threading.Timer(self.max_action_age, self._on_stale)
        self.stale_timer.daemon = True
        self.stale_timer.start()

    def _cancel_stale_timer(self) -> None:
        if self.stale_timer is not None:
            self.stale_timer.cancel()
            self.stale_timer = None

    def _on_stale(self) -> None:
        log.warning("action stale (>%s s without fresh command) — stopping",
                    self.max_action_age)
        with self.lock:
            self.stopped = True
        self.publisher.publish(CmdVel())
