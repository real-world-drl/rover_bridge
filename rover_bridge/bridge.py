# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Top-level orchestrator: wires transport, odometry, control, inference, camera.

Data flow (one direction of the loop each):

    camera ─preprocess─▶ inference.publish_camera ─MQTT▶ model
    model ─MQTT(gemnav/act)▶ inference ─▶ waypoint follower / arc steering
        ─▶ RepeatedCmdVelPublisher ─▶ transport.send_cmd_vel ─UART/MQTT▶ rover
    rover ─tel/wheel─▶ transport ─▶ wheel odometry ─pose─▶ waypoint follower

The rover transport is selectable (UART default, MQTT alternative); everything
on the inference side is MQTT regardless.

Two pose sources can run at once. ``pose_source`` picks the *active* one — it
steers the follower and goes out on ``pose_topic``. The other keeps running as
*ground truth*: republished to ``gt_pose_topic`` and/or logged to SQLite, but
never fed back into control. Running on wheel odometry with VIO as ground truth
is how encoder drift gets measured against a reference on the same timeline.
"""

from __future__ import annotations

import hashlib
import json
import logging
import time
from types import SimpleNamespace

from . import wire
from .battery import percent_from_voltage
from .cameras import make_camera
from .control import ArcSteering, RepeatedCmdVelPublisher, WaypointFollower
from .inference import InferenceClient
from .logging_util import get_logger, log_throttle
from .odometry import WheelOdometry, pose_from_stamped_dict, pose_stamped_dict
from .pose_log import PoseLog
from .transports import TelemetryCallbacks, make_transport

log = get_logger("bridge")


def redact_goal(payload: dict) -> dict:
    """Replace a base64 goal image with a fingerprint.

    ``gemnav/goal`` may carry a whole JPEG inline. Storing that in every log
    would bloat the database for no analytical gain, but which image was used
    still matters for reproducing a run — so keep a short hash and the size.
    """
    out = dict(payload)
    img = out.pop("goal_image", None)
    if img is not None:
        raw = img.encode("utf-8") if isinstance(img, str) else bytes(img)
        out["goal_image_sha256"] = hashlib.sha256(raw).hexdigest()[:16]
        out["goal_image_b64_len"] = len(img)
    return out


def describe_goal(goal: dict) -> str:
    """One-line human summary of a redacted goal, for the log."""
    if goal.get("clear"):
        return "cleared"
    parts = []
    if "goal_world_x" in goal and "goal_world_y" in goal:
        parts.append(f"pose=({goal['goal_world_x']}, {goal['goal_world_y']})")
    if "goal_image_sha256" in goal:
        parts.append(f"image=sha256:{goal['goal_image_sha256']}")
    if "scoring_goal_x" in goal:
        parts.append(f"scoring=({goal['scoring_goal_x']}, {goal['scoring_goal_y']})"
                     " [analysis only]")
    return " ".join(parts) if parts else "empty"


def gt_capture_enabled(cfg: SimpleNamespace) -> bool:
    """True when the bridge should capture the *secondary* pose source as ground
    truth — republish it to ``gt_pose_topic``, log it to ``pose_log_db``, or both.

    Ground truth is whichever source is not driving the follower: VIO under
    ``pose_source: wheel`` (the drift-comparison case), wheel odometry under
    ``pose_source: vio``. Lives here rather than in ``cli`` because ``cli``
    imports this module.
    """
    return bool(cfg.pose_log_db) or bool(cfg.publish_gt_pose and cfg.gt_pose_topic)


class RoverBridge:
    def __init__(self, cfg: SimpleNamespace):
        self.cfg = cfg

        # --- rover transport (telemetry callbacks populated below) ----------
        self.callbacks = TelemetryCallbacks()
        if cfg.transport == "uart":
            transport_kwargs = dict(port=cfg.uart_port, baud=cfg.uart_baud)
        else:
            transport_kwargs = dict(broker=cfg.broker, port=cfg.port,
                                    robot_id=cfg.robot_id)
        self.transport = make_transport(cfg.transport, self.callbacks, **transport_kwargs)

        # --- command publisher (drives the transport, also the heartbeat) ---
        self.publisher = RepeatedCmdVelPublisher(
            self.transport.send_cmd_vel,
            publish_rate=cfg.publish_rate,
            max_publishes=cfg.max_publishes,
            max_zero_publishes=cfg.max_zero_publishes,
        )

        # --- waypoint following / arc steering ------------------------------
        self.follower = None
        if cfg.use_waypoints:
            arc = ArcSteering(
                waypoint_index=cfg.waypoint_index,
                actuation_duration=cfg.actuation_duration,
                max_linear_velocity=cfg.max_linear_velocity,
                max_angular_velocity=cfg.max_angular_velocity,
                turn_in_place_threshold_deg=cfg.turn_in_place_threshold_deg,
                min_angular_velocity=cfg.min_angular_velocity,
            )
            self.follower = WaypointFollower(
                publisher=self.publisher,
                arc_steering=arc,
                action_scale=cfg.action_scale,
                max_waypoint_advance=cfg.max_waypoint_advance,
                waypoint_tolerance=cfg.waypoint_tolerance,
                max_action_age=cfg.max_action_age,
                recompute=cfg.recompute,
            )

        # --- wheel odometry (pose source for the follower) ------------------
        self.odometry = WheelOdometry(
            wheel_diameter_m=cfg.wheel_diameter_mm / 1000.0,
            track_width_m=cfg.track_width_mm / 1000.0,
            encoder_ppr=cfg.encoder_ppr,
            on_pose=self._on_pose,
        )
        self._wheel_seen = False
        self.callbacks.on_wheel = self._on_wheel
        self.callbacks.on_battery = self._on_battery
        self.callbacks.on_status = self._on_status
        # IMU is high-rate and unused by the control loop; ignore by default.

        # --- pose sources: one drives the follower, the other is ground truth -
        # pose_source picks the ACTIVE source (what the follower and pose_topic
        # get). The other source keeps running as the GROUND-TRUTH reference:
        # under `wheel` that's rover_vio's VIO pose off MQTT (the drift-
        # comparison case), under `vio` it's host wheel odometry. Ground truth is
        # republished to gt_pose_topic and/or logged, never fed to the follower.
        self._use_vio = cfg.pose_source == "vio"
        self._active_source = "vio" if self._use_vio else "wheel"
        self._gt_source = "wheel" if self._use_vio else "vio"
        self._gt_enabled = gt_capture_enabled(cfg)
        self._vio_seen = False

        # VIO is subscribed when it is either the active source or ground truth.
        want_vio = self._use_vio or (self._gt_source == "vio" and self._gt_enabled)

        # --- pose log (both sources, for offline drift analysis) ------------
        self.pose_log = None
        if cfg.pose_log_db:
            self.pose_log = PoseLog(cfg.pose_log_db, pose_source=cfg.pose_source,
                                    rate_limit=cfg.gt_pose_rate_limit)

        # --- inference MQTT side --------------------------------------------
        self.inference = InferenceClient(
            broker=cfg.broker, port=cfg.port,
            action_topic=cfg.action_topic, ctrl_topic=cfg.ctrl_topic,
            remote_topic=cfg.remote_topic,
            camera_topic=cfg.camera_topic,
            pose_topic=cfg.pose_topic if cfg.publish_pose else None,
            gt_pose_topic=cfg.gt_pose_topic if cfg.publish_gt_pose else None,
            battery_topic=cfg.battery_topic if cfg.publish_battery else None,
            vio_pose_topic=cfg.vio_pose_topic if want_vio else None,
            on_vio_pose=self._on_vio_pose if want_vio else None,
            goal_topic=cfg.goal_topic or None,
            on_goal=self._on_goal,
            publisher=self.publisher, follower=self.follower,
            action_scale=cfg.action_scale,
        )

        # Pose streaming bookkeeping (rate-limited in _publish_pose/_publish_gt_pose).
        self._pose_seq = 0
        self._last_pose_pub = None
        self._gt_pose_seq = 0
        self._last_gt_pose_pub = None

        # --- camera ---------------------------------------------------------
        self.camera = None
        if cfg.camera_topic and not cfg.no_camera:
            self.camera = make_camera(
                cfg.camera, publish=self.inference.publish_camera,
                rate_limit=cfg.rate_limit,
                width=cfg.width, height=cfg.height, fps=cfg.fps,
                crop_mode=cfg.crop_mode, crop_top_fraction=cfg.crop_top_fraction,
                rotate=cfg.rotate,
            )

    # --- pose / telemetry handlers -----------------------------------------

    def _on_wheel(self, telem: wire.WheelTelem) -> None:
        if not self._wheel_seen:
            self._wheel_seen = True
            log.info("first wheel telemetry received (seq=%d, ticks L=%d R=%d) — "
                     "odometry + pose streaming active",
                     telem.seq, telem.left_ticks, telem.right_ticks)
        self.odometry.update(telem)

    def _on_pose(self, x: float, y: float, yaw: float) -> None:
        """Wheel-odometry pose callback. Drives the follower when wheel is the
        active source; otherwise wheel odometry is the ground-truth stream."""
        if self._use_vio:
            self._consume_gt_pose("wheel", x, y, yaw)
            return
        self._consume_pose(x, y, yaw, publish=self.cfg.publish_pose)

    def _on_vio_pose(self, payload: dict) -> None:
        """External VIO pose (rover_vio) callback.

        Drives the follower under ``pose_source: vio``; under ``pose_source:
        wheel`` the same stream is recorded as ground truth instead, so an
        experiment can run on encoders while VIO measures how far they drifted.
        """
        try:
            x, y, yaw = pose_from_stamped_dict(payload)
        except (KeyError, TypeError, ValueError) as e:
            log_throttle(log, logging.WARNING, 5.0, f"bad VIO pose payload: {e}")
            return
        if not self._vio_seen:
            self._vio_seen = True
            role = "active pose source" if self._use_vio else "ground truth"
            log.info("first VIO pose received on %s — using it as %s",
                     self.cfg.vio_pose_topic, role)
        if self._use_vio:
            self._consume_pose(x, y, yaw, publish=self.cfg.publish_pose)
        else:
            self._consume_gt_pose("vio", x, y, yaw)

    def _on_goal(self, payload: dict) -> None:
        """Observe ``gemnav/goal`` so a recorded run knows its goal.

        The bridge never acts on this: the inference server subscribes to the
        same retained message and re-projects the goal to body frame itself.
        Arrives once on connect (it is retained), plus on every change.
        """
        goal = redact_goal(payload)
        log.info("goal on %s: %s", self.cfg.goal_topic, describe_goal(goal))
        if self.pose_log:
            self.pose_log.record_goal(json.dumps(goal))

    def _consume_pose(self, x: float, y: float, yaw: float, publish: bool) -> None:
        """Handle the ACTIVE pose: steer on it, show it, publish it, log it."""
        if self.follower:
            self.follower.update_pose(x, y, yaw)
        if self.pose_log:
            self.pose_log.record(self._active_source, x, y, yaw, active=True)
        if self.cfg.publish_display:
            # Feed the rover's OLED its host-authoritative pose.
            self.transport.send_cmd_display(x, y, yaw, 0.0, 0.0)
        if publish:
            self._publish_pose(x, y, yaw)

    def _consume_gt_pose(self, source: str, x: float, y: float, yaw: float) -> None:
        """Handle the GROUND-TRUTH pose: log and/or republish, never steer on it.

        Deliberately does not touch the follower or the OLED — the whole point is
        that it observes the run without influencing it.
        """
        if self.pose_log:
            self.pose_log.record(source, x, y, yaw, active=False)
        if not (self.cfg.publish_gt_pose and self.cfg.gt_pose_topic):
            return
        limit = self.cfg.gt_pose_rate_limit
        if limit is not None:
            now = time.monotonic()
            if self._last_gt_pose_pub is not None and \
                    (now - self._last_gt_pose_pub) < 1.0 / limit:
                return
            self._last_gt_pose_pub = now
        msg = pose_stamped_dict(x, y, yaw, frame_id=self.cfg.pose_frame_id,
                                seq=self._gt_pose_seq)
        self._gt_pose_seq += 1
        self.inference.publish_gt_pose(json.dumps(msg))
        log_throttle(log, logging.INFO, 5.0,
                     f"streaming ground truth ({source}) -> "
                     f"{self.cfg.gt_pose_topic}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def _publish_pose(self, x: float, y: float, yaw: float) -> None:
        """Stream odometry pose to MQTT (PoseStamped JSON), rate-limited.

        Wheel telemetry arrives at ~50 Hz; pose_rate_limit caps how often we
        forward it (null = every sample), mirroring ros_ws's per-topic cap.
        """
        limit = self.cfg.pose_rate_limit
        if limit is not None:
            now = time.monotonic()
            if self._last_pose_pub is not None and (now - self._last_pose_pub) < 1.0 / limit:
                return
            self._last_pose_pub = now
        msg = pose_stamped_dict(x, y, yaw, frame_id=self.cfg.pose_frame_id,
                                seq=self._pose_seq)
        self._pose_seq += 1
        self.inference.publish_pose(json.dumps(msg))
        log_throttle(log, logging.INFO, 5.0,
                     f"streaming pose -> {self.cfg.pose_topic}: "
                     f"x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def _on_battery(self, telem: wire.BatteryTelem) -> None:
        pct = percent_from_voltage(telem.voltage_v, cells=self.cfg.battery_cells)
        if self.cfg.publish_battery:
            msg = {
                "data": round(pct, 1),          # charge % (matches ros_ws Float32 shape)
                "voltage_v": round(telem.voltage_v, 3),
                "current_a": round(telem.current_a, 3),
                "cells": self.cfg.battery_cells,
            }
            self.inference.publish_battery(json.dumps(msg))
        log_throttle(log, logging.INFO, 10.0,
                     f"battery {telem.voltage_v:.2f} V {telem.current_a:+.2f} A "
                     f"(~{pct:.0f}%, {self.cfg.battery_cells}S)")

    def _on_status(self, status: str) -> None:
        log.info("rover status: %s", status)

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        log.info("starting rover bridge: transport=%s camera=%s broker=%s id=%s",
                 self.cfg.transport, self.cfg.camera, self.cfg.broker, self.cfg.robot_id)
        self.transport.start()
        self.inference.connect()
        if self.camera:
            if not self.camera.start():
                log.warning("camera failed to start; continuing without it")
                self.camera = None
        log.info("rover bridge running")

    def shutdown(self) -> None:
        log.info("shutting down rover bridge ...")
        if self.camera:
            self.camera.stop()
        self.inference.disconnect()
        self.publisher.stop()
        self.transport.stop()  # sends a final zero cmd_vel
        if self.pose_log:
            self.pose_log.close()
        log.info("rover bridge stopped")
