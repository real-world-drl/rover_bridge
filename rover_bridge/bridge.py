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
"""

from __future__ import annotations

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
from .transports import TelemetryCallbacks, make_transport

log = get_logger("bridge")


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

        # --- pose source: wheel odometry (default) or external VIO ----------
        # With pose_source=vio the bridge consumes rover_vio's pose off MQTT and
        # feeds it to the follower in place of wheel odometry; rover_vio owns the
        # pose topic, so the bridge stops publishing wheel pose to it.
        self._use_vio = cfg.pose_source == "vio"
        self._vio_seen = False

        # --- inference MQTT side --------------------------------------------
        self.inference = InferenceClient(
            broker=cfg.broker, port=cfg.port,
            action_topic=cfg.action_topic, ctrl_topic=cfg.ctrl_topic,
            remote_topic=cfg.remote_topic,
            camera_topic=cfg.camera_topic,
            pose_topic=cfg.pose_topic if (cfg.publish_pose and not self._use_vio) else None,
            battery_topic=cfg.battery_topic if cfg.publish_battery else None,
            vio_pose_topic=cfg.vio_pose_topic if self._use_vio else None,
            on_vio_pose=self._on_vio_pose if self._use_vio else None,
            publisher=self.publisher, follower=self.follower,
            action_scale=cfg.action_scale,
        )

        # Pose streaming bookkeeping (rate-limited in _publish_pose).
        self._pose_seq = 0
        self._last_pose_pub = None

        # --- camera ---------------------------------------------------------
        self.camera = None
        if cfg.camera_topic and not cfg.no_camera:
            self.camera = make_camera(
                cfg.camera, publish=self.inference.publish_camera,
                rate_limit=cfg.rate_limit,
                width=cfg.width, height=cfg.height, fps=cfg.fps,
                crop_mode=cfg.crop_mode, crop_top_fraction=cfg.crop_top_fraction,
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
        """Wheel-odometry pose callback. Drives the follower only when it is the
        active source (wheel odometry still integrates in vio mode, harmlessly)."""
        if self._use_vio:
            return
        self._consume_pose(x, y, yaw, publish=self.cfg.publish_pose)

    def _on_vio_pose(self, payload: dict) -> None:
        """External VIO pose (rover_vio) callback, used when pose_source=vio."""
        try:
            x, y, yaw = pose_from_stamped_dict(payload)
        except (KeyError, TypeError, ValueError) as e:
            log_throttle(log, logging.WARNING, 5.0, f"bad VIO pose payload: {e}")
            return
        if not self._vio_seen:
            self._vio_seen = True
            log.info("first VIO pose received on %s — pose_source=vio active",
                     self.cfg.vio_pose_topic)
        # rover_vio already publishes this pose; the bridge only consumes it.
        self._consume_pose(x, y, yaw, publish=False)

    def _consume_pose(self, x: float, y: float, yaw: float, publish: bool) -> None:
        if self.follower:
            self.follower.update_pose(x, y, yaw)
        if self.cfg.publish_display:
            # Feed the rover's OLED its host-authoritative pose.
            self.transport.send_cmd_display(x, y, yaw, 0.0, 0.0)
        if publish:
            self._publish_pose(x, y, yaw)

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
        log.info("rover bridge stopped")
