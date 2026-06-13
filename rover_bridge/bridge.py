# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Top-level orchestrator: wires transport, odometry, control, inference, camera.

Data flow (one direction of the loop each):

    camera ─preprocess─▶ inference.publish_camera ─MQTT▶ model
    model ─MQTT(omnivla/act)▶ inference ─▶ waypoint follower / arc steering
        ─▶ RepeatedCmdVelPublisher ─▶ transport.send_cmd_vel ─UART/MQTT▶ rover
    rover ─tel/wheel─▶ transport ─▶ wheel odometry ─pose─▶ waypoint follower

The rover transport is selectable (UART default, MQTT alternative); everything
on the inference side is MQTT regardless.
"""

from __future__ import annotations

import logging
from types import SimpleNamespace

from . import wire
from .cameras import make_camera
from .control import ArcSteering, RepeatedCmdVelPublisher, WaypointFollower
from .inference import InferenceClient
from .logging_util import get_logger, log_throttle
from .odometry import WheelOdometry
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
        self.callbacks.on_wheel = self.odometry.update
        self.callbacks.on_battery = self._on_battery
        self.callbacks.on_status = self._on_status
        # IMU is high-rate and unused by the control loop; ignore by default.

        # --- inference MQTT side --------------------------------------------
        self.inference = InferenceClient(
            broker=cfg.broker, port=cfg.port,
            action_topic=cfg.action_topic, ctrl_topic=cfg.ctrl_topic,
            camera_topic=cfg.camera_topic,
            publisher=self.publisher, follower=self.follower,
            action_scale=cfg.action_scale,
        )

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

    def _on_pose(self, x: float, y: float, yaw: float) -> None:
        if self.follower:
            self.follower.update_pose(x, y, yaw)
        if self.cfg.publish_display:
            # Feed the rover's OLED its host-authoritative pose.
            self.transport.send_cmd_display(x, y, yaw, 0.0, 0.0)

    def _on_battery(self, telem: wire.BatteryTelem) -> None:
        log_throttle(log, logging.INFO, 10.0,
                     f"battery {telem.voltage_v:.2f} V {telem.current_a:+.2f} A")

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
