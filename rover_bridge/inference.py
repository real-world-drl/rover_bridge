# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Inference-side MQTT client and action dispatch.

This is the "always MQTT" half of the bridge, ported from ``ros_ws``'s
``mqtt_client_manager.py``. It owns one MQTT client that:

- publishes preprocessed camera frames to ``camera_topic`` (the model's input),
- subscribes to ``action_topic`` (``omnivla/act``) for inference trajectories,
- subscribes to ``ctrl_topic`` (``omnivla/ctrl``) for out-of-band stop/start,
- subscribes to ``remote_topic`` (``omnivla/remote``) for manual teleop.

Action payloads are JSON, identical to the Spot setup since it's the same
model: ``{"waypoints": [[x, y, sin, cos], ...]}`` (preferred) or a raw
``{"linear": .., "angular": ..}`` fallback. The bridge converts those to
cmd_vel via the waypoint follower / arc steering and the repeated publisher;
the rover transport (UART or MQTT) is downstream of all of that.

The stop/start halt state is sticky on purpose: after ``{"stop": true}`` the
client drops every action message until ``{"start": true}`` arrives, so a
still-streaming inference client can't leak motion back in after a manual stop.

Remote teleop is the deliberate exception: ``remote_topic`` accepts
``{"linear": .., "angular": ..}`` and moves the rover *even while halted* — it
bypasses the halt state without changing it, so an operator can drive manually
while inference actuation stays stopped.
"""

from __future__ import annotations

import json
import logging
import threading
from typing import Optional

import paho.mqtt.client as mqtt

from .control import CmdVel, RepeatedCmdVelPublisher, WaypointFollower
from .logging_util import get_logger, log_throttle

log = get_logger("inference")


class InferenceClient:
    def __init__(self, broker: str, port: int = 1883, keepalive: int = 60,
                 action_topic: str = "omnivla/act", ctrl_topic: str = "omnivla/ctrl",
                 remote_topic: Optional[str] = "omnivla/remote",
                 camera_topic: Optional[str] = None, pose_topic: Optional[str] = None,
                 battery_topic: Optional[str] = None,
                 publisher: Optional[RepeatedCmdVelPublisher] = None,
                 follower: Optional[WaypointFollower] = None,
                 action_scale: float = 1.0):
        """
        Args:
            broker/port/keepalive: MQTT connection to the inference broker.
            action_topic: topic carrying inference trajectories.
            ctrl_topic: topic carrying ``{"stop": true}`` / ``{"start": true}``.
            remote_topic: topic carrying manual teleop ``{"linear": ..,
                "angular": ..}``. These move the rover even while halted (waiting
                for a ``{"start": true}``) — they bypass the inference halt state
                without changing it. None disables it.
            camera_topic: topic this bridge publishes camera JPEGs to (the
                model's input). The inference client must subscribe to the same.
            pose_topic: topic this bridge publishes the rover's odometry pose to
                (PoseStamped-shaped JSON, matching ros_ws). None disables it.
            battery_topic: topic this bridge publishes battery charge percentage
                to (``{"data": pct, ...}`` JSON, matching ros_ws's Float32
                charge_percentage). None disables it.
            publisher: RepeatedCmdVelPublisher for the raw-velocity fallback and
                stop commands.
            follower: WaypointFollower for waypoint trajectories (preferred).
            action_scale: multiplier on raw linear/angular (fallback path only;
                waypoint-derived velocities are scaled inside the follower).
        """
        self.broker = broker
        self.port = port
        self.keepalive = keepalive
        self.action_topic = action_topic
        self.ctrl_topic = ctrl_topic
        self.remote_topic = remote_topic
        self.camera_topic = camera_topic
        self.pose_topic = pose_topic
        self.battery_topic = battery_topic
        self.publisher = publisher
        self.follower = follower
        self.action_scale = action_scale

        self.client: Optional[mqtt.Client] = None
        self.halted = False
        self.halt_lock = threading.Lock()

    # --- lifecycle ----------------------------------------------------------

    def connect(self) -> mqtt.Client:
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        log.info("connecting inference MQTT to %s:%d", self.broker, self.port)
        self.client.connect(self.broker, self.port, self.keepalive)
        self.client.loop_start()
        return self.client

    def disconnect(self) -> None:
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            log.info("inference MQTT disconnected")

    def publish_camera(self, jpeg: bytes) -> None:
        """Publish a preprocessed camera frame. Wired to the camera backend."""
        if self.client and self.camera_topic:
            self.client.publish(self.camera_topic, jpeg, qos=0)

    def publish_pose(self, pose_json: str) -> None:
        """Publish a pose message (JSON string). Wired to wheel odometry."""
        if self.client and self.pose_topic:
            self.client.publish(self.pose_topic, pose_json, qos=0)

    def publish_battery(self, battery_json: str) -> None:
        """Publish a battery message (JSON string). Wired to battery telemetry."""
        if self.client and self.battery_topic:
            self.client.publish(self.battery_topic, battery_json, qos=0)

    # --- MQTT callbacks -----------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            log.info("inference MQTT connected")
            client.subscribe(self.action_topic)
            log.info("subscribed to action topic: %s", self.action_topic)
            if self.ctrl_topic:
                client.subscribe(self.ctrl_topic)
                log.info("subscribed to ctrl topic: %s", self.ctrl_topic)
            if self.remote_topic:
                client.subscribe(self.remote_topic)
                log.info("subscribed to remote topic: %s (moves even while halted)",
                         self.remote_topic)
        else:
            log.error("inference MQTT failed to connect (rc=%s)", rc)

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            log.warning("unexpected inference MQTT disconnect (rc=%s); reconnecting", rc)

    def _on_message(self, client, userdata, msg):
        try:
            if self.ctrl_topic and msg.topic == self.ctrl_topic:
                self._handle_ctrl(msg)
            elif self.remote_topic and msg.topic == self.remote_topic:
                self._handle_remote(msg)
            else:
                self._handle_action(msg)
        except Exception as e:
            log.error("error processing MQTT message on %s: %s", msg.topic, e)

    # --- ctrl topic ---------------------------------------------------------

    def _handle_ctrl(self, msg):
        if not self.publisher:
            log.warning("ctrl message received but no publisher configured")
            return
        try:
            payload = json.loads(msg.payload)
        except (json.JSONDecodeError, ValueError):
            payload = msg.payload.decode("utf-8", errors="replace").strip()

        if isinstance(payload, dict) and payload.get("stop") is True:
            with self.halt_lock:
                self.halted = True
            log.info("STOP received on ctrl topic — halting; dropping actions until "
                     '{"start": true}')
            # Route through the follower (when present) so trajectory state is
            # cleared too; otherwise publish a bare zero.
            if self.follower:
                self.follower.force_stop()
            else:
                self.publisher.publish(CmdVel())
        elif isinstance(payload, dict) and payload.get("start") is True:
            with self.halt_lock:
                was_halted = self.halted
                self.halted = False
            if was_halted:
                log.info("START received on ctrl topic — resuming on next action")
            else:
                log.info("START received on ctrl topic — already running (no-op)")
        else:
            log.warning("unrecognized ctrl payload (no action taken): %r", payload)

    # --- action topic -------------------------------------------------------

    def _handle_action(self, msg):
        if not self.publisher:
            log.warning("action message received but no publisher configured")
            return

        with self.halt_lock:
            halted = self.halted
        if halted:
            log_throttle(log, logging.INFO, 2.0,
                         'halted (waiting for {"start": true}) — ignoring action')
            return

        try:
            payload = json.loads(msg.payload)
        except json.JSONDecodeError as e:
            log.error("failed to parse action message as JSON: %s", e)
            return

        # Preferred: waypoint trajectory through the follower (handles the
        # initial command plus pose-driven advance + recompute).
        if self.follower and "waypoints" in payload:
            self.follower.on_new_action(payload["waypoints"])
            return

        # Fallback: raw linear/angular with action_scale.
        cmd = CmdVel()
        if "linear" in payload:
            cmd.linear_x = payload["linear"] * self.action_scale
        if "angular" in payload:
            cmd.angular_z = payload["angular"] * self.action_scale
        self.publisher.publish(cmd)

    # --- remote topic -------------------------------------------------------

    def _handle_remote(self, msg):
        """Handle a manual teleop message: ``{"linear": .., "angular": ..}`` -> cmd_vel.

        Unlike action messages, remote commands ignore the halt state so an
        operator can drive the rover while inference actuation is stopped
        (waiting for a ``{"start": true}``). They do NOT change the halt state —
        once the remote command's publisher buffer expires the rover stops and
        inference stays halted. Send a steady stream to keep driving.
        """
        if not self.publisher:
            log.warning("remote message received but no publisher configured")
            return

        try:
            payload = json.loads(msg.payload)
        except json.JSONDecodeError as e:
            log.error("failed to parse remote message as JSON: %s", e)
            return

        if not isinstance(payload, dict):
            log.warning("unrecognized remote payload (expected JSON object): %r", payload)
            return

        cmd = CmdVel(
            linear_x=float(payload.get("linear", 0.0)) * self.action_scale,
            angular_z=float(payload.get("angular", 0.0)) * self.action_scale,
        )

        # Clear any in-flight waypoint trajectory so a pose update can't override
        # the remote command via WaypointFollower._maybe_advance. force_stop
        # pushes a zero cmd_vel, but the publish below immediately supersedes it.
        if self.follower:
            self.follower.force_stop()
        self.publisher.publish(cmd)
