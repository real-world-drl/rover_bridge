# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""MQTT rover transport — bare packed structs on the ``ugv/<id>/v1/...`` topics.

This is the alternative to the UART link (``--transport mqtt``). It is a
*separate* MQTT client from the inference side (``inference.py``), even when
pointed at the same broker, so the two concerns stay decoupled and can use
different brokers if needed. Payloads are the raw little-endian structs from
:mod:`rover_bridge.wire` — no UART framing, no JSON.
"""

from __future__ import annotations

import paho.mqtt.client as mqtt

from .. import wire
from ..logging_util import get_logger
from .base import RoverTransport, TelemetryCallbacks

log = get_logger("transport.mqtt")


class MqttRoverTransport(RoverTransport):
    def __init__(self, callbacks: TelemetryCallbacks,
                 broker: str = "localhost", port: int = 1883,
                 robot_id: str = "ugv01", keepalive: int = 10):
        self._callbacks = callbacks
        self._broker = broker
        self._port = port
        self._robot_id = robot_id
        self._keepalive = keepalive
        self._client: mqtt.Client | None = None

        # Precompute outbound topics.
        self._t_cmd_vel = wire.topic(robot_id, wire.TOPIC_CMD_VEL)
        self._t_cmd_pid = wire.topic(robot_id, wire.TOPIC_CMD_PID)
        self._t_cmd_display = wire.topic(robot_id, wire.TOPIC_CMD_DISPLAY)

        # Inbound telemetry topic -> packet type.
        self._tel_types = {
            wire.topic(robot_id, wire.TOPIC_TEL_WHEEL): wire.PKT_TEL_WHEEL,
            wire.topic(robot_id, wire.TOPIC_TEL_IMU): wire.PKT_TEL_IMU,
            wire.topic(robot_id, wire.TOPIC_TEL_BATT): wire.PKT_TEL_BATT,
            wire.topic(robot_id, wire.TOPIC_STATUS): wire.PKT_STATUS,
        }

    def start(self) -> None:
        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        log.info("connecting rover MQTT transport to %s:%d (id=%s)",
                 self._broker, self._port, self._robot_id)
        self._client.connect(self._broker, self._port, self._keepalive)
        self._client.loop_start()

    def stop(self) -> None:
        try:
            self.send_cmd_vel(0.0, 0.0)
        except Exception:
            pass
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
        log.info("rover MQTT transport stopped")

    # --- TX -----------------------------------------------------------------

    def send_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if self._client:
            self._client.publish(self._t_cmd_vel, wire.pack_cmd_vel(linear_x, angular_z),
                                 qos=1, retain=False)

    def send_cmd_pid(self, kp, ki, kd, output_clamp, deadband) -> None:
        if self._client:
            # Retained, matching the firmware contract (gains persist across reboots).
            self._client.publish(self._t_cmd_pid,
                                 wire.pack_cmd_pid(kp, ki, kd, output_clamp, deadband),
                                 qos=1, retain=True)

    def send_cmd_display(self, x, y, yaw, pitch, roll) -> None:
        if self._client:
            self._client.publish(self._t_cmd_display,
                                 wire.pack_cmd_display(x, y, yaw, pitch, roll),
                                 qos=1, retain=False)

    # --- RX -----------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            for t in self._tel_types:
                client.subscribe(t, qos=0)
            log.info("rover MQTT transport connected; subscribed to %d telemetry topic(s)",
                     len(self._tel_types))
        else:
            log.error("rover MQTT transport failed to connect (rc=%s)", rc)

    def _on_message(self, client, userdata, msg):
        pkt_type = self._tel_types.get(msg.topic)
        if pkt_type is None:
            return
        try:
            self._callbacks.dispatch(pkt_type, msg.payload)
        except Exception as e:
            log.error("error dispatching %s on %s: %s",
                      wire.NAME_BY_TYPE.get(pkt_type, pkt_type), msg.topic, e)
