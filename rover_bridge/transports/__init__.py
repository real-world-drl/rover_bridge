# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Rover-link transports: how cmd_vel reaches the rover and telemetry returns.

The inference side of the bridge is always MQTT; only this link is selectable.
``--transport uart`` (default) uses the tethered serial link; ``--transport
mqtt`` uses the rover's MQTT contract. Both implement :class:`RoverTransport`.
"""

from .base import RoverTransport, TelemetryCallbacks
from .uart import UartTransport
from .mqtt import MqttRoverTransport


def make_transport(kind: str, callbacks: TelemetryCallbacks, **kwargs) -> RoverTransport:
    """Factory. ``kind`` is ``"uart"`` or ``"mqtt"``."""
    kind = kind.lower()
    if kind == "uart":
        return UartTransport(callbacks, **kwargs)
    if kind == "mqtt":
        return MqttRoverTransport(callbacks, **kwargs)
    raise ValueError(f"unknown rover transport {kind!r} (expected 'uart' or 'mqtt')")


__all__ = [
    "RoverTransport",
    "TelemetryCallbacks",
    "UartTransport",
    "MqttRoverTransport",
    "make_transport",
]
