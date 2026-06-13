# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Transport abstraction shared by the UART and MQTT rover links."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional

from .. import wire


@dataclass
class TelemetryCallbacks:
    """Hooks the transport invokes when it decodes inbound telemetry.

    All are optional. The bridge wires ``on_wheel`` to wheel odometry, and the
    others to logging/diagnostics. Callbacks run on the transport's own RX
    thread, so keep them quick and thread-safe.
    """

    on_wheel: Optional[Callable[[wire.WheelTelem], None]] = None
    on_imu: Optional[Callable[[wire.ImuTelem], None]] = None
    on_battery: Optional[Callable[[wire.BatteryTelem], None]] = None
    on_status: Optional[Callable[[str], None]] = None

    def dispatch(self, pkt_type: int, payload: bytes) -> None:
        """Decode a raw (type, payload) pair and fire the matching callback.

        Shared by both transports so the bot→host parsing lives in exactly one
        place. Unknown/cmd types are ignored (the bot only emits telemetry).
        """
        if pkt_type == wire.PKT_TEL_WHEEL:
            if self.on_wheel:
                self.on_wheel(wire.unpack_wheel(payload))
        elif pkt_type == wire.PKT_TEL_IMU:
            if self.on_imu:
                self.on_imu(wire.unpack_imu(payload))
        elif pkt_type == wire.PKT_TEL_BATT:
            if self.on_battery:
                self.on_battery(wire.unpack_battery(payload))
        elif pkt_type == wire.PKT_STATUS:
            if self.on_status:
                self.on_status(payload.decode("ascii", errors="replace"))


class RoverTransport(ABC):
    """Bidirectional link to the rover. Subclasses own their I/O thread."""

    @abstractmethod
    def start(self) -> None:
        """Open the link and begin receiving telemetry. Raises on failure."""

    @abstractmethod
    def stop(self) -> None:
        """Send a final zero cmd_vel (best effort) and close the link."""

    @abstractmethod
    def send_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """Send a velocity command. Also serves as the firmware heartbeat."""

    @abstractmethod
    def send_cmd_pid(self, kp: float, ki: float, kd: float,
                     output_clamp: float, deadband: float) -> None:
        """Push live PID gains to the rover."""

    @abstractmethod
    def send_cmd_display(self, x: float, y: float, yaw: float,
                         pitch: float, roll: float) -> None:
        """Push host pose to the rover's OLED feed."""
