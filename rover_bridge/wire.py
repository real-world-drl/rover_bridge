# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""RoverLink wire contract: packing, unpacking, CRC8 and UART framing.

Hand-mirrored from ``RoverLink/main/ugv_packets.h`` (and its Python sibling
``RoverLink/tools/ugv_packets.py``). This is the single source of truth on the
host side for both transports — the MQTT transport publishes/parses the bare
packed structs, the UART transport wraps/unwraps them in the framing below.

Little-endian throughout. Sizes are guarded with assertions so any drift from
the firmware structs fails loudly at import time rather than silently
misaligning at runtime (the firmware itself has matching ``_Static_assert``s).

Topic conventions (with ``<id>`` = ``CONFIG_UGV_ROBOT_ID``)::

    ugv/<id>/v1/cmd/vel      host -> bot,  QoS 1, not retained
    ugv/<id>/v1/cmd/pid      host -> bot,  QoS 1, retained
    ugv/<id>/v1/cmd/display  host -> bot,  QoS 1, not retained
    ugv/<id>/v1/tel/wheel    bot  -> host, QoS 0
    ugv/<id>/v1/tel/imu      bot  -> host, QoS 0
    ugv/<id>/v1/tel/battery  bot  -> host, QoS 0
    ugv/<id>/v1/status       bot  -> host, QoS 1, retained (LWT)
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional

# --- Topic structure -------------------------------------------------------

TOPIC_VERSION = "v1"
TOPIC_CMD_VEL = "cmd/vel"
TOPIC_CMD_PID = "cmd/pid"
TOPIC_CMD_DISPLAY = "cmd/display"
TOPIC_TEL_WHEEL = "tel/wheel"
TOPIC_TEL_IMU = "tel/imu"
TOPIC_TEL_BATT = "tel/battery"
TOPIC_STATUS = "status"

STATUS_ONLINE = "online"
STATUS_OFFLINE = "offline"


def topic(robot_id: str, leaf: str) -> str:
    """Full MQTT topic, e.g. ``topic("ugv01", TOPIC_CMD_VEL)``."""
    return f"ugv/{robot_id}/{TOPIC_VERSION}/{leaf}"


# --- UART framing ----------------------------------------------------------
# Frame on the wire: [SYNC] [type:1] [len:1] [payload:len] [crc8:1]
# CRC8 covers (type, len, payload); poly 0x07, init 0x00.
UART_SYNC = 0xA5

# --- Packet type IDs (must match ugv_pkt_type_t in firmware) --------------
PKT_CMD_VEL = 0x01
PKT_CMD_PID = 0x02
PKT_CMD_DISPLAY = 0x03
PKT_TEL_WHEEL = 0x10
PKT_TEL_IMU = 0x11
PKT_TEL_BATT = 0x12
PKT_STATUS = 0x20

# --- Struct formats (must match the packed structs in firmware) ----------
FMT_CMD_VEL = "<Qff"          # u64 host_ts_us, f linear_x, f angular_z
FMT_CMD_PID = "<fffff"        # f kp, ki, kd, output_clamp, deadband
FMT_CMD_DISPLAY = "<Qfffff"   # u64 host_ts_us, f x, y, yaw, pitch, roll
FMT_TEL_WHEEL = "<QIiiffff"   # u64 ts, u32 seq, i32 lt, i32 rt, f lv, rv, lset, rset
FMT_TEL_IMU = "<QI" + "f" * 10 + "Bxxx"  # u64 ts, u32 seq, 10 floats, u8 mag_fresh, 3 pad
FMT_TEL_BATT = "<Qff"         # u64 ts, f voltage_v, f current_a

SIZE_BY_TYPE = {
    PKT_CMD_VEL: struct.calcsize(FMT_CMD_VEL),          # 16
    PKT_CMD_PID: struct.calcsize(FMT_CMD_PID),          # 20
    PKT_CMD_DISPLAY: struct.calcsize(FMT_CMD_DISPLAY),  # 28
    PKT_TEL_WHEEL: struct.calcsize(FMT_TEL_WHEEL),      # 36
    PKT_TEL_IMU: struct.calcsize(FMT_TEL_IMU),          # 56
    PKT_TEL_BATT: struct.calcsize(FMT_TEL_BATT),        # 16
    PKT_STATUS: None,                                   # variable-length string
}

NAME_BY_TYPE = {
    PKT_CMD_VEL: "cmd_vel",
    PKT_CMD_PID: "cmd_pid",
    PKT_CMD_DISPLAY: "cmd_display",
    PKT_TEL_WHEEL: "tel_wheel",
    PKT_TEL_IMU: "tel_imu",
    PKT_TEL_BATT: "tel_battery",
    PKT_STATUS: "status",
}

# Guard against drift from the firmware contract (matches the _Static_asserts).
assert SIZE_BY_TYPE[PKT_CMD_VEL] == 16, "ugv_cmd_vel_t size drift"
assert SIZE_BY_TYPE[PKT_CMD_PID] == 20, "ugv_cmd_pid_t size drift"
assert SIZE_BY_TYPE[PKT_CMD_DISPLAY] == 28, "ugv_cmd_display_t size drift"
assert SIZE_BY_TYPE[PKT_TEL_WHEEL] == 36, "ugv_wheel_telem_t size drift"
assert SIZE_BY_TYPE[PKT_TEL_IMU] == 56, "ugv_imu_telem_t size drift"
assert SIZE_BY_TYPE[PKT_TEL_BATT] == 16, "ugv_battery_telem_t size drift"


# --- Decoded telemetry structures -----------------------------------------

@dataclass
class WheelTelem:
    device_timestamp_us: int
    seq: int
    left_ticks: int
    right_ticks: int
    left_velocity_mps: float
    right_velocity_mps: float
    left_setpoint_mps: float
    right_setpoint_mps: float


@dataclass
class ImuTelem:
    device_timestamp_us: int
    seq: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float
    temp_c: float
    mag_fresh: bool


@dataclass
class BatteryTelem:
    device_timestamp_us: int
    voltage_v: float
    current_a: float


# --- CRC8 (poly 0x07, init 0x00) ------------------------------------------

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def frame(pkt_type: int, payload: bytes) -> bytes:
    """Wrap a packed-struct payload in the UART framing."""
    body = bytes([pkt_type, len(payload)]) + payload
    return bytes([UART_SYNC]) + body + bytes([crc8(body)])


# --- Command packing (host -> bot) ----------------------------------------

def _now_us() -> int:
    return time.time_ns() // 1000


def pack_cmd_vel(linear_x: float, angular_z: float, host_ts_us: Optional[int] = None) -> bytes:
    """ugv_cmd_vel_t payload. linear_x m/s (+forward), angular_z rad/s (+CCW)."""
    if host_ts_us is None:
        host_ts_us = _now_us()
    return struct.pack(FMT_CMD_VEL, host_ts_us, float(linear_x), float(angular_z))


def pack_cmd_pid(kp: float, ki: float, kd: float,
                 output_clamp: float, deadband: float) -> bytes:
    """ugv_cmd_pid_t payload."""
    return struct.pack(FMT_CMD_PID, float(kp), float(ki), float(kd),
                       float(output_clamp), float(deadband))


def pack_cmd_display(x: float, y: float, yaw: float, pitch: float, roll: float,
                     host_ts_us: Optional[int] = None) -> bytes:
    """ugv_cmd_display_t payload (host pose feed for the OLED)."""
    if host_ts_us is None:
        host_ts_us = _now_us()
    return struct.pack(FMT_CMD_DISPLAY, host_ts_us, float(x), float(y),
                       float(yaw), float(pitch), float(roll))


# --- Telemetry unpacking (bot -> host) ------------------------------------

def unpack_wheel(payload: bytes) -> WheelTelem:
    return WheelTelem(*struct.unpack(FMT_TEL_WHEEL, payload))


def unpack_imu(payload: bytes) -> ImuTelem:
    (ts, seq, ax, ay, az, gx, gy, gz, mx, my, mz,
     temp_c, mag_fresh) = struct.unpack(FMT_TEL_IMU, payload)
    return ImuTelem(ts, seq, ax, ay, az, gx, gy, gz, mx, my, mz,
                    temp_c, bool(mag_fresh))


def unpack_battery(payload: bytes) -> BatteryTelem:
    return BatteryTelem(*struct.unpack(FMT_TEL_BATT, payload))


# --- Streaming UART frame decoder -----------------------------------------

class FrameDecoder:
    """Byte-fed FSM that mirrors ``uart_link.c``'s RX state machine.

    Feed it raw serial bytes; it invokes ``on_packet(pkt_type, payload)`` for
    every CRC-valid frame. Counters (``ok``, ``bad_crc``, ``bad_len``,
    ``bad_type``) match the firmware's diagnostic names so bring-up logs line
    up across the two ends of the link.
    """

    def __init__(self, on_packet: Callable[[int, bytes], None]):
        self._on_packet = on_packet
        self._state = "WAIT_SYNC"
        self._type = 0
        self._len: Optional[int] = 0
        self._payload = bytearray()
        self.stats = {"ok": 0, "bad_crc": 0, "bad_len": 0, "bad_type": 0}

    def feed(self, data: bytes) -> None:
        for b in data:
            self._step(b)

    def _step(self, b: int) -> None:
        state = self._state

        if state == "WAIT_SYNC":
            if b == UART_SYNC:
                self._state = "READ_TYPE"

        elif state == "READ_TYPE":
            expected = SIZE_BY_TYPE.get(b)
            # Status is variable-length; accept any len for it. An unknown type
            # (None expected and not STATUS) is a framing error.
            if expected is None and b != PKT_STATUS:
                self.stats["bad_type"] += 1
                self._state = "WAIT_SYNC"
                return
            self._type = b
            self._len = expected  # may be None for STATUS (finalized at READ_LEN)
            self._state = "READ_LEN"

        elif state == "READ_LEN":
            if self._len is not None and b != self._len:
                self.stats["bad_len"] += 1
                self._state = "WAIT_SYNC"
                return
            self._len = b  # finalize length (covers STATUS too)
            self._payload = bytearray()
            self._state = "READ_PAYLOAD" if self._len > 0 else "READ_CRC"

        elif state == "READ_PAYLOAD":
            self._payload.append(b)
            if len(self._payload) >= self._len:
                self._state = "READ_CRC"

        elif state == "READ_CRC":
            body = bytes([self._type, self._len]) + bytes(self._payload)
            if crc8(body) != b:
                self.stats["bad_crc"] += 1
            else:
                self.stats["ok"] += 1
                self._on_packet(self._type, bytes(self._payload))
            self._state = "WAIT_SYNC"
