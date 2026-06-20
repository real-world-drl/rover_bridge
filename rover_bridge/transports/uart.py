# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""UART rover transport — the default, tethered binary link.

Mirrors ``RoverLink/main/uart_link.c``: TX serializes packets as
``[0xA5][type][len][payload][crc8]`` in a single write; RX runs the framing
FSM (:class:`rover_bridge.wire.FrameDecoder`) on a background reader thread.

Pi 5 wiring note: ``/dev/ttyAMA0`` (header pins 8/10) is shared with the
Bluetooth HCI by default — either ``dtoverlay=disable-bt`` and use ttyAMA0, or
``dtoverlay=uart0`` and use ttyAMA1. See RoverLink's README "Pi 5 UART gotcha".
"""

from __future__ import annotations

import threading
import time

from .. import wire
from ..logging_util import get_logger
from .base import RoverTransport, TelemetryCallbacks

log = get_logger("transport.uart")


class UartTransport(RoverTransport):
    def __init__(self, callbacks: TelemetryCallbacks,
                 port: str = "/dev/ttyAMA0", baud: int = 921600,
                 stats_period: float = 5.0):
        self._callbacks = callbacks
        self._port = port
        self._baud = baud
        self._stats_period = stats_period

        self._serial = None  # pyserial module, imported lazily in start()
        self._ser = None
        self._decoder = wire.FrameDecoder(self._on_packet)
        self._write_lock = threading.Lock()
        self._rx_thread: threading.Thread | None = None
        self._running = False

    def start(self) -> None:
        import serial  # imported lazily so an MQTT-only user needn't install pyserial
        self._serial = serial
        log.info("opening %s at %d baud ...", self._port, self._baud)
        self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="uart-rx", daemon=True)
        self._rx_thread.start()
        log.info("UART transport up (RX thread started)")

    def stop(self) -> None:
        self._running = False
        # Final zero so the bot stops immediately rather than after the
        # firmware heartbeat timeout (~500 ms).
        try:
            self.send_cmd_vel(0.0, 0.0)
        except Exception:
            pass
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.flush()
                time.sleep(0.05)
                self._ser.close()
            except Exception as e:
                log.warning("error closing serial port: %s", e)
        log.info("UART transport stopped (rx stats: %s)", self._decoder.stats)

    # --- TX -----------------------------------------------------------------

    def _write_frame(self, pkt_type: int, payload: bytes) -> None:
        if not self._ser:
            return
        data = wire.frame(pkt_type, payload)
        # Lock so concurrent senders never interleave byte-wise on the wire.
        with self._write_lock:
            self._ser.write(data)

    def send_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        self._write_frame(wire.PKT_CMD_VEL, wire.pack_cmd_vel(linear_x, angular_z))

    def send_cmd_pid(self, kp, ki, kd, output_clamp, deadband) -> None:
        self._write_frame(wire.PKT_CMD_PID,
                          wire.pack_cmd_pid(kp, ki, kd, output_clamp, deadband))

    def send_cmd_display(self, x, y, yaw, pitch, roll) -> None:
        self._write_frame(wire.PKT_CMD_DISPLAY,
                          wire.pack_cmd_display(x, y, yaw, pitch, roll))

    # --- RX -----------------------------------------------------------------

    def _on_packet(self, pkt_type: int, payload: bytes) -> None:
        try:
            self._callbacks.dispatch(pkt_type, payload)
        except Exception as e:
            log.error("error dispatching %s frame: %s",
                      wire.NAME_BY_TYPE.get(pkt_type, pkt_type), e)

    def _rx_loop(self) -> None:
        last_stats = time.monotonic()
        bytes_rx = 0
        while self._running:
            try:
                data = self._ser.read(256)
                if data:
                    bytes_rx += len(data)
                    self._decoder.feed(data)
                now = time.monotonic()
                if now - last_stats > self._stats_period:
                    s = self._decoder.stats
                    if bytes_rx == 0:
                        log.warning("UART RX: no bytes in %.0fs on %s — check wiring "
                                    "(TX/RX), baud (%d), and that the rover is sending "
                                    "telemetry", self._stats_period, self._port, self._baud)
                    elif s["ok"] == 0:
                        log.warning("UART RX: %d bytes but 0 valid frames (%s) — likely "
                                    "a baud mismatch or non-UGV data on the line",
                                    bytes_rx, s)
                    else:
                        log.info("UART RX: %d bytes, frames %s", bytes_rx, s)
                    bytes_rx = 0
                    last_stats = now
            except self._serial.SerialException as e:
                log.error("serial read error: %s", e)
                time.sleep(0.2)
            except Exception as e:
                log.error("unexpected RX error: %s", e)
                time.sleep(0.1)
