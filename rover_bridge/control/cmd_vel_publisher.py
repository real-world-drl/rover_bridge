# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Repeated cmd_vel publisher.

Ported from ``ros_ws/src/scripts/twist_publisher.py``. Instead of publishing a
ROS Twist, it calls a ``send(linear_x, angular_z)`` callback — wired to the
selected rover transport. The repeated publishing serves double duty: it
bridges gaps between inference messages *and* satisfies the firmware heartbeat
(no cmd_vel within ``UGV_HEARTBEAT_TIMEOUT_MS`` → the rover zeroes its motors),
so ``publish_rate`` must stay comfortably above 2 Hz. Default 10 Hz.

Lifecycle per received command:
  1. Republish the new cmd_vel up to ``max_publishes`` times at ``publish_rate``
     (the "buffer" that bridges gaps between inference messages).
  2. After the buffer expires, publish zero cmd_vel for ``max_zero_publishes``
     ticks so the rover stops promptly rather than coasting on the last command
     until its own heartbeat watchdog fires.
  3. Go silent once both phases are done; the heartbeat watchdog is the backstop.

A new ``publish()`` cancels the current cycle and restarts from step 1.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional

from ..logging_util import get_logger

log = get_logger("cmd_vel")


@dataclass
class CmdVel:
    linear_x: float = 0.0
    angular_z: float = 0.0


class RepeatedCmdVelPublisher:
    def __init__(self, send: Callable[[float, float], None],
                 publish_rate: float = 10.0,
                 max_publishes: int = 20, max_zero_publishes: int = 20):
        """
        Args:
            send: ``send(linear_x, angular_z)`` — the transport's cmd_vel sender.
            publish_rate: Hz at which cmd_vel is (re)published. Keep > 2 Hz so
                the firmware heartbeat never times out between ticks.
            max_publishes: Ticks to repeat the current command before zeroing.
                Buffer duration = ``max_publishes / publish_rate`` (default 2 s
                at 10 Hz, sized for ~1 Hz inference with margin).
            max_zero_publishes: Zero ticks published after the buffer expires
                before going silent (default 2 s of explicit stop at 10 Hz).
        """
        self._send = send
        self.publish_rate = publish_rate
        self.max_publishes = max_publishes
        self.max_zero_publishes = max_zero_publishes

        self.current: Optional[CmdVel] = None
        self.publish_count = 0
        self._timer: Optional[threading.Timer] = None
        self._recompute_callback: Optional[Callable[[], Optional[CmdVel]]] = None
        self.lock = threading.Lock()

    def publish(self, cmd: CmdVel,
                recompute_callback: Optional[Callable[[], Optional[CmdVel]]] = None) -> None:
        """Start (re)publishing ``cmd`` repeatedly, cancelling any prior cycle.

        Args:
            cmd: command for tick 0 and the fallback for later ticks.
            recompute_callback: optional zero-arg callable invoked each
                buffer-phase tick; if it returns a CmdVel that is sent and
                cached, if it returns None the cached command is resent. Not
                called during the zero phase.
        """
        self._stop_timer()
        with self.lock:
            self.current = cmd
            self.publish_count = 0
            self._recompute_callback = recompute_callback
        self._publish_once()

    def stop(self) -> None:
        """Halt publishing entirely (used at shutdown)."""
        self._stop_timer()
        with self.lock:
            self.current = None
            self.publish_count = 0
            self._recompute_callback = None

    def _publish_once(self) -> None:
        with self.lock:
            count = self.publish_count
            cmd = self.current
            callback = self._recompute_callback

        if cmd is None:
            return

        total_active = self.max_publishes + self.max_zero_publishes
        if count >= total_active:
            # Rover is definitely stopped — go silent until a new command arrives.
            log.debug("active-stop window finished, going silent")
            with self.lock:
                self.current = None
                self._timer = None
                self._recompute_callback = None
            return

        if count < self.max_publishes:
            # Buffer phase: ask the owner for a freshly-computed command if it
            # provided a callback; fall back to the cached command on None.
            if callback is not None:
                fresh = callback()
                if fresh is not None:
                    cmd = fresh
                    with self.lock:
                        self.current = fresh
            self._send(cmd.linear_x, cmd.angular_z)
        else:
            # Past the buffer — actively publish zero so the rover stops.
            self._send(0.0, 0.0)

        with self.lock:
            self.publish_count += 1
            interval = 1.0 / self.publish_rate
            self._timer = threading.Timer(interval, self._publish_once)
            self._timer.daemon = True
            self._timer.start()

    def _stop_timer(self) -> None:
        with self.lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
