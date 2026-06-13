# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Small logging helpers.

The bridge is ported from a ROS node that leaned on ``rospy.loginfo`` and the
``*_throttle`` variants. This module gives the equivalents on top of the
stdlib ``logging`` so the port reads the same without a ROS dependency.
"""

from __future__ import annotations

import logging
import time

_THROTTLE_LAST: dict[str, float] = {}


def setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)


def log_throttle(logger: logging.Logger, level: int, period: float, msg: str) -> None:
    """Log ``msg`` at most once per ``period`` seconds.

    Throttling is keyed by the message text (so distinct static messages each
    get their own window), mirroring how ``rospy.*_throttle`` is used here.
    """
    now = time.monotonic()
    last = _THROTTLE_LAST.get(msg)
    if last is None or (now - last) >= period:
        _THROTTLE_LAST[msg] = now
        logger.log(level, msg)
