# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Camera backends. ``--camera {picamera,oakd,realsense}`` selects one; default picamera."""

from .base import CameraSource
from .oakd import OakDCamera
from .picamera import PiCamera3
from .realsense import RealSenseCamera


def make_camera(kind: str, *args, **kwargs) -> CameraSource:
    """Factory. ``kind`` is ``"picamera"`` (default), ``"oakd"`` or ``"realsense"``."""
    kind = kind.lower()
    if kind in ("oakd", "oak-d", "oak"):
        return OakDCamera(*args, **kwargs)
    if kind in ("realsense", "rs", "d435i", "d435"):
        return RealSenseCamera(*args, **kwargs)
    if kind in ("picamera", "picam", "pi", "module3", "imx708"):
        return PiCamera3(*args, **kwargs)
    raise ValueError(
        f"unknown camera {kind!r} (expected 'picamera', 'oakd' or 'realsense')")


__all__ = ["CameraSource", "OakDCamera", "RealSenseCamera", "PiCamera3", "make_camera"]
