# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Camera backends. ``--camera {oakd,realsense}`` selects one; default oakd."""

from .base import CameraSource
from .oakd import OakDCamera
from .realsense import RealSenseCamera


def make_camera(kind: str, *args, **kwargs) -> CameraSource:
    """Factory. ``kind`` is ``"oakd"`` (default) or ``"realsense"``."""
    kind = kind.lower()
    if kind in ("oakd", "oak-d", "oak"):
        return OakDCamera(*args, **kwargs)
    if kind in ("realsense", "rs", "d435i", "d435"):
        return RealSenseCamera(*args, **kwargs)
    raise ValueError(f"unknown camera {kind!r} (expected 'oakd' or 'realsense')")


__all__ = ["CameraSource", "OakDCamera", "RealSenseCamera", "make_camera"]
