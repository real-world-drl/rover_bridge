# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Intel RealSense D435i color camera backend (``--camera realsense``).

Ported from ``ros_ws/src/scripts/realsense_camera_service.py``. Only the RGB
color stream is used. Note that 1280×720 @ 30 FPS is not supported on most
D435 color streams — default to 15 FPS at that resolution.

RealSense exclusivity: the device can only be opened by one process. If you
also run the data logger, point only one of them at the camera at a time.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..logging_util import get_logger
from .base import CameraSource

log = get_logger("camera.realsense")


class RealSenseCamera(CameraSource):
    name = "realsense"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pipeline = None

    def open_device(self) -> None:
        import pyrealsense2 as rs  # imported lazily so the dep is optional

        self._rs = rs
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height,
                             rs.format.rgb8, self.fps)
        self._pipeline.start(config)
        log.info("RealSense color stream up (%dx%d @ %d FPS)",
                 self.width, self.height, self.fps)

    def read_rgb(self) -> Optional[np.ndarray]:
        frames = self._pipeline.wait_for_frames(timeout_ms=5000)
        color = frames.get_color_frame()
        if not color:
            return None
        # Stream is already rgb8, so no channel swap needed.
        return np.asanyarray(color.get_data())

    def close_device(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
