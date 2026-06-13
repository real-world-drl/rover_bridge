# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""DepthAI OAK-D Lite color camera backend (the default).

Only the RGB color stream is used — the same input the OmniVLA model trains
on. Depth is available on this device but unused here; add a depth XLinkOut if
a future model wants it.

Capture note: the OAK-D Lite color sensor's native presets are 1080p/4K/12MP.
We run 1080p and ISP-downscale 2:3 to 1280×720, then the shared preprocess
crops/resizes to the model's square input. ``--width/--height`` are therefore
advisory for this backend (final frames are 224×224 regardless); the RealSense
backend honors them exactly.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..logging_util import get_logger
from .base import CameraSource

log = get_logger("camera.oakd")


class OakDCamera(CameraSource):
    name = "oakd"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._device = None
        self._queue = None

    def open_device(self) -> None:
        import depthai as dai  # imported lazily so the dep is optional

        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setIspScale(2, 3)  # 1920x1080 -> 1280x720
        cam.setInterleaved(False)
        cam.setFps(float(self.fps))

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.video.link(xout.input)

        self._device = dai.Device(pipeline)
        # maxSize=1 + non-blocking keeps us on the freshest frame; stale frames
        # are dropped rather than queued into seconds of lag.
        self._queue = self._device.getOutputQueue("rgb", maxSize=1, blocking=False)
        log.info("OAK-D color stream up (1080p ISP-scaled to 1280x720 @ %d FPS)", self.fps)

    def read_rgb(self) -> Optional[np.ndarray]:
        # Block until a frame is available so the loop doesn't busy-spin.
        in_frame = self._queue.get()
        if in_frame is None:
            return None
        # getCvFrame() yields BGR (OpenCV order); flip to RGB for PIL.
        bgr = in_frame.getCvFrame()
        return np.ascontiguousarray(bgr[:, :, ::-1])

    def close_device(self) -> None:
        if self._device is not None:
            self._device.close()
            self._device = None
            self._queue = None
