# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""DepthAI OAK-D Lite color camera backend (the default).

Only the RGB color stream is used — the same input the GemNav model trains
on. Depth is available on this device but unused here.

Targets the DepthAI v3 API (``Camera.build`` + ``requestOutput`` +
``createOutputQueue``). v3 removed the legacy ``ColorCamera`` + ``XLinkOut``
pipeline; ``requestOutput((w, h), ...)`` ISP-scales to the exact requested
size, so ``--width/--height`` are honored.
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
        self._pipeline = None
        self._queue = None

    def open_device(self) -> None:
        import depthai as dai  # imported lazily so the dep is optional

        self._pipeline = dai.Pipeline()
        cam = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        # requestOutput((w, h), ...) ISP-scales to the exact requested size.
        # BGR888i comes out in OpenCV channel order; we flip to RGB for PIL in
        # read_rgb().
        output = cam.requestOutput((self.width, self.height),
                                   dai.ImgFrame.Type.BGR888i, fps=float(self.fps))
        self._queue = output.createOutputQueue(maxSize=1, blocking=False)
        self._pipeline.start()
        log.info("OAK-D color stream up (%dx%d @ %d FPS, depthai %s)",
                 self.width, self.height, self.fps, dai.__version__)

    def read_rgb(self) -> Optional[np.ndarray]:
        # Block until a frame is available so the loop doesn't busy-spin.
        in_frame = self._queue.get()
        if in_frame is None:
            return None
        # getCvFrame() yields BGR (OpenCV order); flip to RGB for PIL.
        bgr = in_frame.getCvFrame()
        return np.ascontiguousarray(bgr[:, :, ::-1])

    def close_device(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._queue = None
