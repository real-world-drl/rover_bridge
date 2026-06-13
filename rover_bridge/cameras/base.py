# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Camera source base class.

Defines the capture/preprocess/publish loop once; backends only implement
device bring-up (:meth:`_open`), grabbing one RGB frame (:meth:`_read_rgb`),
and teardown (:meth:`_close`). The loop handles rate limiting, cropping +
resize to the model's input size, JPEG encoding, and handing bytes to the
``publish`` callback (which the bridge wires to the inference MQTT client).
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, Optional

import numpy as np
from PIL import Image

from ..logging_util import get_logger
from .preprocess import VALID_CROP_MODES, preprocess_to_jpeg

log = get_logger("camera")


class CameraSource(ABC):
    name = "camera"

    def __init__(self, publish: Callable[[bytes], None],
                 rate_limit: Optional[float] = None,
                 width: int = 1280, height: int = 720, fps: int = 15,
                 crop_mode: str = "center", crop_top_fraction: float = 0.5,
                 target_size: int = 224):
        """
        Args:
            publish: ``publish(jpeg_bytes)`` — sends a preprocessed frame onward.
            rate_limit: Max publish frequency in Hz. None = as fast as frames arrive.
            width/height/fps: Requested capture stream config.
            crop_mode: ``center`` | ``top`` | ``stretch``.
            crop_top_fraction: Bottom fraction kept when ``crop_mode == "top"``.
            target_size: Square model input size (default 224).
        """
        self._publish = publish
        self.rate_limit = rate_limit
        self.width = width
        self.height = height
        self.fps = fps

        if crop_mode not in VALID_CROP_MODES:
            log.warning("invalid crop_mode %r; falling back to 'center' (valid: %s)",
                        crop_mode, VALID_CROP_MODES)
            crop_mode = "center"
        self.crop_mode = crop_mode

        if not (0.0 < crop_top_fraction <= 1.0):
            clamped = max(min(crop_top_fraction, 1.0), 1e-3)
            log.warning("crop_top_fraction=%s out of (0, 1]; clamping to %s",
                        crop_top_fraction, clamped)
            crop_top_fraction = clamped
        self.crop_top_fraction = crop_top_fraction

        self.target_size = target_size
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_publish: Optional[float] = None

    # --- backend hooks (public so tools like the data logger can reuse a
    # backend for raw, full-resolution capture without the threaded loop) ---

    @abstractmethod
    def open_device(self) -> None:
        """Bring up the device pipeline. Raise on failure."""

    @abstractmethod
    def read_rgb(self) -> Optional[np.ndarray]:
        """Return one HxWx3 uint8 RGB frame, or None if none available."""

    @abstractmethod
    def close_device(self) -> None:
        """Tear down the device pipeline."""

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> bool:
        try:
            self.open_device()
        except Exception as e:
            log.error("failed to initialize %s camera: %s", self.name, e)
            return False
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"{self.name}-cap",
                                        daemon=True)
        self._thread.start()
        log.info("%s camera started (%dx%d@%d, crop=%s, rate_limit=%s)",
                 self.name, self.width, self.height, self.fps,
                 self.crop_mode, self.rate_limit)
        return True

    def stop(self) -> None:
        log.info("stopping %s camera ...", self.name)
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self.close_device()
        except Exception as e:
            log.warning("error closing %s camera: %s", self.name, e)

    def _loop(self) -> None:
        while self._running:
            try:
                if self.rate_limit is not None:
                    now = time.time()
                    if self._last_publish is not None:
                        min_interval = 1.0 / self.rate_limit
                        wait = min_interval - (now - self._last_publish)
                        if wait > 0:
                            time.sleep(wait)
                            continue
                    self._last_publish = now

                frame = self.read_rgb()
                if frame is None:
                    continue

                img = Image.fromarray(frame, "RGB")
                jpeg = preprocess_to_jpeg(img, self.crop_mode, self.crop_top_fraction,
                                          target_size=self.target_size)
                self._publish(jpeg)
            except Exception as e:
                log.error("error in %s capture loop: %s", self.name, e)
                time.sleep(0.1)
