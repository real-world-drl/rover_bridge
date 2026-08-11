# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Raspberry Pi camera backend via ``rpicam-vid`` (``--camera picamera``).

Captures by spawning ``rpicam-vid`` (the libcamera-based CLI from
``rpicam-apps``) and reading an MJPEG stream off its stdout. This deliberately
avoids ``picamera2`` and the ``libcamera`` Python bindings — on Ubuntu (incl.
Ubuntu 24.04 on a Pi 5) those aren't packaged and the bindings are painful to
install, whereas ``rpicam-apps`` builds cleanly and is the part that actually
works there. The only runtime requirement is the ``rpicam-vid`` binary on
``PATH`` (set ``ROVER_RPICAM_BIN`` to override; legacy ``libcamera-vid`` is
auto-detected as a fallback). Works the same on Raspberry Pi OS.

Only an RGB stream is used — the same input the GemNav model trains on. The
Module 3 is autofocus-capable; focus is locked to infinity at startup
(``--autofocus-mode manual --lens-position 0.0``) so the lens never hunts while
driving. MJPEG frames are decoded with PIL; the base loop re-encodes the
preprocessed 224×224 frame, so the intermediate decode is cheap at our rates.
"""

from __future__ import annotations

import os
import select
import shlex
import shutil
import subprocess
import threading
from io import BytesIO
from typing import Optional

import numpy as np
from PIL import Image

from ..logging_util import get_logger
from .base import CameraSource

log = get_logger("camera.picamera")

# JPEG start-of-image / end-of-image markers used to frame the MJPEG stream.
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"


class PiCamera3(CameraSource):
    name = "picamera"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._proc: Optional[subprocess.Popen] = None
        self._buf = b""
        self._exit_logged = False
        # Populated by open_device; declared here so close_device is safe to
        # call on a device that never opened.
        self._reader: Optional[threading.Thread] = None
        self._reader_stop: Optional[threading.Event] = None
        self._latest: Optional[bytes] = None
        self._latest_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self.frames_captured = 0
        self.frames_dropped = 0

    def _rpicam_binary(self) -> str:
        override = os.environ.get("ROVER_RPICAM_BIN")
        if override:
            return override
        for name in ("rpicam-vid", "libcamera-vid"):
            path = shutil.which(name)
            if path:
                return path
        raise FileNotFoundError(
            "rpicam-vid not found on PATH (install rpicam-apps, or set "
            "ROVER_RPICAM_BIN). On Ubuntu this is built from source; on "
            "Raspberry Pi OS it ships via apt.")

    def open_device(self) -> None:
        binary = self._rpicam_binary()
        cmd = [
            binary,
            "-t", "0",                       # run until terminated
            "-n",                            # no preview window
            "--codec", "mjpeg",              # one JPEG per frame on stdout
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", str(self.fps),
            "--autofocus-mode", "manual",    # lock focus (Module 3); ignored on
            "--lens-position", "0.0",        # fixed-focus modules
        ]
        # NOTE: rpicam-vid's --rotation/--hflip/--vflip transforms are silently
        # ignored on the Pi 5 (PiSP) pipeline, so frame rotation for an inverted
        # mount is done in Python instead — set `rotate: 180` (config/--rotate).
        # Force a full-FOV sensor mode. On the Module 3 (IMX708) the mode rpicam
        # auto-picks for 720p is a centre-cropped one with reduced FOV; the
        # 2304x1296 binned mode is full-frame. Set ROVER_RPICAM_MODE="" (or
        # "auto") to let rpicam choose, or to another WxH (e.g. "4608:2592").
        mode = os.environ.get("ROVER_RPICAM_MODE", "2304:1296")
        if mode and mode.lower() != "auto":
            cmd += ["--mode", mode]
        # Optional passthrough for mount-specific flags, e.g.
        # ROVER_RPICAM_EXTRA_ARGS="--hflip --vflip" for an inverted camera.
        extra = os.environ.get("ROVER_RPICAM_EXTRA_ARGS")
        if extra:
            cmd += shlex.split(extra)
        cmd += ["-o", "-"]                   # stream to stdout
        self._buf = b""
        self._exit_logged = False
        # rpicam-vid's own diagnostics (buffer allocation, dropped frames, mode
        # selection) go to stderr. Inherit it rather than DEVNULL — when capture
        # stalls, that output is the difference between reading the cause and
        # inferring it from arrival gaps. ROVER_RPICAM_QUIET=1 to silence.
        stderr = subprocess.DEVNULL if os.environ.get("ROVER_RPICAM_QUIET") else None
        # bufsize=0: unbuffered, so we read frames with os.read on the raw fd.
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=stderr, bufsize=0)

        # Drain the pipe continuously on a reader thread, keeping only the newest
        # complete frame. rpicam-vid writes into a 64 KB kernel pipe — smaller
        # than a single 720p MJPEG frame — so if the only reader is a capture
        # loop polling at the publish rate, rpicam spends most of its life
        # blocked in write() and capture stalls for seconds at a time. Lowering
        # --framerate does not help: the problem is the back-pressure, not the
        # data rate. (The OAK-D backend never shows this because depthai does its
        # own draining internally.)
        self._latest = None
        self._latest_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._reader_stop = threading.Event()
        self.frames_captured = 0
        self.frames_dropped = 0
        self._reader = threading.Thread(target=self._reader_loop,
                                        name="rpicam-rx", daemon=True)
        self._reader.start()
        log.info("rpicam-vid stream up (%s, %dx%d @ %d FPS, MJPEG)",
                 os.path.basename(binary), self.width, self.height, self.fps)

    def _reader_loop(self) -> None:
        """Drain rpicam-vid's stdout continuously, keeping only the newest frame.

        Runs for the life of the device. Its only job is to make sure nobody is
        ever waiting to write into the pipe — the capture loop's rate is then a
        publishing decision, not a back-pressure knob on the sensor.
        """
        fd = self._proc.stdout.fileno()
        while not self._reader_stop.is_set():
            try:
                ready, _, _ = select.select([fd], [], [], 0.5)
                if not ready:
                    continue
                chunk = os.read(fd, 1 << 16)
                if not chunk:
                    log.warning("rpicam-vid stdout closed; no more frames")
                    return
                self._buf += chunk
                jpeg = self._take_last_jpeg()
                if jpeg is None:
                    continue
                with self._latest_lock:
                    if self._latest is not None:
                        # Captured faster than the publish rate: expected, and
                        # the whole point — the consumer always gets the newest.
                        self.frames_dropped += 1
                    self._latest = jpeg
                    self.frames_captured += 1
                self._frame_ready.set()
            except OSError as e:
                if not self._reader_stop.is_set():
                    log.error("rpicam-vid read error: %s", e)
                return

    def read_rgb(self) -> Optional[np.ndarray]:
        """Return the newest complete frame decoded to RGB, or None if none has
        arrived. Never touches the pipe — the reader thread owns it."""
        if self._proc is None:
            return None
        if self._proc.poll() is not None:
            if not self._exit_logged:
                log.warning("rpicam-vid exited (code %s); no frames",
                            self._proc.returncode)
                self._exit_logged = True
            return None

        # Wait briefly so an uncapped capture loop doesn't spin on None.
        if not self._frame_ready.wait(timeout=2.0):
            return None
        with self._latest_lock:
            jpeg, self._latest = self._latest, None
            self._frame_ready.clear()
        return self._decode(jpeg) if jpeg is not None else None

    def _take_last_jpeg(self) -> Optional[bytes]:
        """Pop the last complete JPEG from the buffer, discarding older ones."""
        end = self._buf.rfind(_EOI)
        if end == -1:
            return None
        end += len(_EOI)
        start = self._buf.rfind(_SOI, 0, end)
        if start == -1:
            self._buf = self._buf[end:]   # drop leading garbage before the EOI
            return None
        jpeg = self._buf[start:end]
        self._buf = self._buf[end:]       # keep any partial next frame
        return jpeg

    @staticmethod
    def _decode(jpeg: bytes) -> Optional[np.ndarray]:
        try:
            img = Image.open(BytesIO(jpeg)).convert("RGB")
            return np.asarray(img)
        except Exception as e:
            log.warning("failed to decode MJPEG frame: %s", e)
            return None

    def close_device(self) -> None:
        if self._reader_stop is not None:
            self._reader_stop.set()
        if self._reader is not None and self._reader.is_alive():
            self._reader.join(timeout=2.0)
        if self._proc is not None:
            log.info("rpicam-vid captured %d frame(s), %d superseded before "
                     "publish", self.frames_captured, self.frames_dropped)
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        self._buf = b""
