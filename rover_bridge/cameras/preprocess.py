# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Frame preprocessing shared by all camera backends.

Mirrors the training-time preprocessing (and ``ros_ws``'s
``realsense_camera_service._apply_crop``) so inference frames match what the
model was trained on regardless of which camera produced them: apply a crop
mode, then resize to a square target, then JPEG-encode.
"""

from __future__ import annotations

from io import BytesIO

from PIL import Image

VALID_CROP_MODES = ("center", "top", "stretch")


def apply_crop(img: Image.Image, crop_mode: str, crop_top_fraction: float) -> Image.Image:
    """Crop a PIL image per ``crop_mode``; caller resizes the result to a square.

    - ``center``: centred square from the smaller dimension.
    - ``top``: keep only the bottom ``crop_top_fraction`` of rows (drop
      sky/ceiling), then a centred square from that strip.
    - ``stretch``: no crop; caller resizes the full frame, accepting distortion.
    """
    w, h = img.size
    if crop_mode == "stretch":
        return img
    if crop_mode == "top":
        keep_h = max(1, int(round(h * crop_top_fraction)))
        img = img.crop((0, h - keep_h, w, h))
        w, h = img.size
    side = min(w, h)
    left = (w - side) // 2
    top = (h - side) // 2
    return img.crop((left, top, left + side, top + side))


def preprocess_to_jpeg(img: Image.Image, crop_mode: str, crop_top_fraction: float,
                       target_size: int = 224, quality: int = 85) -> bytes:
    """Crop + resize to ``target_size`` square + JPEG-encode. Returns raw bytes."""
    cropped = apply_crop(img, crop_mode, crop_top_fraction)
    resized = cropped.resize((target_size, target_size), Image.LANCZOS)
    buf = BytesIO()
    resized.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()
