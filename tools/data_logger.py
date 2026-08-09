#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Standalone recorder: camera frames + wheel-odometry pose into SQLite.

ROS-free port of ``ros_ws/src/scripts/data_logger.py``. Pose here comes from
the rover's own wheel odometry (received over the same UART/MQTT transport the
bridge uses) integrated on the host, rather than a ROS odometry topic.

Pass ``--vio-pose-topic`` to also record rover_vio's VIO pose as ground truth in
the same rows (``gt_x``/``gt_y``/``gt_yaw``), so wheel drift can be measured
against it offline. ``gt_age_s`` is how stale that VIO sample was when the row
was written — treat a large value as "no usable ground truth here" (it is NULL
when no VIO pose has arrived at all).

Run it *instead of* the bridge (or with the bridge's ``--no-camera`` set): the
camera can only be opened by one process. Each run writes::

    <base_dir>/<YYYYMMDD_HHMMSS>/
        images/<timestamp_ns>.jpg      # full-resolution RGB JPEG
        <YYYYMMDD_HHMMSS>.db           # SQLite, robot_telemetry table

Match images to rows via the shared ``timestamp`` (also in the filename). Rows
are committed every tick so the DB survives an unexpected power-off.

Usage:
    python tools/data_logger.py --base-dir /data/record/rover --frequency 2.0
    python tools/data_logger.py --base-dir /data/record/rover --frequency 2.0 \
        --transport mqtt --broker mqtt-h --robot-id ugv01 --camera oakd
    python tools/data_logger.py --base-dir /data/record/rover --frequency 2.0 \
        --broker darkhorse --vio-pose-topic gemnav/odometry_vio
"""

import argparse
import json
import os
import signal
import sqlite3
import sys
import threading
import time
from datetime import datetime

# Make the package importable when run as a script from the repo root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PIL import Image  # noqa: E402

from rover_bridge.cameras import make_camera  # noqa: E402
from rover_bridge.logging_util import get_logger, setup_logging  # noqa: E402
from rover_bridge.odometry import WheelOdometry, pose_from_stamped_dict  # noqa: E402
from rover_bridge.transports import TelemetryCallbacks, make_transport  # noqa: E402

log = get_logger("data_logger")


class VioGroundTruth:
    """Latest VIO pose from rover_vio, sampled per log row as ground truth.

    A dedicated MQTT client, separate from the rover transport's (which may not
    be MQTT at all) — same decoupling the bridge keeps between its inference and
    rover clients. Holds only the most recent pose: the capture loop samples it,
    it never queues.
    """

    def __init__(self, broker: str, port: int, topic: str, keepalive: int = 60):
        self.broker, self.port, self.topic, self.keepalive = broker, port, topic, keepalive
        self.client = None
        self._lock = threading.Lock()
        self._pose = None          # (x, y, yaw, received_ns)
        self._seen = False

    def start(self):
        import paho.mqtt.client as mqtt  # lazy: only needed with --vio-pose-topic

        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        log.info("connecting VIO ground-truth MQTT to %s:%d", self.broker, self.port)
        self.client.connect(self.broker, self.port, self.keepalive)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.topic)
            log.info("subscribed to VIO ground-truth topic: %s", self.topic)
        else:
            log.error("VIO ground-truth MQTT failed to connect (rc=%s)", rc)

    def _on_message(self, client, userdata, msg):
        try:
            x, y, yaw = pose_from_stamped_dict(json.loads(msg.payload))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            log.error("bad VIO pose payload on %s: %s", msg.topic, e)
            return
        if not self._seen:
            self._seen = True
            log.info("first VIO ground-truth pose received on %s", self.topic)
        with self._lock:
            self._pose = (x, y, yaw, time.time_ns())

    def sample(self, now_ns: int):
        """Return ``(x, y, yaw, age_s)`` or ``(None, None, None, None)``."""
        with self._lock:
            pose = self._pose
        if pose is None:
            return (None, None, None, None)
        x, y, yaw, ts = pose
        return (x, y, yaw, (now_ns - ts) / 1e9)

    def stop(self):
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            log.info("VIO ground-truth MQTT disconnected")


class DataLogger:
    def __init__(self, args):
        self.args = args
        self.session_dir = None
        self.images_dir = None
        self.conn = None
        self.cursor = None

        self.odometry = WheelOdometry(
            wheel_diameter_m=args.wheel_diameter_mm / 1000.0,
            track_width_m=args.track_width_mm / 1000.0,
            encoder_ppr=args.encoder_ppr,
        )
        callbacks = TelemetryCallbacks(on_wheel=self.odometry.update)
        if args.transport == "uart":
            tkw = dict(port=args.uart_port, baud=args.uart_baud)
        else:
            tkw = dict(broker=args.broker, port=args.port, robot_id=args.robot_id)
        self.transport = make_transport(args.transport, callbacks, **tkw)

        # Optional VIO ground truth alongside the (drifting) wheel odometry.
        self.vio = None
        if args.vio_pose_topic:
            self.vio = VioGroundTruth(args.broker, args.port, args.vio_pose_topic)

        # Reuse a camera backend for raw, full-resolution capture (publish is
        # unused here — we save frames to disk, not MQTT).
        self.camera = make_camera(args.camera, publish=lambda _b: None,
                                  width=args.width, height=args.height, fps=args.fps)

        self.running = False
        self.capture_thread = None

    def setup_session(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.args.base_dir, stamp)
        self.images_dir = os.path.join(self.session_dir, "images")
        os.makedirs(self.images_dir, exist_ok=True)

        db_path = os.path.join(self.session_dir, f"{stamp}.db")
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS robot_telemetry (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp   INTEGER,
                image_file  TEXT,
                x REAL, y REAL, yaw REAL,
                -- VIO ground truth (--vio-pose-topic); NULL when unused or not
                -- yet received. gt_age_s = staleness of the VIO sample at the
                -- moment this row was written.
                gt_x REAL, gt_y REAL, gt_yaw REAL, gt_age_s REAL
            );
            """)
        self.conn.commit()
        log.info("logging session: %s", self.session_dir)

    def start(self):
        self.transport.start()
        if self.vio:
            self.vio.start()
        self.camera.open_device()
        self.running = True
        self.capture_thread = threading.Thread(target=self._loop, daemon=True)
        self.capture_thread.start()
        log.info("capture loop started @ %.2f Hz", self.args.frequency)

    def _loop(self):
        period = 1.0 / self.args.frequency
        next_t = time.monotonic()
        while self.running:
            try:
                frame = self.camera.read_rgb()
                if frame is None:
                    continue
                x, y, yaw = self.odometry.pose
                timestamp_ns = time.time_ns()
                gt_x, gt_y, gt_yaw, gt_age = (
                    self.vio.sample(timestamp_ns) if self.vio else (None, None, None, None))
                rel = os.path.join("images", f"{timestamp_ns}.jpg")
                Image.fromarray(frame, "RGB").save(
                    os.path.join(self.session_dir, rel), format="JPEG", quality=85)
                self.cursor.execute(
                    "INSERT INTO robot_telemetry (timestamp, image_file, x, y, yaw, "
                    "gt_x, gt_y, gt_yaw, gt_age_s) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (timestamp_ns, rel, x, y, yaw, gt_x, gt_y, gt_yaw, gt_age))
                self.conn.commit()

                next_t += period
                sleep_for = next_t - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_t = time.monotonic()
            except Exception as e:
                log.error("error in capture loop: %s", e)
                time.sleep(0.1)

    def shutdown(self):
        log.info("shutting down data logger ...")
        self.running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        try:
            self.camera.close_device()
        except Exception as e:
            log.warning("error closing camera: %s", e)
        if self.vio:
            self.vio.stop()
        self.transport.stop()
        if self.conn:
            try:
                self.conn.commit()
                self.cursor.close()
                self.conn.close()
            except Exception as e:
                log.warning("error closing SQLite: %s", e)
        log.info("session saved to %s", self.session_dir)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--base-dir", required=True, help="base dir; a YYYYMMDD_HHMMSS/ subfolder is created per run")
    p.add_argument("--frequency", type=float, required=True, help="capture rate (Hz)")
    p.add_argument("--transport", default="uart", choices=["uart", "mqtt"])
    p.add_argument("--broker", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--robot-id", default="ugv01")
    p.add_argument("--uart-port", default="/dev/ttyAMA0")
    p.add_argument("--uart-baud", type=int, default=921600)
    p.add_argument("--vio-pose-topic", default=None,
                   help="record rover_vio's VIO pose as ground truth (gt_* columns), "
                        "e.g. gemnav/odometry_vio. Uses --broker/--port, and needs "
                        "rover_vio running. Off by default.")
    p.add_argument("--camera", default="picamera",
                   choices=["picamera", "oakd", "realsense"])
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--wheel-diameter-mm", type=float, default=80.0)
    p.add_argument("--track-width-mm", type=float, default=172.0)
    p.add_argument("--encoder-ppr", type=int, default=1650)
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args(argv)
    if args.frequency <= 0:
        p.error("--frequency must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    setup_logging(args.log_level)

    logger = DataLogger(args)
    logger.setup_session()

    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    try:
        logger.start()
        log.info("data logger running. Press Ctrl-C to exit.")
        stop_event.wait()
    finally:
        logger.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
