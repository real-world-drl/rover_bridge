# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Command-line entry point + config resolution.

Precedence (highest first): CLI flag > YAML config file > built-in default —
the same model ``ros_ws`` used. A bare ``python -m rover_bridge`` auto-loads
the bundled ``config/bridge.yaml``; pass ``--config ''`` to skip it.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
from types import SimpleNamespace

import yaml

from .bridge import RoverBridge
from .logging_util import get_logger, setup_logging

log = get_logger("cli")

# Built-in defaults. Every key here is a valid YAML key and (with dashes) a CLI
# flag. Keep this the single source of truth for the config surface.
DEFAULTS = {
    # transports / connection
    "transport": "uart",            # uart | mqtt  (rover link; inference is always MQTT)
    "broker": "localhost",          # MQTT broker for inference (and rover link if mqtt)
    "port": 1883,
    "robot_id": "ugv01",            # matches CONFIG_UGV_ROBOT_ID; mqtt transport only
    "uart_port": "/dev/ttyAMA0",
    "uart_baud": 921600,

    # inference topics
    "action_topic": "gemnav/act",
    "ctrl_topic": "gemnav/ctrl",
    "remote_topic": "gemnav/remote",  # manual teleop {"linear":..,"angular":..}; moves even while halted
    "camera_topic": "rover/cam",    # bridge publishes frames here; model subscribes
    "pose_source": "wheel",         # wheel | vio  — what feeds the waypoint follower
    "pose_topic": "rover/pose",     # bridge publishes odometry pose here (PoseStamped JSON)
    "vio_pose_topic": "r2/slam/odom/tip/pose",  # pose_source=vio: subscribe to rover_vio's pose here
    "publish_pose": True,           # stream wheel-odometry pose to pose_topic (wheel source only)
    "pose_rate_limit": 10.0,        # pose publish cap (Hz); null = every odom sample
    "pose_frame_id": "odom",        # header.frame_id in the published pose
    "battery_topic": "rover/battery",  # bridge publishes battery charge % here ({"data": pct})
    "publish_battery": True,        # stream battery charge percentage to battery_topic
    "battery_cells": 3,             # series Li-ion cells (3S pack) for the SoC estimate

    # camera
    "camera": "oakd",               # oakd | realsense | picamera
    "no_camera": False,             # disable capture (e.g. when running the logger)
    "rate_limit": 1.0,              # camera publish cap (Hz); null = uncapped
    "width": 1280,
    "height": 720,
    "fps": 15,
    "crop_mode": "center",          # center | top | stretch
    "crop_top_fraction": 0.5,
    "rotate": 0,                    # 0/90/180/270 clockwise; 180 for an inverted mount

    # action handling / arc steering
    "action_scale": 1.0,
    "use_waypoints": True,
    "waypoint_index": 2,
    "actuation_duration": 2.0,
    "max_linear_velocity": 0.5,     # m/s  (rover-scale; firmware also clamps)
    "max_angular_velocity": 2.0,    # rad/s
    "turn_in_place_threshold_deg": 45.0,
    "min_angular_velocity": 0.0,

    # cmd_vel publishing cadence (also the firmware heartbeat — keep > 2 Hz)
    "publish_rate": 10.0,
    "max_publishes": 20,            # 20 @ 10 Hz = 2 s buffer
    "max_zero_publishes": 20,       # 20 @ 10 Hz = 2 s explicit stop

    # pose-driven advance (pose = host wheel odometry)
    "max_waypoint_advance": 3,
    "waypoint_tolerance": 0.3,
    "max_action_age": None,
    "recompute": True,

    # wheel odometry geometry (must match firmware Kconfig)
    "wheel_diameter_mm": 80.0,
    "track_width_mm": 172.0,
    "encoder_ppr": 1650,
    "publish_display": False,       # feed host odometry pose back to the OLED

    # misc
    "log_level": "INFO",
}

# (flag_dest, type, help) for keys whose CLI type isn't a plain str/auto.
_BOOL_KEYS = {"no_camera", "use_waypoints", "recompute", "publish_display",
              "publish_pose", "publish_battery"}
_FLOAT_KEYS = {"rate_limit", "crop_top_fraction", "action_scale", "actuation_duration",
               "max_linear_velocity", "max_angular_velocity",
               "turn_in_place_threshold_deg", "min_angular_velocity",
               "publish_rate", "waypoint_tolerance", "max_action_age",
               "wheel_diameter_mm", "track_width_mm", "pose_rate_limit"}
_INT_KEYS = {"port", "uart_baud", "width", "height", "fps", "waypoint_index",
             "max_publishes", "max_zero_publishes", "max_waypoint_advance",
             "encoder_ppr", "battery_cells", "rotate"}


def _default_config_path():
    """Locate the bundled ``config/bridge.yaml`` (CWD first, then repo root)."""
    candidates = [
        os.path.join(os.getcwd(), "config", "bridge.yaml"),
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "config", "bridge.yaml"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def _load_yaml(path):
    if not path:
        return {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except OSError as e:
        log.error("could not read config file %s: %s", path, e)
        sys.exit(1)
    except yaml.YAMLError as e:
        log.error("failed to parse YAML config %s: %s", path, e)
        sys.exit(1)
    if not isinstance(data, dict):
        log.error("config file %s must be a YAML mapping at the top level (got %s)",
                  path, type(data).__name__)
        sys.exit(1)
    unknown = set(data) - set(DEFAULTS)
    if unknown:
        log.warning("ignoring unknown config keys: %s", sorted(unknown))
    log.info("loaded %d key(s) from config file %s", len(data), path)
    return data


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rover_bridge",
        description="GemNav inference bridge for the RoverLink differential rover.")
    p.add_argument("--config", default=None,
                   help="YAML config path (default: bundled config/bridge.yaml; "
                        "pass '' to skip)")
    # Every DEFAULTS key gets a flag. CLI default is None so we can tell
    # "unset" from "explicitly set" for the precedence merge.
    for key in DEFAULTS:
        flag = "--" + key.replace("_", "-")
        if key in _BOOL_KEYS:
            # Provide both --flag / --no-flag so YAML/default can be overridden either way.
            p.add_argument(flag, dest=key, action="store_true", default=None,
                           help=f"enable {key} (default {DEFAULTS[key]})")
            p.add_argument("--no-" + key.replace("_", "-"), dest=key,
                           action="store_false", default=None,
                           help=f"disable {key}")
        elif key in _INT_KEYS:
            p.add_argument(flag, dest=key, type=int, default=None)
        elif key in _FLOAT_KEYS:
            p.add_argument(flag, dest=key, type=float, default=None)
        else:
            p.add_argument(flag, dest=key, default=None)
    return p


def resolve_config(argv=None) -> SimpleNamespace:
    parser = _build_parser()
    args = parser.parse_args(argv)

    config_path = args.config if args.config is not None else _default_config_path()
    yaml_cfg = _load_yaml(config_path)

    merged = dict(DEFAULTS)
    merged.update({k: v for k, v in yaml_cfg.items() if k in DEFAULTS})
    # CLI overrides anything that was explicitly provided (not None).
    for key in DEFAULTS:
        val = getattr(args, key, None)
        if val is not None:
            merged[key] = val

    cfg = SimpleNamespace(**merged)
    _validate(cfg)
    return cfg


def _validate(cfg: SimpleNamespace) -> None:
    if cfg.transport not in ("uart", "mqtt"):
        log.error("--transport must be 'uart' or 'mqtt', got %r", cfg.transport)
        sys.exit(1)
    if cfg.camera not in ("oakd", "realsense", "picamera"):
        log.error("--camera must be 'oakd', 'realsense' or 'picamera', got %r", cfg.camera)
        sys.exit(1)
    if cfg.pose_source not in ("wheel", "vio"):
        log.error("--pose-source must be 'wheel' or 'vio', got %r", cfg.pose_source)
        sys.exit(1)
    if cfg.publish_rate <= 2.0:
        log.warning("publish_rate=%.1f Hz is at/below the firmware heartbeat window "
                    "(~2 Hz) — the rover may stutter-stop. Use 5-10 Hz.",
                    cfg.publish_rate)


def main(argv=None) -> int:
    cfg = resolve_config(argv)
    setup_logging(cfg.log_level)

    log.info("config: transport=%s camera=%s broker=%s:%d id=%s",
             cfg.transport, cfg.camera, cfg.broker, cfg.port, cfg.robot_id)
    if cfg.transport == "uart":
        log.info("rover link: UART %s @ %d", cfg.uart_port, cfg.uart_baud)
    log.info("inference: action=%s ctrl=%s remote=%s camera_topic=%s",
             cfg.action_topic, cfg.ctrl_topic, cfg.remote_topic, cfg.camera_topic)
    if cfg.pose_source == "vio":
        log.info("pose source: VIO (subscribing %s); wheel-odom pose publish disabled",
                 cfg.vio_pose_topic)
    else:
        log.info("pose source: wheel odometry (publish=%s -> %s)",
                 cfg.publish_pose, cfg.pose_topic)
    if cfg.use_waypoints:
        log.info("arc steering: waypoint_index=%d advance=%d tolerance=%.2f m recompute=%s",
                 cfg.waypoint_index, cfg.max_waypoint_advance, cfg.waypoint_tolerance,
                 cfg.recompute)

    bridge = RoverBridge(cfg)

    stop_event = threading.Event()

    def _on_signal(_sig, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        bridge.start()
        log.info("press Ctrl-C to exit")
        stop_event.wait()
    finally:
        bridge.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
