# rover_bridge

GemNav inference bridge for the **RoverLink** differential-drive rover.

This is the sibling of `../../ros_ws` (the ROS↔MQTT bridge for Spot), rebuilt
for a small differential rover that runs the [RoverLink](../RoverLink)
ESP32 firmware instead of ROS. The inference loop is identical — same model,
same MQTT topics — only the robot link changes: instead of publishing a ROS
`Twist`, the bridge sends RoverLink's native packed `cmd_vel` over **UART** or
**MQTT**, and reads telemetry back the same way.

## What it does

```
 camera ─crop/resize 224─▶ MQTT(camera_topic) ─▶  GemNav model
   model ─MQTT(gemnav/act)─▶ bridge ─arc steering─▶ repeated cmd_vel
        ─UART/MQTT─▶ rover         (also the firmware heartbeat)
   rover ─tel/wheel─▶ bridge ─wheel odometry─▶ pose ─▶ waypoint advance
                                                    └─MQTT(pose_topic)▶ consumers
```

1. **Camera → model.** Captures from an OAK-D Lite (default) or RealSense
   D435i, center/top/stretch-crops + resizes to 224×224 JPEG, and publishes to
   `camera_topic` for the inference client to consume.
2. **Action → motion.** Subscribes to `gemnav/act`; converts the inference
   waypoint trajectory to `(linear, angular)` via pure-pursuit arc steering and
   republishes it as `cmd_vel` at a fixed rate. The repeated publish doubles as
   RoverLink's heartbeat (no `cmd_vel` within ~500 ms → the rover stops).
3. **Stop/start.** `gemnav/ctrl` accepts `{"stop": true}` (immediate, sticky
   halt) and `{"start": true}` (resume).
4. **Remote teleop.** `gemnav/remote` accepts `{"linear": .., "angular": ..}`
   and drives the rover manually — *even while halted* (see below).
5. **Pose-driven advance.** The rover's encoders feed `tel/wheel`; the bridge
   integrates the cumulative ticks into `(x, y, yaw)` and advances through the
   trajectory's waypoints as the rover reaches each one — smoothing over
   inference latency.

The **inference side is always MQTT**. Only the **rover link** (cmd_vel out,
telemetry back) is selectable via `--transport`.

## Install

### uv (quickest)

[`uv`](https://docs.astral.sh/uv/) creates the venv and installs deps on the
first `uv run`, so there's no separate install step:

```bash
uv run --extra oakd rover-bridge --pose-source vio         # OAK-D Lite (default camera)
uv run --extra realsense rover-bridge     # D435i
uv run rover-bridge --no-camera           # core only (no camera SDK)
```

`rover-bridge` is the console entry point; `uv run python -m rover_bridge` is
equivalent. Pre-install (and write a lockfile) with `uv sync --extra oakd` if
you'd rather not install on first run. Camera SDKs are optional extras (lazy
imports), so the core install runs without them — see the Pi 5 ARM notes below
if `pyrealsense2` has no aarch64 wheel.

### Conda (Raspberry Pi 5 / the deployment target)

The core deps are all on conda-forge for `linux-aarch64`; the camera SDKs are
not reliably packaged for conda on ARM, so install whichever one matches your
hardware via pip afterward.

```bash
conda env create -f environment.yml
conda activate rover_bridge
pip install -e '.[oakd]'          # OR  pip install -e '.[realsense]'
```

Update the env after editing `environment.yml`:

```bash
conda env update -f environment.yml --prune
```

### Plain pip

```bash
pip install -e .                  # core (paho-mqtt, pyserial, pillow, numpy, pyyaml)
pip install -e '.[oakd]'          # + DepthAI for the OAK-D Lite (default camera)
pip install -e '.[realsense]'     # + pyrealsense2 for the D435i
```

Camera SDKs are imported lazily, so you only need the one matching your
hardware. `pyserial` is likewise only needed for the UART transport.

> **Pi 5 notes.** `depthai` has ARM64 wheels and pip-installs cleanly;
> `pyrealsense2` often has no prebuilt aarch64 wheel and may need librealsense
> built from source. For the UART transport, add your user to the `dialout`
> group (`sudo usermod -aG dialout $USER`, then re-login) so it can open the
> serial port without root.

## Run

With `uv` (no activation needed; prefix any of the below with
`uv run --extra oakd`):

```bash
# Defaults: UART link on /dev/ttyAMA0, OAK-D camera, broker localhost.
# Auto-loads config/bridge.yaml. VIO pose source instead of wheel odometry:
uv run --extra oakd rover-bridge --pose-source vio
```

Or with an activated env (conda / `pip install -e`), use `python -m rover_bridge`:

```bash
# Defaults: UART link on /dev/ttyAMA0, OAK-D camera, broker localhost.
# Auto-loads config/bridge.yaml.
python -m rover_bridge

# MQTT rover link to a remote broker, RealSense camera:
python -m rover_bridge --transport mqtt --broker mqtt-h --robot-id ugv01 --camera realsense

# Skip the YAML entirely (built-in defaults + CLI only):
python -m rover_bridge --config ''
```

Any CLI flag overrides the YAML, which overrides the built-in default
(precedence: **CLI > YAML > default**). Every key in `config/bridge.yaml` has a
matching `--kebab-case` flag.

## Rover link: UART vs MQTT

| | `--transport uart` (default) | `--transport mqtt` |
|---|---|---|
| cmd_vel | `0xA5`-framed `ugv_cmd_vel_t` on serial | binary on `ugv/<id>/v1/cmd/vel`, QoS 1 |
| telemetry | framed packets decoded by an RX thread | subscribed on `ugv/<id>/v1/tel/*` |
| needs | `pyserial`, a serial port | a broker reachable by both ends |

The wire contract is mirrored verbatim from `../RoverLink/main/ugv_packets.h`
in [`rover_bridge/wire.py`](rover_bridge/wire.py) — same packed structs, same
CRC8, same UART framing FSM. Import-time size asserts fail loudly if it drifts
from the firmware.

**Pi 5 UART gotcha:** `/dev/ttyAMA0` (header pins 8/10) is shared with the
Bluetooth HCI by default. Either `dtoverlay=disable-bt` in
`/boot/firmware/config.txt` (then use `ttyAMA0`), or `dtoverlay=uart0` (then use
`ttyAMA1`). This is the #1 reason "the Pi can't see the bot."

## Cameras

`--camera oakd` (default) or `--camera realsense`. Both use only the RGB
stream — the model's input. The shared preprocessing (`crop_mode` ∈
`center|top|stretch`, then resize to 224×224 JPEG) matches training-time
preprocessing, so frames are interchangeable across backends.

- **OAK-D Lite** captures via the DepthAI v3 API (`Camera.requestOutput`), which
  ISP-scales to the exact `--width/--height`. The final inference frame is
  224×224 either way.
- Capture is rate-limited by `--rate-limit` (Hz), sized to your inference rate.

## Arc steering & differential drive

`ArcSteering` produces `(linear, angular)` directly — exactly what a
differential rover's `cmd_vel` wants. Differences from the Spot tuning:

- `turn_in_place_threshold_deg` defaults to **45°**: for targets sharper than
  that the rover pivots in place (which it does cleanly) instead of tracing a
  backwards-looping arc.
- Velocity caps default to rover scale (`max_linear_velocity` 0.5 m/s,
  `max_angular_velocity` 2.0 rad/s). These are host-side shaping caps; the
  firmware also clamps via `UGV_MAX_LINEAR/ANGULAR`.

## Stop latency & heartbeat

`publish_rate` (default 10 Hz) sets how often `cmd_vel` is republished. Keep it
well above ~2 Hz or the rover stutter-stops on its heartbeat timeout. Per
command the publisher runs two phases:

1. **Buffer** — repeats the command `max_publishes` ticks (default 20 @ 10 Hz =
   2 s), bridging gaps between inference messages.
2. **Active stop** — publishes zero `cmd_vel` for `max_zero_publishes` ticks so
   the rover stops promptly when inference goes silent.

A new action cancels both and restarts. `{"stop": true}` on the ctrl topic
halts immediately and stays halted until `{"start": true}`.

## Remote teleop

`remote_topic` (default `gemnav/remote`) lets an operator drive the rover
manually, independent of inference. It accepts `{"linear": 0.3, "angular":
0.14}` — linear (m/s) and angular (rad/s) velocities, converted to `cmd_vel`
and pushed through the same repeated publisher (so the heartbeat and buffer/zero
phases still apply). Missing `linear`/`angular` default to 0.

Remote velocities are multiplied by `action_scale` (default `1.0`, i.e. 1:1).
`action_scale` is **shared** with inference actions on `gemnav/act`; there is
no separate remote-only scale.

Unlike actions on `gemnav/act`, remote commands **move the rover even while the
bridge is halted** (i.e. after `{"stop": true}`, waiting for `{"start": true}`).
They bypass the halt state but do **not** change it: once the command's
republish buffer expires (see above) the rover stops and inference stays halted.
Each remote command also clears any in-flight waypoint trajectory so pose-driven
advance can't override it. Send a steady stream of remote messages to keep
driving.

## Pose source: wheel odometry vs VIO

`pose_source` selects what feeds the waypoint follower:

- **`wheel`** (default) — integrate the rover's encoders (below); the bridge
  publishes that pose to `pose_topic`.
- **`vio`** — consume [`rover_vio`](../rover_vio)'s visual-inertial pose off MQTT
  (`vio_pose_topic`, default `r2/slam/odom/tip/pose`) and feed it to the follower
  instead. `rover_vio` owns that topic, so the bridge does **not** publish wheel
  pose in this mode. Both frames are REP-103 (x-forward, y-left), so the follower
  gets compatible poses either way. Wheel odometry still integrates (harmlessly)
  but doesn't drive the follower.

VIO is the more accurate source (wheel odometry drifts with slip); wheel is the
zero-dependency fallback. There's no automatic failover — if VIO stops
publishing, the follower simply stops getting fresh pose.

[`rover_vio`](../rover_vio) is the sibling project that produces the VIO pose —
standalone OpenVINS on a RealSense D435i (no ROS), publishing the same
`PoseStamped` JSON contract on the same broker. Build and run it on the rover,
then start the bridge with `--pose-source vio`:

```bash
cd ../rover_vio && ./build/rover_vio    # stereo by default; publishes pose to r2/slam/odom/tip/pose
```

## Wheel odometry

The rover's encoders work and `tel/wheel` carries cumulative signed ticks. The
host integrates them with standard diff-drive geometry into `(x, y, yaw)`,
feeding the waypoint follower. The geometry **must match the firmware Kconfig**:

| Config | Default | Firmware Kconfig |
|---|---|---|
| `wheel_diameter_mm` | 80 | `UGV_WHEEL_DIAMETER_MM` |
| `track_width_mm` | 172 | `UGV_TRACK_WIDTH_MM` |
| `encoder_ppr` | 1650 | `UGV_ENCODER_PPR` |

Set `publish_display: true` to feed this host pose back to the rover's OLED
(`cmd/display`).

### Pose streaming

The odometry pose is also streamed to MQTT for the inference side / external
consumers, in the **same format ros_ws used** — a `geometry_msgs/PoseStamped`
serialized to JSON:

```json
{"header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": "odom"},
 "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
          "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
```

`z` is always 0 and the quaternion is yaw-only (the rover is planar). Yaw is
the standard CCW math convention; consumers apply their own heading convention,
exactly as the data logger's `quat_to_rpy` does. Configure with:

- `publish_pose` (default `true`) — enable/disable.
- `pose_topic` (default `rover/pose`) — must match what the consumer subscribes to.
- `pose_rate_limit` (default `10.0` Hz; `null` = every ~50 Hz odometry sample).
- `pose_frame_id` (default `odom`).

### Battery

The rover reports raw pack voltage + current (`tel/battery`); the bridge
converts pack voltage to a charge percentage for the 3S Li-ion pack and streams
it, with `data` matching ros_ws's `Float32` `charge_percentage` shape:

```json
{"data": 78.0, "voltage_v": 12.05, "current_a": 1.2, "cells": 3}
```

- `publish_battery` (default `true`), `battery_topic` (default `rover/battery`).
- `battery_cells` (default `3`) — series Li-ion cells.

The percentage comes from a per-cell **open-circuit-voltage** lookup
([`battery.py`](rover_bridge/battery.py)), so it's a rough gauge, not a fuel
gauge: Li-ion OCV is flat through the mid-range, and under motor load the pack
voltage sags, so the estimate reads low while driving. Published at the
firmware's battery telemetry rate (~1 Hz). Retune the OCV table in `battery.py`
if your cells differ.

## Data logger

A standalone recorder ([`tools/data_logger.py`](tools/data_logger.py)) saves
full-resolution camera frames + wheel-odometry pose into a timestamped SQLite
session. It opens its own transport + camera, so run it *instead of* the bridge
(or start the bridge with `--no-camera`) — the camera can only be opened by one
process.

```bash
python tools/data_logger.py --base-dir /data/record/rover --frequency 2.0
python tools/data_logger.py --base-dir /data/record/rover --frequency 2.0 \
    --transport mqtt --broker mqtt-h --robot-id ugv01 --camera realsense
```

Each session writes `images/<timestamp_ns>.jpg` and a `robot_telemetry` table
(`timestamp, image_file, x, y, yaw`); rows are committed every tick.

## Layout

```
rover_bridge/
  wire.py            # RoverLink wire contract: pack/unpack, CRC8, UART framing FSM
  odometry.py        # diff-drive wheel odometry (ticks -> x,y,yaw)
  inference.py       # always-MQTT side: camera publish + act/ctrl dispatch
  bridge.py          # orchestrator that wires it all together
  cli.py             # argparse + YAML config (CLI > YAML > default)
  transports/        # rover link: base ABC, uart, mqtt
  control/           # arc_steering, cmd_vel_publisher, waypoint_follower
  cameras/           # base + oakd + realsense backends, shared preprocess
config/bridge.yaml   # checked-in defaults
tools/data_logger.py # standalone recorder
```
