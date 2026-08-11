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
     VIO ─MQTT(vio_pose_topic)─▶ bridge ─ground truth─▶ MQTT(gt_pose_topic)
                                                      └─▶ SQLite pose log
```

1. **Camera → model.** Captures from a Pi Camera Module 3 (default), OAK-D Lite
   or RealSense D435i, center/top/stretch-crops + resizes to 224×224 JPEG, and
   publishes to `camera_topic` for the inference client to consume.
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
6. **Ground truth.** Whichever pose source isn't steering (VIO by default) is
   republished to `gt_pose_topic` and optionally logged to SQLite alongside the
   active one, so encoder drift can be measured against a reference.

The **inference side is always MQTT**. Only the **rover link** (cmd_vel out,
telemetry back) is selectable via `--transport`.

### MQTT contract

The `gemnav/*` names are shared across robots — the same off-board inference
client drives Spot (via [`ros_ws`](../../ros_ws)) or this rover unchanged. Every
topic below is a config key, so any of them can be renamed.

| Topic | dir | Payload |
|---|---|---|
| `gemnav/camera` | out | 224×224 JPEG (`camera_topic`) |
| `gemnav/odometry` | out | active pose, PoseStamped JSON (`pose_topic`) |
| `gemnav/odometry_gt` | out | ground-truth pose, same format (`gt_pose_topic`) |
| `gemnav/battery` | out | `{"data": pct, ...}` (`battery_topic`) |
| `gemnav/act` | in | inference waypoint trajectory |
| `gemnav/ctrl` | in | `{"stop": true}` / `{"start": true}` |
| `gemnav/remote` | in | manual teleop; moves even while halted |
| `gemnav/odometry_vio` | in | VIO pose from `rover_vio_iphone` (or `rover_vio`) (`vio_pose_topic`) |
| `gemnav/goal` | in | navigation goal; observed only (`goal_topic`) |

On Spot `gemnav/odometry` likewise carries plain odometry — from the `slam`
package's odom output, not a SLAM-corrected pose.

### Observations and the goal

**Nothing here assembles `gemnav/obs`** — the inference server builds
observations itself. `vla_gemma.stream` defaults to *direct-obs* mode, where it
subscribes to the raw topics (`stream.py`: `--image-topic gemnav/camera`,
`--pose-topic gemnav/odometry`, `--goal-topic gemnav/goal`) and skips the
base64 repackaging hop the retired `spot_client` used to do. Those defaults are
this bridge's contract, so no flags are needed on either side:

```bash
# inference host — direct-obs is the default
python -m vla_gemma.stream --backend onnx --config configs/default.yaml --host darkhorse

# set the goal once; it is published RETAINED, so the server picks it up
# whenever it connects
python -m vla_gemma.goal_client --host darkhorse -gx 13.5 -gy 1.0
```

The formats line up as: camera = **raw binary JPEG** (which is also the server's
inference trigger), odometry = PoseStamped JSON, goal = retained JSON. A server
run with `--image-topic none` falls back to packaged obs from an external
producer — this bridge doesn't provide one.

**Goal frame.** `goal_world_x/y` is expressed in the same frame as the pose on
`pose_topic`, and the server re-projects it to body frame every tick. GemNav's
docs call that frame "global ROS-NWU, x=North" — that's the REP-103 axis
convention (x forward, y left), *not* a compass heading. The origin is wherever
odometry started, on Spot as much as here: restart the odometry and `(0,0,0)`
moves with it. So goals are start-pose-relative metres, and need re-issuing
after a restart. Switching `pose_source` also moves the origin, since VIO
initialises its own.

**The bridge only observes the goal** — it never acts on it; the server does the
projection. It records each goal onto the run in the pose log (`goal_log` table
plus `run.goal`), so a recorded experiment says what it was driving to. A base64
`goal_image` is fingerprinted (`goal_image_sha256` + `goal_image_b64_len`)
rather than stored whole, which keeps a 260 KB inline image at ~160 bytes on
disk while still identifying it. Set `goal_topic: ''` to not subscribe at all.

## Install

### uv (quickest)

[`uv`](https://docs.astral.sh/uv/) creates the venv and installs deps on the
first `uv run`, so there's no separate install step:

```bash
uv run rover-bridge                       # Pi Camera Module 3 (default; needs rpicam-vid on PATH)
uv run --extra oakd rover-bridge          # OAK-D Lite
uv run --extra realsense rover-bridge     # D435i
uv run rover-bridge --no-camera           # core only (no camera at all)
```

`rover-bridge` is the console entry point; `uv run python -m rover_bridge` is
equivalent. The default camera needs no extra — only the `rpicam-vid` binary on
`PATH`. Pre-install (and write a lockfile) with `uv sync` if you'd rather not
install on first run. The OAK-D / RealSense SDKs are optional extras (lazy
imports), so the core install runs without them — see the Pi 5 ARM notes below
if `pyrealsense2` has no aarch64 wheel.

### Conda (Raspberry Pi 5 / the deployment target)

The core deps are all on conda-forge for `linux-aarch64`; the camera SDKs are
not reliably packaged for conda on ARM, so install whichever one matches your
hardware via pip afterward.

```bash
conda env create -f environment.yml
conda activate rover_bridge
pip install -e '.[oakd]'          # only for an OAK-D; '.[realsense]' for a D435i.
                                  # The default Pi Camera Module 3 needs neither.
```

Update the env after editing `environment.yml`:

```bash
conda env update -f environment.yml --prune
```

### Plain pip

```bash
pip install -e .                  # core (paho-mqtt, pyserial, pillow, numpy, pyyaml)
                                  # — enough for the default Pi Camera Module 3,
                                  #   which needs only rpicam-vid on PATH.
pip install -e '.[oakd]'          # + DepthAI for the OAK-D Lite
pip install -e '.[realsense]'     # + pyrealsense2 for the D435i
```

Camera SDKs are imported lazily, so you only need the one matching your
hardware. `pyserial` is likewise only needed for the UART transport.

> **Pi 5 notes.** `depthai` has ARM64 wheels and pip-installs cleanly;
> `pyrealsense2` often has no prebuilt aarch64 wheel and may need librealsense
> built from source. For the **Pi Camera Module 3**, the backend only needs the
> `rpicam-vid` binary on `PATH` (see *Pi Camera Module 3 on Ubuntu 24.04* below).
> No `picamera2`/`libcamera` Python bindings are required. For the UART
> transport, add your user to the `dialout` group (`sudo usermod -aG dialout
> $USER`, then re-login) so it can open the serial port without root.

### Pi Camera Module 3 on Ubuntu 24.04 (Pi 5)

The `--camera picamera` backend shells out to `rpicam-vid` (from `rpicam-apps`)
and reads its MJPEG stdout, so the **only** requirement is the `rpicam-vid`
binary on `PATH` — no `picamera2`, no `libcamera` Python bindings, and the venv
needs no `--system-site-packages`.

- **Raspberry Pi OS:** `sudo apt install rpicam-apps` — done.
- **Ubuntu 24.04:** there are no Pi camera packages, and the Pi 5 ISP (PiSP) is
  only supported by the *Raspberry Pi fork* of libcamera, so build both from
  source (one-time):

```bash
# 1. Build dependencies
sudo apt update
sudo apt install -y git build-essential pkg-config meson ninja-build cmake \
  python3-venv python3-dev python3-pip pybind11-dev \
  libboost-dev libgnutls28-dev libssl-dev openssl libtiff-dev \
  python3-ply python3-yaml \
  libboost-program-options-dev libexif-dev libavcodec-dev \
  i2c-tools v4l-utils libdrm-dev libjpeg-dev libpng-dev

# 2. Enable the camera in firmware (use cam0/cam1 to match your connector)
#    Add to /boot/firmware/config.txt, then reboot:
#        camera_auto_detect=0
#        dtoverlay=imx708,cam0
sudo usermod -aG video $USER       # then: sudo reboot

# 3. Build the Raspberry Pi libcamera fork (rpi/pisp = Pi 5 support)
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=enabled -Dgstreamer=disabled -Dpycamera=disabled \
  -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled \
  -Ddocumentation=disabled
ninja -C build && sudo ninja -C build install && sudo ldconfig
cd ..

# 4. Build rpicam-apps (provides rpicam-vid in /usr/local/bin)
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build --buildtype=release
ninja -C build && sudo ninja -C build install && sudo ldconfig
cd ..

# 5. Verify before starting the bridge
rpicam-hello --list-cameras        # must list the IMX708
```

> `-Dgstreamer=disabled -Dpycamera=disabled` trims the build to what this
> backend needs (we use neither GStreamer nor the Python bindings); that also
> lets you drop `libglib2.0-dev`/`libgstreamer-plugins-base1.0-dev` from step 1.
> On a board with ≤1 GB RAM, append `-j 1` to the `ninja` commands.

## Run

With `uv` (no activation needed):

```bash
# Defaults: UART link on /dev/ttyAMA0, Pi Camera Module 3, broker localhost.
# Auto-loads config/bridge.yaml.
uv run rover-bridge

# VIO as the active pose source instead of wheel odometry:
uv run rover-bridge --pose-source vio
```

Or with an activated env (conda / `pip install -e`), use `python -m rover_bridge`:

```bash
# Defaults: UART link on /dev/ttyAMA0, Pi Camera Module 3, broker localhost.
# Auto-loads config/bridge.yaml. The YAML's stretch crop matches the GemNav
# training crop; --rotate 180 flips this rover's inverted mount (rpicam's own
# flip flags are ignored on Pi 5).
python -m rover_bridge

# MQTT rover link to a remote broker:
python -m rover_bridge --transport mqtt --broker mqtt-h --robot-id ugv01

# Other cameras (installed as extras):
python -m rover_bridge --camera oakd
python -m rover_bridge --camera realsense

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

`--camera picamera` (default), `--camera oakd`, or `--camera realsense`. All use
only the RGB stream — the model's input. The shared preprocessing (`crop_mode` ∈
`center|top|stretch`, then resize to 224×224 JPEG) matches training-time
preprocessing, so frames are interchangeable across backends.

- **Pi Camera Module 3** (`picamera`, the default) captures by shelling out to `rpicam-vid`
  (from `rpicam-apps`) and reading its MJPEG stdout — no `depthai`/`pyrealsense2`
  and, deliberately, no `picamera2`/`libcamera` Python bindings (which aren't
  packaged on Ubuntu). The only requirement is `rpicam-vid` on `PATH` (override
  with `ROVER_RPICAM_BIN`; legacy `libcamera-vid` is auto-detected). Focus is
  locked to infinity at startup so the lens never hunts while driving. The
  Module 3 sensor is 16:9, so the default `1280×720` is the right aspect; the
  backend forces a full-FOV sensor mode (`2304×1296`, overridable via
  `ROVER_RPICAM_MODE`) so rpicam doesn't auto-pick a cropped-FOV mode. Match the
  model's training crop with `--crop-mode stretch` for the current GemNav
  checkpoint.
- **OAK-D Lite** captures via the DepthAI v3 API (`Camera.requestOutput`), which
  ISP-scales to the exact `--width/--height`. The final inference frame is
  224×224 either way.
- **Inverted mount?** rpicam-vid's `--rotation`/`--hflip`/`--vflip` are silently
  ignored on the Pi 5 (PiSP) pipeline, so rotation is done in Python: set
  `--rotate 180` (or `rotate:` in YAML). `--rotate` (0/90/180/270, clockwise) is
  applied before crop/resize and works for **any** camera backend, not just the
  Pi cam.
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

### The rover won't turn hard enough (grippy surfaces)

A 4-wheel skid-steer has to scrub all four wheels sideways to rotate, and on
grippy asphalt that needs far more yaw authority than the arc geometry asks for.
The gap is bigger than it looks: a pivot commands `target_angle /
actuation_duration`, so a 30° bearing over 1 s is **0.52 rad/s** — a quarter of
the ~2 rad/s that visibly works under manual teleop.

Levers, best first:

1. **`min_drive` in the firmware** — the only one that addresses the physics
   rather than compensating for it. `UGV_MIN_DRIVE_PWM` floors the PWM *while a
   wheel is stalled*, to break stiction, and disengages once rolling so it does
   not flatten the wheel differential mid-turn. Live-tunable, no reflash:
   `../RoverLink/tools/pid_tune.py --broker darkhorse --id ugv01 --min-drive 55`.
2. **`waypoint_index`** (lower = nearer target) — the pure-pursuit lookahead.
   Shorter lookahead means higher curvature, i.e. more turn per unit of forward
   motion. The right lever when it turns but not *tightly* enough.
3. **`angular_action_scale`** — multiplies the angular component only, on top of
   `action_scale`. Use when the rover drives fine but under-turns; `action_scale`
   alone makes it faster *and* turnier, which usually isn't what you want.
4. **`actuation_duration`** (lower) — divides into both velocities, so everything
   gets more aggressive with the arc shape preserved.

Two traps:

- **`min_angular_velocity` is a dead-band, not a floor.** Anything below it is
  snapped to **zero** (`arc_steering.py`), to suppress jittery micro-turns.
  Raising it deletes exactly the weak turns you are trying to strengthen. The
  bridge warns if you combine it with `angular_action_scale > 1`.
- **The caps are applied preserving the linear/angular ratio.** If an arc exceeds
  `max_linear_velocity`, *both* components are scaled down — so the linear cap
  throttles your turn rate, and raising `max_linear_velocity` can *increase* the
  delivered angular. Counter-intuitive, but it falls out of keeping arc geometry
  intact.

`action_scale` and `angular_action_scale` both apply to **remote teleop as well**
as inference. If manual driving is your reference for what the surface allows,
remember that raising these moves that reference too.

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

Two pose sources can run at once. `pose_source` picks the **active** one — the
one that steers the waypoint follower and goes out on `pose_topic`. The other
keeps running as **ground truth**: republished to `gt_pose_topic` and/or logged,
never fed back into control.

- **`wheel`** (default) — integrate the rover's encoders (below). Ground truth is
  then the visual-inertial pose consumed off `vio_pose_topic` (default
  `gemnav/odometry_vio`), published by
  [`rover_vio_iphone`](../rover_vio_iphone) or [`rover_vio`](../rover_vio).
- **`vio`** — steer on that VIO pose instead; host wheel odometry becomes the
  ground-truth stream.

Both frames are REP-103 (x-forward, y-left), so the follower gets compatible
poses either way. VIO is the more accurate source — wheel odometry drifts with
slip, while the ARKit pose shows no stationary drift and survives bumps — and
wheel is the zero-dependency fallback that needs no second process.

**There is no automatic failover.** If the active VIO producer stops publishing,
the follower simply stops getting fresh pose; and a VIO that loses tracking may
keep republishing a *stale* pose, which looks identical to a stationary rover
from here. Nothing downstream notices either case.

**The three pose topics must be distinct**, and the bridge refuses to start
otherwise. Publishing to the topic the VIO producer owns puts two publishers on
one topic and silently interleaves wheel and VIO poses — the check exists because
that is exactly what the shipped config used to do.

### Ground truth: measuring wheel drift against VIO

To run an experiment on wheel odometry while recording VIO as the reference —
the default configuration:

```bash
# publishes VIO to gemnav/odometry_vio (see "Which VIO producer" below)
cd ../rover_vio_iphone && uv run --extra device rover-vio-iphone --broker darkhorse

cd ../rover_bridge && uv run rover-bridge --pose-log-db /data/record/rover/pose_log.db
```

The rover navigates on encoders; VIO is republished to `gemnav/odometry_gt` for
live consumers and both streams land in SQLite for offline comparison:

| Config | Default | Meaning |
|---|---|---|
| `publish_gt_pose` | `true` | republish the ground-truth pose |
| `gt_pose_topic` | `gemnav/odometry_gt` | where it goes |
| `gt_pose_rate_limit` | `10.0` Hz | publish cap; also caps the pose log |
| `pose_log_db` | `null` | SQLite path; `null` disables logging |

The log ([`rover_bridge/pose_log.py`](rover_bridge/pose_log.py)) writes from a
background thread, so disk I/O can't stall the control path or the cmd_vel
heartbeat; if it ever falls behind it drops samples and warns rather than growing
memory. Each process run appends a `run` row, so one file can hold many
experiments:

```sql
run(id, started_at, pose_source, note, goal)
pose_log(id, run_id, timestamp, source, active, x, y, yaw)
goal_log(id, run_id, timestamp, goal)
```

`source` is `wheel` or `vio`, `active` is 1 for whichever drove the follower, and
`timestamp` is host wall clock in ns — the same clock the data logger stamps
images with, so the two line up:

```sql
SELECT timestamp, source, x, y, yaw FROM pose_log
WHERE run_id = (SELECT MAX(id) FROM run) ORDER BY timestamp;
```

Set `publish_gt_pose: false` and leave `pose_log_db` null to switch ground-truth
capture off entirely; the bridge then doesn't subscribe to the VIO topic at all.

### Which VIO producer

Two sibling projects publish that pose, in the same `PoseStamped` JSON contract
on the same topic. The bridge cannot tell them apart, so run **one, never both** —
two publishers on `gemnav/odometry_vio` silently interleave poses from different
frames.

- **[`rover_vio_iphone`](../rover_vio_iphone) — the one to use.** ARKit's pose
  off an iPhone over USB (Record3D). Measured on the rover: ~17 Hz, no
  stationary drift, and no bump-induced runaway. ARKit is factory-calibrated per
  device, so there is no calibration step at all.
- **[`rover_vio`](../rover_vio)** — standalone OpenVINS on a RealSense D435i (no
  ROS). Still supported, and the fallback if no phone is available, but it needs
  a Kalibr cam-IMU solve and ZUPT tuning, and it can run away after the rover
  hits an obstacle.

```bash
cd ../rover_vio_iphone && uv run --extra device rover-vio-iphone --broker darkhorse
# or the D435i path:
cd ../rover_vio && ./build/rover_vio     # stereo by default
```

Then start the bridge — with `--pose-source vio` to steer on it, or with the
default `wheel` to record it as ground truth.

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

The active pose is also streamed to MQTT for the inference side / external
consumers, in the **same format ros_ws used** — a `geometry_msgs/PoseStamped`
serialized to JSON. The ground-truth pose uses the identical format on
`gt_pose_topic`:

```json
{"header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": "odom"},
 "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
          "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
```

`z` is always 0 and the quaternion is yaw-only (the rover is planar). Yaw is
the standard CCW math convention; consumers apply their own heading convention,
exactly as the data logger's `quat_to_rpy` does. Configure with:

- `publish_pose` (default `true`) — enable/disable.
- `pose_topic` (default `gemnav/odometry`) — must match what the consumer
  subscribes to, and must differ from `vio_pose_topic` and `gt_pose_topic`. The
  name matches the Spot bridge's `gemnav/odometry`, which likewise carries plain
  odometry (from the `slam` package's odom output, not a SLAM-corrected pose).
- `pose_rate_limit` (default `10.0` Hz; `null` = every ~50 Hz odometry sample).
- `pose_frame_id` (default `odom`) — also used for the ground-truth pose.

### Battery

The rover reports raw pack voltage + current (`tel/battery`); the bridge
converts pack voltage to a charge percentage for the 3S Li-ion pack and streams
it, with `data` matching ros_ws's `Float32` `charge_percentage` shape:

```json
{"data": 78.0, "voltage_v": 12.05, "current_a": 1.2, "cells": 3}
```

- `publish_battery` (default `true`), `battery_topic` (default `gemnav/battery`).
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
    --transport mqtt --broker mqtt-h --robot-id ugv01 --camera oakd
# with VIO ground truth alongside the wheel pose:
python tools/data_logger.py --base-dir /data/record/rover --frequency 2.0 \
    --broker darkhorse --vio-pose-topic gemnav/odometry_vio
```

Each session writes `images/<timestamp_ns>.jpg` and a `robot_telemetry` table
(`timestamp, image_file, x, y, yaw, gt_x, gt_y, gt_yaw, gt_age_s`); rows are
committed every tick.

`--vio-pose-topic` subscribes to the VIO producer (its own MQTT client,
independent of the rover transport) and fills the `gt_*` columns with the most recent VIO pose
at each tick. `gt_age_s` is how stale that sample was when the row was written —
a large value means there's no usable ground truth for that row, and the columns
are `NULL` when no VIO pose has arrived at all. This is the per-frame equivalent
of the bridge's `pose_log_db`; use the logger when you want images too, the
bridge's log when you're recording a live inference run.

## Troubleshooting: "the camera froze" / "everything is laggy"

These symptoms are misleading, and in practice several causes stack and mask
each other. **Measure before theorising** — [`tools/mqtt_probe.py`](tools/mqtt_probe.py)
answers "is it arriving, at what rate, how evenly" in 20 seconds, and that
separates a camera fault from a network fault from a stale-content fault:

```bash
python tools/mqtt_probe.py --broker darkhorse --duration 30
```

Healthy looks like this — rates matching the config, and `p99` close to `p50`:

| topic | Hz | p50 gap | p99 gap | stalls |
|---|---|---|---|---|
| `gemnav/camera` | 3.00 (`rate_limit`) | 334 ms | 339 ms | none |
| `gemnav/odometry` | ~8–10 (`pose_rate_limit`) | 119 ms | 151 ms | none |
| `gemnav/odometry_vio` | 10.0 | 100 ms | 118 ms | none |
| `gemnav/battery` | 2.0 | 508 ms | 554 ms | none |

A `p50` of 333 ms with a `max` of 8000 ms is not "a bit slow" — it is a stall,
and averages hide it. Then work down this list.

### Everything lags a little, the camera lags a lot → WiFi power save

The classic, and the easiest to misread as a camera fault. Ping the rover:

```bash
ping -c 15 192.168.1.104
```

The signature is a **capable minimum with a terrible average**: `min 3 ms,
avg 90 ms, max 199 ms, mdev 75`, at 0% loss. The radio sleeps between beacons,
so packets wait for the next wake-up. Small, infrequent messages (battery) look
fine; multi-packet camera frames catch a sleep cycle repeatedly and arrive in
bursts. Bandwidth is irrelevant — the whole stream is ~40 KB/s.

```bash
sudo iw dev wlan0 set power_save off        # immediate
```

Persist it across reboots, for every network, with a NetworkManager drop-in
(`/etc/NetworkManager/conf.d/wifi-powersave-off.conf`):

```ini
[connection]
wifi.powersave = 2
```

`nmcli con mod <profile> 802-11-wireless.powersave 2` also works but is
per-profile, so it silently comes back on a network you forgot.

### Camera stalls for seconds, telemetry is fine → capture back-pressure

If the link is healthy and only `gemnav/camera` stalls, suspect the capture
path. `rpicam-vid` writes into a 64 KB kernel pipe — smaller than one 720p MJPEG
frame — so anything that drains it slowly stalls the sensor itself. Fixed by the
reader thread in [`cameras/picamera.py`](rover_bridge/cameras/picamera.py); the
tells if it ever regresses:

- Lowering `fps` makes it **worse**, not better. The problem is back-pressure,
  not data rate.
- The OAK-D is unaffected (`--camera oakd`), because depthai drains the device
  internally. That A/B is the fastest way to confirm.
- `rpicam-vid` run standalone to a *file* is fine — no back-pressure there.

The bridge logs its own capture rate every 10 s, which distinguishes "camera not
producing" from "produced but not delivered":

```
picamera: published 30 frame(s) in 10 s (3.0 Hz)
```

| bridge log | probe | verdict |
|---|---|---|
| ~3 Hz | frames arriving | fine — look downstream |
| ~3 Hz | nothing | network / broker |
| 0 Hz or absent | nothing | camera side |

### Pose arrives punctually but is stale, and gets worse → a queue

If `gemnav/odometry_vio` shows a perfect rate and tiny gaps while the rover's
position visibly trails reality — and the error *grows* — the VIO producer is
outrunning its transport. Delivery rate tells you nothing here; only a physical
move-and-watch does. See the growing-lag section in
[`rover_vio_iphone`](../rover_vio_iphone), whose first suspect is the Record3D
frame rate (an app update reverting to the free version locks it high).

### Nothing arrives at all

Check the bridge's startup log for the topic names it actually resolved — the
`gemnav/*` names are config keys and a stale YAML on the rover is the usual
culprit. `pose_topic`, `gt_pose_topic` and `vio_pose_topic` must be three
distinct topics or the bridge refuses to start, which is itself a useful signal.

## Layout

```
rover_bridge/
  wire.py            # RoverLink wire contract: pack/unpack, CRC8, UART framing FSM
  odometry.py        # diff-drive wheel odometry (ticks -> x,y,yaw)
  pose_log.py        # SQLite log of both pose sources (drift analysis)
  inference.py       # always-MQTT side: camera publish + act/ctrl dispatch
  bridge.py          # orchestrator that wires it all together
  cli.py             # argparse + YAML config (CLI > YAML > default)
  transports/        # rover link: base ABC, uart, mqtt
  control/           # arc_steering, cmd_vel_publisher, waypoint_follower
  cameras/           # base + picamera + oakd + realsense backends, shared preprocess
config/bridge.yaml   # checked-in defaults
tools/data_logger.py # standalone recorder
tools/mqtt_probe.py  # per-topic arrival rates/gaps — start here when "it's laggy"
```
