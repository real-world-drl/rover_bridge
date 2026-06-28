# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with
code in this repository.

## What this is

**rover_bridge** — the host-side GemNav inference bridge for the RoverLink
differential rover. It is the non-ROS sibling of `../../ros_ws` (the Spot
ROS↔MQTT bridge): same model, same MQTT inference contract, but it drives
`../RoverLink` over that firmware's native binary `cmd_vel`/telemetry instead
of publishing a ROS `Twist`. Pure Python.

For run instructions, config keys, and the transport/camera matrix, see
`README.md` — don't duplicate it here.

## Verify

There is no test suite yet. "Verified" means:

```bash
python3 -m py_compile rover_bridge/*.py rover_bridge/**/*.py tools/*.py
python -m rover_bridge --config '' --transport mqtt --no-camera   # constructs + connects
```

The core logic (wire round-trip, odometry integration, arc steering, the
publisher buffer→zero phases, halt/start dispatch) is pure and testable without
hardware — exercise it directly rather than spinning up a broker. The camera
and serial paths need real devices.

## Architecture, in one paragraph

`bridge.py:RoverBridge` wires five pieces. The **inference side is always MQTT**
(`inference.py:InferenceClient`): it publishes preprocessed camera frames and
subscribes to `gemnav/act` + `gemnav/ctrl`. Actions flow action →
`WaypointFollower` (arc steering) → `RepeatedCmdVelPublisher` → the **rover
transport**. The rover transport is the *only* selectable link — `transports/`
has a `RoverTransport` ABC with `uart.py` (default) and `mqtt.py`
implementations; both send `cmd_vel`/`cmd_pid`/`cmd_display` and fire
`TelemetryCallbacks` on inbound telemetry. Wheel telemetry feeds
`odometry.py:WheelOdometry`, which integrates cumulative ticks into `(x,y,yaw)`
and pushes pose back into the follower for waypoint advance. The repeated
`cmd_vel` publish doubles as the firmware heartbeat.

## Things to know before changing code

### `rover_bridge/wire.py` is the wire contract — keep it in lockstep
It mirrors `../RoverLink/main/ugv_packets.h` (and `../RoverLink/tools/ugv_packets.py`):
packed little-endian structs, CRC8 (poly 0x07), and the
`[0xA5][type][len][payload][crc8]` UART framing. There are import-time size
asserts matching the firmware's `_Static_assert`s — if you change a struct,
change it on both sides and bump the topic version (`v1` → `v2`) rather than
breaking the format in place. Don't invent a different CRC.

### The transport is the only thing `--transport` switches
Camera-out, `gemnav/act`, and `gemnav/ctrl` are MQTT regardless of transport.
A common mistake would be to route inference I/O through the rover transport —
don't. The rover MQTT transport (`transports/mqtt.py`) is a *separate* paho
client from the inference client, even on the same broker, so the two concerns
stay decoupled.

### cmd_vel is the heartbeat — never let the publish rate drop near 2 Hz
RoverLink zeroes the motors if no `cmd_vel` arrives within
`UGV_HEARTBEAT_TIMEOUT_MS` (~500 ms). `RepeatedCmdVelPublisher` republishes at
`publish_rate` (default 10 Hz) precisely to satisfy this. `cli._validate` warns
below 2 Hz. Anything that injects motion must go through the publisher, not
straight to `transport.send_cmd_vel`, or the heartbeat/stop-phase logic is
bypassed.

### Odometry geometry must match the firmware Kconfig
`wheel_diameter_mm` / `track_width_mm` / `encoder_ppr` mirror
`UGV_WHEEL_DIAMETER_MM` / `UGV_TRACK_WIDTH_MM` / `UGV_ENCODER_PPR`. If the host
pose disagrees with the rover's, check these first. Ticks are *cumulative* and
signed; the integrator differences successive samples, so the first sample only
establishes a baseline (no motion emitted).

### Camera SDKs and pyserial are imported lazily — keep them that way
`depthai`, `pyrealsense2`, `picamera2`/`libcamera`, and `serial` are imported
inside `open_device()` / `start()`, not at module top, so a host with only one
camera (or MQTT-only) can still import and run. Don't hoist these imports to
module scope.

### Differential drive changed two arc-steering defaults
`turn_in_place_threshold_deg` is a real 45° (the rover pivots in place) rather
than Spot's effectively-disabled 180°, and the velocity caps are rover-scale.
The arc geometry itself is unchanged from `ros_ws/arc_steering.py`.

### The data logger and the bridge can't share a camera
A camera opens exclusively in one process. `tools/data_logger.py` opens its own
transport + camera; run it instead of the bridge, or start the bridge with
`--no-camera`.

### Ported from ros_ws — preserve the parallel structure
`control/cmd_vel_publisher.py` ↔ `twist_publisher.py`,
`control/waypoint_follower.py` ↔ `waypoint_follower.py`,
`inference.py` ↔ `mqtt_client_manager.py`,
`cameras/realsense.py` ↔ `realsense_camera_service.py`. When fixing a bug that
also exists in `ros_ws`, note it; the logic was intentionally kept aligned so
behavior matches across the two robots.

## Memory & user context

The user prefers packed-binary transport over JSON (the inference *payloads*
are JSON because that's the model's contract; the *rover* link is binary). This
started as a Python project "for now" — flag if Python becomes a constraint
(e.g. UART RX latency). The rover is small, differential-steered, and has
working encoders that yield usable x/y odometry (the RoverLink docs that say
"no encoders" are stale).
