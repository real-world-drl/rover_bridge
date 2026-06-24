# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""rover_bridge — GemNav inference bridge for the RoverLink differential rover.

Mirrors the ROS↔MQTT bridge (``../../ros_ws``) but talks to the RoverLink
firmware (``../RoverLink``) over its native binary contract instead of ROS:
cmd_vel goes out over UART or MQTT as a packed ``ugv_cmd_vel_t``, telemetry
comes back the same way, and the GemNav inference loop (camera frames out,
``gemnav/act`` + ``gemnav/ctrl`` in) runs over MQTT exactly as before.
"""

__version__ = "0.1.0"
