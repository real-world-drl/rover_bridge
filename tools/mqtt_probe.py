#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Measure what is actually arriving on the broker, per topic.

The first tool to reach for when the rover "feels laggy" or the camera "stops".
It answers the only question that matters up front — is the data arriving, at
what rate, and how evenly — which is what separates a camera fault from a
network fault from a consumer fault. Every symptom in the README's
Troubleshooting section was diagnosed with this.

Run it from the broker host (or anywhere with a route to it) while the bridge
is running::

    python tools/mqtt_probe.py                       # 20 s of gemnav/#
    python tools/mqtt_probe.py --duration 30 --topics 'gemnav/camera'
    python tools/mqtt_probe.py --broker darkhorse --duration 60

Reading the output:

- **Hz** against what the config asks for. `gemnav/camera` should equal
  ``rate_limit``, `gemnav/odometry` should equal ``pose_rate_limit``.
- **p50 gap** is the real cadence. For a 3 Hz cap, expect ~333 ms.
- **p99 / max gap** is where faults hide. A p50 of 333 ms with a max of 8000 ms
  is not "a bit slow", it is a stall — and the average will not show it.
- **stalls** counts gaps over ``--stall-ms``: the total dead time in the window.

A steady rate with healthy gaps means the bridge and the link are fine, and the
problem is downstream or in the *content* of the messages (a pose can arrive
punctually and still be stale — see the growing-lag note in rover_vio_iphone).
"""

import argparse
import collections
import statistics
import time

import paho.mqtt.client as mqtt


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--broker", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--topics", default="gemnav/#", help="subscription filter")
    p.add_argument("--duration", type=float, default=20.0, help="seconds to listen")
    p.add_argument("--stall-ms", type=float, default=700.0,
                   help="gaps above this count as stalls")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    stats = collections.defaultdict(lambda: {"n": 0, "bytes": 0, "t": []})

    def on_connect(client, _u, _f, rc):
        if rc != 0:
            print(f"connect failed (rc={rc})")
            return
        client.subscribe(args.topics)
        print(f"listening {args.duration:.0f}s on {args.topics} "
              f"@ {args.broker}:{args.port}\n")

    def on_message(_c, _u, msg):
        s = stats[msg.topic]
        s["n"] += 1
        s["bytes"] += len(msg.payload)
        s["t"].append(time.monotonic())

    client = mqtt.Client()
    client.on_connect, client.on_message = on_connect, on_message
    client.connect(args.broker, args.port, 30)
    client.loop_start()
    t0 = time.monotonic()
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        pass
    client.loop_stop()
    elapsed = time.monotonic() - t0

    if not stats:
        print(f"NOTHING received on {args.topics}.\n"
              "The bridge is not publishing, not connected, or is publishing to "
              "different topic names. Check its startup log.")
        return 1

    print(f"{'topic':<24}{'msgs':>6}{'Hz':>7}{'avg KB':>8}{'KB/s':>8}"
          f"{'p50':>7}{'p99':>7}{'max':>8}  stalls")
    print("-" * 88)
    for topic in sorted(stats):
        s = stats[topic]
        gaps = sorted((b - a) * 1e3 for a, b in zip(s["t"], s["t"][1:]))
        if len(gaps) >= 2:
            p50 = statistics.median(gaps)
            p99 = gaps[min(int(len(gaps) * 0.99), len(gaps) - 1)]
            big = [g for g in gaps if g > args.stall_ms]
            stall = (f"{len(big)} ({sum(big) / 1000:.1f}s of {elapsed:.0f}s)"
                     if big else "none")
            g50, g99, gmax = f"{p50:.0f}", f"{p99:.0f}", f"{gaps[-1]:.0f}"
        else:
            g50 = g99 = gmax = "-"
            stall = "n/a"
        print(f"{topic:<24}{s['n']:>6}{s['n'] / elapsed:>7.2f}"
              f"{s['bytes'] / s['n'] / 1024:>8.1f}{s['bytes'] / elapsed / 1024:>8.1f}"
              f"{g50:>7}{g99:>7}{gmax:>8}  {stall}")
    print("\ngap columns are milliseconds between consecutive messages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
