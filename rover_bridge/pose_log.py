# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""SQLite log of both pose sources, for offline drift analysis.

Wheel odometry drifts with slip; VIO does not (much). When you run an
experiment on wheel odometry you usually want the VIO pose recorded alongside
it as a reference, so the drift can be quantified after the fact rather than
guessed at. This module records *both* streams — one row per sample, tagged
with its source — into a single table on a shared host clock.

Rows are written by a background thread: MQTT callbacks and the odometry
integrator hand samples to a queue and return immediately, so disk I/O can
never stall the control path or the cmd_vel heartbeat. If the queue backs up
(slow disk, huge rate limit) samples are dropped with a throttled warning
rather than growing memory without bound.

The navigation goal (``gemnav/goal``) is recorded too, so a logged run says
what it was driving *to*, not just where it went.

Schema::

    run(id, started_at, pose_source, note, goal)
    pose_log(id, run_id, timestamp, source, active, x, y, yaw)
    goal_log(id, run_id, timestamp, goal)

``source`` is ``'wheel'`` or ``'vio'``; ``active`` is 1 for the source that was
driving the waypoint follower. ``timestamp`` is host wall clock in ns, the same
clock ``tools/data_logger.py`` stamps images with, so the two line up.

Compare the two tracks with e.g.::

    SELECT timestamp, source, x, y, yaw FROM pose_log
    WHERE run_id = (SELECT MAX(id) FROM run) ORDER BY timestamp;
"""

from __future__ import annotations

import logging
import os
import queue
import sqlite3
import threading
import time
from typing import Optional

from .logging_util import get_logger, log_throttle

log = get_logger("pose_log")

_SENTINEL = object()

_SCHEMA = """
CREATE TABLE IF NOT EXISTS run (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at  INTEGER,
    pose_source TEXT,
    note        TEXT,
    goal        TEXT          -- latest gemnav/goal payload seen this run
);
CREATE TABLE IF NOT EXISTS pose_log (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id    INTEGER,
    timestamp INTEGER NOT NULL,
    source    TEXT NOT NULL,
    active    INTEGER NOT NULL,
    x REAL, y REAL, yaw REAL
);
CREATE INDEX IF NOT EXISTS idx_pose_log_ts ON pose_log(timestamp);
CREATE INDEX IF NOT EXISTS idx_pose_log_run ON pose_log(run_id, source);
CREATE TABLE IF NOT EXISTS goal_log (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id    INTEGER,
    timestamp INTEGER NOT NULL,
    goal      TEXT NOT NULL   -- gemnav/goal payload; each row is one change
);
CREATE INDEX IF NOT EXISTS idx_goal_log_run ON goal_log(run_id);
"""


class PoseLog:
    """Background-thread SQLite writer for wheel + VIO poses.

    Each process run appends a ``run`` row, so several experiments can share one
    database file and still be separable.
    """

    def __init__(self, path: str, pose_source: str = "wheel",
                 rate_limit: Optional[float] = 10.0, note: Optional[str] = None,
                 queue_size: int = 4096, commit_interval: float = 1.0):
        """
        Args:
            path: SQLite file. Parent directories are created; an existing file
                is appended to (a new ``run`` row separates this session).
            pose_source: recorded on the run row for context ("wheel"/"vio").
            rate_limit: per-source cap in Hz on logged samples (wheel telemetry
                arrives at ~50 Hz). None logs every sample.
            note: free-text label for the run row (e.g. the experiment name).
            queue_size: max pending samples before dropping.
            commit_interval: seconds between commits; a crash loses at most this.
        """
        self.path = path
        self.rate_limit = rate_limit
        self.commit_interval = commit_interval

        parent = os.path.dirname(os.path.abspath(path))
        if parent:
            os.makedirs(parent, exist_ok=True)

        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.executescript(_SCHEMA)
        # `run.goal` was added after the first version shipped; CREATE TABLE IF
        # NOT EXISTS won't add it to a database written by that version.
        cols = {r[1] for r in self._conn.execute("PRAGMA table_info(run)")}
        if "goal" not in cols:
            self._conn.execute("ALTER TABLE run ADD COLUMN goal TEXT")
        cur = self._conn.execute(
            "INSERT INTO run (started_at, pose_source, note) VALUES (?, ?, ?)",
            (time.time_ns(), pose_source, note))
        self.run_id = cur.lastrowid
        self._conn.commit()

        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._last_logged: dict[str, float] = {}
        self._dropped = 0
        self._written = 0
        self._closed = False
        self._thread = threading.Thread(target=self._writer, name="pose-log",
                                        daemon=True)
        self._thread.start()
        log.info("pose log: run %d -> %s (rate_limit=%s Hz)",
                 self.run_id, path, self.rate_limit)

    # --- producer side (called from callback threads) -----------------------

    def record(self, source: str, x: float, y: float, yaw: float,
               active: bool) -> None:
        """Queue one pose sample. Never blocks; drops if the writer is behind."""
        if self._closed:
            return
        if self.rate_limit is not None:
            now = time.monotonic()
            last = self._last_logged.get(source)
            if last is not None and (now - last) < 1.0 / self.rate_limit:
                return
            # Benign race: two threads may both pass this check for the same
            # source and log an extra sample. Harmless for a drift log, and
            # cheaper than serializing the control path on a lock.
            self._last_logged[source] = now
        try:
            self._queue.put_nowait(
                ("pose",
                 (self.run_id, time.time_ns(), source, 1 if active else 0, x, y, yaw)))
        except queue.Full:
            self._dropped += 1
            log_throttle(log, logging.WARNING, 10.0,
                         f"pose log queue full — dropped {self._dropped} sample(s); "
                         "lower pose_log rate_limit or check disk speed")

    def record_goal(self, goal_json: str) -> None:
        """Queue a ``gemnav/goal`` change: one ``goal_log`` row, and it becomes
        the run's current goal. Not rate-limited — goals change rarely."""
        if self._closed:
            return
        try:
            self._queue.put_nowait(("goal", (self.run_id, time.time_ns(), goal_json)))
        except queue.Full:
            log.warning("pose log queue full — goal not recorded")

    # --- consumer side ------------------------------------------------------

    def _writer(self) -> None:
        pending = False
        last_commit = time.monotonic()
        while True:
            try:
                item = self._queue.get(timeout=self.commit_interval)
            except queue.Empty:
                item = None
            if item is _SENTINEL:
                break
            if item is not None:
                kind, row = item
                try:
                    if kind == "pose":
                        self._conn.execute(
                            "INSERT INTO pose_log (run_id, timestamp, source, active, "
                            "x, y, yaw) VALUES (?, ?, ?, ?, ?, ?, ?)", row)
                        self._written += 1
                    else:
                        self._conn.execute(
                            "INSERT INTO goal_log (run_id, timestamp, goal) "
                            "VALUES (?, ?, ?)", row)
                        self._conn.execute("UPDATE run SET goal = ? WHERE id = ?",
                                           (row[2], row[0]))
                    pending = True
                except sqlite3.Error as e:
                    log_throttle(log, logging.ERROR, 10.0,
                                 f"pose log insert failed ({kind}): {e}")
            now = time.monotonic()
            if pending and (now - last_commit) >= self.commit_interval:
                self._commit()
                pending, last_commit = False, now
        self._commit()

    def _commit(self) -> None:
        try:
            self._conn.commit()
        except sqlite3.Error as e:
            log.error("pose log commit failed: %s", e)

    def close(self) -> None:
        """Flush the queue, commit, and close. Safe to call twice."""
        if self._closed:
            return
        self._closed = True
        try:
            self._queue.put(_SENTINEL, timeout=2.0)
        except queue.Full:
            log.warning("pose log queue full at shutdown; some samples lost")
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            log.warning("pose log writer did not finish within 5 s")
        try:
            self._conn.close()
        except sqlite3.Error as e:
            log.warning("error closing pose log: %s", e)
        log.info("pose log: run %d wrote %d row(s) to %s (%d dropped)",
                 self.run_id, self._written, self.path, self._dropped)
