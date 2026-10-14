"""Heading feedback shared by the ROS backend and offline checks."""

from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class Heading:
    radians: float
    received_at: float


class HeadingTracker:
    """Unwrap successive yaw samples so a full circle is not mistaken for zero."""

    def __init__(self):
        self._lock = threading.Lock()
        self._previous = None
        self._total = 0.0
        self._heading = None
        self._frame = None
        self._error = None

    def update(self, quaternion, frame):
        x, y, z, w = quaternion
        if not all(math.isfinite(v) for v in quaternion):
            return
        norm = math.hypot(*quaternion)
        if not math.isfinite(norm) or norm < 1e-6:
            return
        x, y, z, w = (v / norm for v in quaternion)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        now = time.monotonic()
        with self._lock:
            if self._frame is not None and self._frame != frame:
                self._error = "Odometry frame changed; restart the agent after checking /odom."
                return
            self._frame = frame
            if self._previous is not None:
                delta = math.atan2(math.sin(yaw - self._previous), math.cos(yaw - self._previous))
                self._total += delta
            self._previous = yaw
            self._heading = Heading(self._total, now)

    def read(self):
        with self._lock:
            if self._error:
                raise RuntimeError(self._error)
            return self._heading
