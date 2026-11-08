"""Heading feedback shared by the ROS backend and offline checks."""

from dataclasses import dataclass
import math
import threading
import time

TURN_TOLERANCE_RAD = math.radians(1)
SETTLE_DURATION_S = 0.35
SETTLE_SPREAD_RAD = math.radians(0.5)
MIN_TURN_SPEED_RAD_S = 0.10
TURN_GAIN = 1.5


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
                # Large pose resets must not count as physical rotation. This is
                # a generous discontinuity check, not a motion/accuracy estimate.
                elapsed = now - self._heading.received_at
                if elapsed <= 0 or abs(delta) > 4.0 * elapsed + math.radians(10):
                    self._error = "Odometry heading jumped unexpectedly; stopped. Check /odom and restart the agent."
                    return
                self._total += delta
            self._previous = yaw
            self._heading = Heading(self._total, now)

    def read(self):
        with self._lock:
            if self._error:
                raise RuntimeError(self._error)
            return self._heading


class AngleTurnController:
    """Follow measured, unwrapped heading; stop and verify before declaring success."""

    def __init__(self, initial, angle_deg, direction, max_speed):
        self.initial = initial.radians
        self.sign = 1 if direction == 'left' else -1
        self.target = self.initial + self.sign * math.radians(angle_deg)
        self.tolerance = min(TURN_TOLERANCE_RAD, math.radians(angle_deg) / 2)
        self.max_speed = max_speed
        self.complete = False
        self.error = self.target - self.initial
        self.measured = 0.0
        self._last_received = initial.received_at
        self._settling = []

    def update(self, heading):
        self.error = self.target - heading.radians
        self.measured = self.sign * (heading.radians - self.initial)
        if self.measured < -math.radians(10):
            raise RuntimeError("Heading moved opposite to the requested turn; stopped.")
        new_sample = heading.received_at > self._last_received
        if new_sample:
            self._last_received = heading.received_at
        if abs(self.error) <= self.tolerance:
            if new_sample:
                self._settling.append(heading)
                values = [sample.radians for sample in self._settling]
                if max(values) - min(values) > min(SETTLE_SPREAD_RAD, self.tolerance):
                    self._settling = [heading]
                span = heading.received_at - self._settling[0].received_at
                self.complete = len(self._settling) >= 3 and span >= SETTLE_DURATION_S
            return 0.0
        self._settling.clear()
        self.complete = False
        # Smaller measured error -> slower turn. Overshoot produces a slow
        # corrective turn rather than an unconditional success at first crossing.
        speed = min(self.max_speed, max(MIN_TURN_SPEED_RAD_S, TURN_GAIN * abs(self.error)))
        if self.error * self.sign < 0:
            speed = min(speed, 0.35)
        return math.copysign(speed, self.error)
