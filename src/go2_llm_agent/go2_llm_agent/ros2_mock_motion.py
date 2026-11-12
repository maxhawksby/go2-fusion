"""Simulation backend. Does not import ROS, connect to Oak, or publish topics."""

import time
from go2_llm_agent.heading import Heading


class MockMotionController:
    def __init__(self, output=print):
        self.output = output
        self._previous = None
        self._yaw = 0.0
        self._angular = 0.0
        self._updated_at = time.monotonic()

    def read_heading(self):
        now = time.monotonic()
        self._yaw += self._angular * (now - self._updated_at)
        self._updated_at = now
        return Heading(self._yaw, now)

    def start(self):
        self.output("[SIMULATION] Controller ready; no ROS publishers.")

    def publish_twist(self, linear_x, angular_z=0.0):
        self.read_heading()
        self._angular = angular_z
        velocity = (linear_x, angular_z)
        if velocity != self._previous:
            self.output(f"[SIMULATION] velocity: forward={linear_x:g} m/s, left-turn={angular_z:g} rad/s")
            self._previous = velocity

    def publish_stop(self):
        self.publish_twist(0.0, 0.0)

    def close(self):
        pass  # Session.close sends the stop when execution is enabled.
