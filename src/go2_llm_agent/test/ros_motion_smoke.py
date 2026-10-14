"""Real DDS tests with simulated odometry, isolated from Oak."""

import os
import time
import json
import math

if os.getenv("ROS_LOCALHOST_ONLY") != "1" or os.getenv("ROS_DOMAIN_ID") != "87":
    raise SystemExit("Run with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87 (isolated test domain).")

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from go2_llm_agent.agent import AgentSession
from go2_llm_agent.commands import validate_plan
from go2_llm_agent.ros2_motion import RosMotionController


class Client:
    def __init__(self, steps):
        self.plan = validate_plan(json.dumps({"status": "execute", "reason": "Isolated ROS test.", "steps": steps}))

    def interpret(self, text):
        return self.plan


def main():
    rclpy.init()
    probe = Node("go2_llm_test_probe")
    samples = []
    velocity = [0.0, 0.0]

    def record(msg):
        velocity[:] = [msg.linear.x, msg.angular.z]
        samples.append((msg.linear.x, msg.angular.z))

    sub = probe.create_subscription(Twist, "/cmd_vel_agent", record, 100)
    odom_pub = probe.create_publisher(Odometry, "/odom", 1)
    motion = RosMotionController()
    motion.start()
    session = None
    yaw = math.radians(170)  # Both angle tests cross the +/-pi boundary.
    updated_at = time.monotonic()

    def tick():
        nonlocal yaw, updated_at
        now = time.monotonic()
        yaw += velocity[1] * (now - updated_at)
        updated_at = now
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        odom_pub.publish(msg)
        rclpy.spin_once(probe, timeout_sec=0.005)
        time.sleep(0.02)  # Fixed sensor cadence; do not flood DDS when callbacks are ready.

    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            tick()
            if (probe.count_publishers("/cmd_vel_agent") == 1
                    and probe.count_subscribers("/odom") == 1
                    and motion.read_heading() is not None):
                break
        assert probe.count_publishers("/cmd_vel_agent") == 1, "Unexpected/missing test publisher"
        assert probe.count_subscribers("/odom") == 1, "Unexpected/missing test odometry subscriber"
        assert motion.read_heading() is not None, "Test odometry was not received"

        cases = [
            ("timed backward", [{"action":"walk", "direction":"backward", "quantity":0.3, "unit":"seconds"}], None, None),
            ("default left turn", [{"action":"turn", "direction":"left", "quantity":None, "unit":"default"}], 90, 1),
            ("full clockwise spin", [{"action":"spin", "direction":"right", "quantity":None, "unit":"default"}], 360, -1),
        ]
        for name, steps, angle, sign in cases:
            initial_yaw, initial_sample = yaw, len(samples)
            session = AgentSession(Client(steps), motion)
            done = False
            session.submit("test")
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                tick()
                if session._worker is not None and not session._worker.is_alive():
                    done = True
                    break
            assert done, "Test worker did not finish"
            session.wait()
            # Drain the final zero through DDS before assessing the measured result.
            drain = time.monotonic() + 0.2
            while time.monotonic() < drain:
                tick()
            observed = samples[initial_sample:]
            assert session.last_error is None, (session.last_error, math.degrees(yaw - initial_yaw), len(observed), observed[-5:], motion.read_heading())
            assert observed and observed[-1] == (0, 0), observed[-5:]
            if angle is None:
                assert any(x == -0.2 and z == 0 for x, z in observed), observed
            else:
                measured = sign * math.degrees(yaw - initial_yaw)
                assert angle - 3 <= measured <= angle + 3, (name, measured)
                assert any(z * sign > 0 and x == 0 for x, z in observed), observed
                print(f"PASS: {name}; measured {measured:.2f} degrees across heading wraparound.")
            print(f"PASS: {name}; {len(observed)} DDS messages, ending in zero.")
            session = None  # Same backend is intentionally reused across independent sessions.
    finally:
        if session is not None:
            session.close()
        else:
            motion.close()
        probe.destroy_subscription(sub)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
