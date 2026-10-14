"""Existing agent's ROS backend, limited to its twist_mux movement input."""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from go2_llm_agent.heading import HeadingTracker


class RosMotionController:
    def __init__(self, cmd_vel_topic="/cmd_vel_agent"):
        self._topic = cmd_vel_topic
        self._node = None
        self._context = None
        self._heading = HeadingTracker()
        self._last_odom_stamp = None

    def start(self):
        if self._node is not None:
            return
        self._context = Context()
        rclpy.init(args=[], context=self._context)
        self._node = Node("go2_llm_agent", context=self._context)
        self._pub = self._node.create_publisher(Twist, self._topic, 10)
        self._odom_sub = self._node.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_odom_stamp is not None and stamp <= self._last_odom_stamp:
            return  # Repeated/out-of-order telemetry must not make old data look fresh.
        self._last_odom_stamp = stamp
        q = msg.pose.pose.orientation
        self._heading.update((q.x, q.y, q.z, q.w), msg.header.frame_id)

    def read_heading(self):
        return self._heading.read()

    def publish_twist(self, linear_x, angular_z=0.0):
        if self._node is None:
            raise RuntimeError("Motion controller is not running.")
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear_x), float(angular_z)
        self._pub.publish(msg)

    def publish_stop(self):
        self.publish_twist(0.0, 0.0)

    def close(self):
        if self._node is not None:
            try:
                self.publish_stop()
                # Give DDS a bounded opportunity to send the final zero before destruction.
                self._pub.wait_for_all_acked(rclpy.duration.Duration(seconds=0.2))
            finally:
                self._executor.shutdown(timeout_sec=1.0)
                self._thread.join(timeout=1.0)
                self._node.destroy_node()
                self._node = None
                self._context.shutdown()
