import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import json
import math


class ObjectNavigator(Node):

    def __init__(self):
        super().__init__("object_navigator")

        # Subscribe to object dictionary
        self.dict_sub = self.create_subscription(
            String,
            "/detected_objects_dict",
            self.dict_callback,
            10
        )

        # Subscribe to user commands
        self.command_sub = self.create_subscription(
            String,
            "/navigate_to_object",
            self.command_callback,
            10
        )

        # Publish velocity on this node's own twist_mux input (lowest priority:
        # it has no obstacle avoidance, so every other source overrides it)
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel_object_nav",
            10
        )

        self.objects = {}
        self.target_object = None

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Object navigator ready")

    def dict_callback(self, msg):
        try:
            self.objects = json.loads(msg.data)
        except ValueError:
            self.objects = {}

    def command_callback(self, msg):
        target = msg.data.strip()

        if target in self.objects:
            self.target_object = target
            self.get_logger().info(f"Navigating to {target}")
        else:
            self.get_logger().warn(f"{target} not found")

    def control_loop(self):

        if self.target_object is None:
            return

        if self.target_object not in self.objects:
            self.get_logger().warn("Target disappeared")
            self.stop()
            self.target_object = None
            return

        x = self.objects[self.target_object]["x"]
        y = self.objects[self.target_object]["y"]

        distance = math.sqrt(x*x + y*y)
        angle = math.atan2(y, x)

        cmd = Twist()

        # Angular control
        cmd.angular.z = 1.5 * angle

        # Linear control (only move forward if mostly facing object)
        if abs(angle) < 0.3:
            cmd.linear.x = 0.5 * distance

        # Stop if close enough
        if distance < 0.5:
            self.get_logger().info("Arrived at object")
            self.stop()
            self.target_object = None
            return

        # Clamp speeds
        cmd.linear.x = max(min(cmd.linear.x, 0.4), 0.0)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ObjectNavigator()
    rclpy.spin(node)
    rclpy.shutdown()
