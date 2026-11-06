#!/usr/bin/env python3
"""
Asks the SocialNav model which way to walk, and shows its answer.

Never drives.

Every few seconds it sends the last 5 camera frames (1 s apart), where the robot was for each, and
the goal to the SocialNav server, and publishes the 5 waypoints it predicts as /socialnav/path
(nav_msgs/Path, map frame) for RViz. Each waypoint is checked against the lidar; /socialnav/status
says how many have room and how long the model took.

The goal is the explorer's current one (/frontier_explorer/goal) when it runs, otherwise a point
goal_ahead_m in front of where the robot was when this started.
"""

import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

from go2_auto_mapper.explorer import encode_jpeg, stop_when_parent_dies
from go2_auto_mapper.socialnav import History, build_request, lidar_room, post, to_map

DEFAULT_URL = 'http://localhost:8100/waypoints'
# Room the lidar must see beyond a waypoint for it to count as reachable.
ROOM_MARGIN_M = 0.3
SCAN_MAX_AGE_SEC = 2.0


class SocialNavView(Node):

    def __init__(self):
        super().__init__('socialnav_view')
        self.url = self.declare_parameter(
            'url', os.environ.get('SOCIALNAV_URL', DEFAULT_URL)).value
        self.frame_id = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.goal_ahead = self.declare_parameter('goal_ahead_m', 3.0).value
        self.goal_left = self.declare_parameter('goal_left_m', 0.0).value
        period = self.declare_parameter('period_sec', 3.0).value
        self.timeout = self.declare_parameter('timeout_sec', 30.0).value

        self.history = History()
        self.frame = None
        self.scan = None
        self.scan_time = 0.0
        self.goal = None
        self.fallback_goal = None
        self.busy = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(PoseStamped, '/frontier_explorer/goal', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/socialnav/path', 10)
        self.status_pub = self.create_publisher(String, '/socialnav/status', 10)
        self.create_timer(0.2, self.remember)
        self.create_timer(period, self.ask)
        self.get_logger().info(f'Asking SocialNav at {self.url}; it never drives the robot.')

    def camera_callback(self, msg):
        self.frame = msg

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, rclpy.time.Time()).transform
        except TransformException:
            return None
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def remember(self):
        pose = self.pose()
        if pose is None or self.frame is None:
            return
        if self.fallback_goal is None:
            self.fallback_goal = to_map(-self.goal_left, self.goal_ahead, *pose)
        self.history.offer(time.monotonic(), *pose, self.frame)

    def ask(self):
        if self.busy or not self.history.full():
            return
        goal = self.goal or self.fallback_goal
        samples = list(self.history.samples)
        self.busy = True
        threading.Thread(target=self.plan, args=(samples, goal), daemon=True).start()

    def plan(self, samples, goal):
        try:
            started = time.monotonic()
            reply = post(self.url, build_request(samples, *goal, encode_jpeg), self.timeout)
            self.publish(samples[-1], reply['waypoints'], time.monotonic() - started, goal)
        except Exception as e:   # a down server must not end the node
            self.status_pub.publish(String(data=f'SocialNav not answering: {e}'))
            self.get_logger().warn(f'SocialNav not answering: {e}', throttle_duration_sec=30.0)
        finally:
            self.busy = False

    def publish(self, newest, waypoints, seconds, goal):
        _, x, y, yaw, _ = newest
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for right, forward in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = to_map(right, forward, x, y, yaw)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        scan = self.scan
        fresh = scan is not None and time.monotonic() - self.scan_time < SCAN_MAX_AGE_SEC
        rooms = [lidar_room(scan.ranges, scan.angle_min, scan.angle_increment, r, f, ROOM_MARGIN_M)
                 if fresh else None for r, f in waypoints]
        last = waypoints[-1]
        self.status_pub.publish(String(data=(
            f'{sum(r is True for r in rooms)} of {len(rooms)} waypoints have lidar room '
            f'({sum(r is None for r in rooms)} unchecked); path ends {last[1]:.2f} m ahead, '
            f'{last[0]:.2f} m right; goal ({goal[0]:.2f}, {goal[1]:.2f}); {seconds:.1f} s')))


def main(args=None):
    stop_when_parent_dies()
    rclpy.init(args=args)
    node = SocialNavView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
