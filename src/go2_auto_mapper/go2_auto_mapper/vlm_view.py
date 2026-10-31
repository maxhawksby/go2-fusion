#!/usr/bin/env python3
"""Shows what the VLM sees: asks it to describe the camera every few seconds
and suggest a next move, which it checks against the lidar.

A node of its own because the explorer drives the robot, and this should work
with the robot standing still. It only reads the camera and publishes text and
pictures, never a motion command. The RViz AutoMapper panel's VLM view button
starts it and shows /vlm/view and /vlm/description. In vlm_mode active the
explorer reads /vlm/suggestion and favors goals that way; Nav2 still drives.
"""

import math
import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

from go2_auto_mapper.explorer import encode_jpeg, stop_when_parent_dies
from go2_auto_mapper.vlm_ranking import (
    MOVES,
    build_describe_request,
    clear_distance,
    describe_move,
    lidar_sentence,
    parse_suggestion,
    post_chat,
    suggestion_message,
)

DEFAULT_URL = 'http://localhost:8000/v1/chat/completions'
DEFAULT_MODEL = 'Qwen/Qwen2.5-VL-7B-Instruct'
# An older scan says nothing about what is there now.
SCAN_MAX_AGE_SEC = 2.0
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class VlmView(Node):

    def __init__(self):
        super().__init__('vlm_view')
        self.declare_parameter('vlm_url', os.environ.get('VLM_URL', DEFAULT_URL))
        self.declare_parameter('vlm_model', os.environ.get('VLM_MODEL', DEFAULT_MODEL))
        self.declare_parameter('period_sec', 5.0)
        self.declare_parameter('timeout_sec', 90.0)   # the first answer loads the model
        self.url = self.get_parameter('vlm_url').value
        self.model = self.get_parameter('vlm_model').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.api_key = os.environ.get('VLM_API_KEY')

        self.frame = None
        self.busy = False
        self.scan = None
        self.scan_time = 0.0
        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, camera_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, camera_qos)
        # Latched, so the panel gets the last answer whenever it subscribes.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.view_pub = self.create_publisher(CompressedImage, '/vlm/view', latched)
        self.text_pub = self.create_publisher(String, '/vlm/description', latched)
        self.suggestion_pub = self.create_publisher(String, '/vlm/suggestion', latched)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(self.get_parameter('period_sec').value, self.ask)
        self.get_logger().info(f'Describing the camera with {self.model} at {self.url}.')

    def camera_callback(self, msg):
        self.frame = msg

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def map_yaw(self):
        """The robot's heading on the map, or None if TF can't tell."""
        try:
            q = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time()).transform.rotation
        except TransformException:
            return None
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def suggestion_text(self, reply, frame_yaw):
        """The panel's text: the move, what the lidar thinks of it, the
        description. Also publishes the suggestion for the explorer."""
        description, move, reason = parse_suggestion(reply)
        if move is None:
            return description
        scan = self.scan
        have_scan = scan is not None and time.monotonic() - self.scan_time < SCAN_MAX_AGE_SEC
        # The move was meant from where the picture was taken; the robot may
        # have turned since the question went out.
        bearing = MOVES[move][0]
        heading = None
        yaw_now = self.map_yaw()
        if bearing is not None and frame_yaw is not None:
            heading = frame_yaw + bearing
        nearest = None
        if have_scan and bearing is not None:
            relative = bearing if heading is None or yaw_now is None else heading - yaw_now
            nearest = clear_distance(scan.ranges, scan.angle_min, scan.angle_increment, relative)
        self.suggestion_pub.publish(
            String(data=suggestion_message(move, heading, nearest, time.time())))
        lines = [f'Suggested move: {MOVES[move][1]}. {reason}',
                 describe_move(move, nearest, have_scan), description]
        return '\n'.join(line for line in lines if line)

    def ask(self):
        """One request at a time, off the spin thread: answers can take a while."""
        if self.busy or self.frame is None:
            return
        frame, self.frame = self.frame, None
        self.busy = True
        threading.Thread(
            target=self.describe, args=(frame, self.map_yaw()), daemon=True).start()

    def describe(self, frame, frame_yaw):
        try:
            jpeg = encode_jpeg(frame)
            view = CompressedImage(format='jpeg', data=jpeg)
            view.header = frame.header
            self.view_pub.publish(view)
            scan = self.scan
            if scan is not None and time.monotonic() - self.scan_time < SCAN_MAX_AGE_SEC:
                lidar = lidar_sentence(scan.ranges, scan.angle_min, scan.angle_increment)
                payload = build_describe_request(self.model, jpeg, lidar)
            else:
                payload = build_describe_request(self.model, jpeg)
            text = self.suggestion_text(
                post_chat(self.url, payload, self.timeout, self.api_key), frame_yaw)
        except Exception as e:   # a down server must not end the node
            text = f'VLM not answering: {e}'
            self.get_logger().warn(text, throttle_duration_sec=30.0)
        self.text_pub.publish(String(data=text))
        self.busy = False


def main(args=None):
    stop_when_parent_dies()
    rclpy.init(args=args)
    node = VlmView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
