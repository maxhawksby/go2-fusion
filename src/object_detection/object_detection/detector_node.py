import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point

from collections import defaultdict
import numpy as np
import json
import math
import os

# How often the object memory is logged. Logging it on every frame floods
# the console and the /rosout topic.
MEMORY_LOG_PERIOD_SEC = 5.0

# A detection this close (meters) to a remembered object of the same class is
# taken to be that object.
MATCH_RADIUS_M = 0.5


class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self.bridge = CvBridge()
        # Defaults to the weights installed with this package; override with
        # `--ros-args -p model_path:=/path/to/weights.pt`.
        default_model = os.path.join(
            get_package_share_directory("object_detection"), "models", "yolov8n.pt"
        )
        model_path = self.declare_parameter("model_path", default_model).value
        # On CPU, PyTorch uses every core by default, which starves the driver,
        # SLAM, Nav2 and RViz running in the same container.
        num_threads = self.declare_parameter("num_threads", 4).value
        # Inference runs at most this often; frames in between are dropped.
        max_rate_hz = self.declare_parameter("max_rate_hz", 5.0).value
        self.min_period_sec = 1.0 / max_rate_hz if max_rate_hz > 0 else 0.0
        self.last_inference_sec = 0.0
        # YOLO's own default (0.25) favors recall: on a live feed it flickers
        # through many wrong boxes, and each one becomes an ID in the memory
        # that the navigator or LLM agent may act on. 0.5 keeps the clear ones.
        self.min_confidence = self.declare_parameter("min_confidence", 0.5).value

        # The memory lives in a fixed frame so a remembered object stays put
        # while the robot turns or walks. map needs SLAM or localization
        # running; odom always exists while the driver runs but drifts.
        self.fixed_frame = self.declare_parameter("fixed_frame", "map").value
        self.fallback_frame = self.declare_parameter("fallback_frame", "odom").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value
        # How long to wait for the TF at the image's timestamp before using
        # the latest TF instead.
        self.tf_timeout = Duration(
            seconds=self.declare_parameter("tf_timeout_sec", 0.1).value
        )
        # spin_thread: the lookup below waits for TF inside a callback, so the
        # listener needs its own thread to keep receiving it meanwhile.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        torch.set_num_threads(num_threads)
        self.get_logger().info(
            f"Loading YOLO model from {model_path} "
            f"({num_threads} CPU threads, max {max_rate_hz} Hz, "
            f"min confidence {self.min_confidence})"
        )
        self.model = YOLO(model_path)

        # Depth 1: when inference falls behind, only the newest frame is kept
        # instead of a backlog of stale ones.
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.rgb_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.rgb_callback,
            camera_qos
        )

        # Publishers
        self.annotated_pub = self.create_publisher(
            Image,
            "/detections/image_annotated",
            10
        )

        # Labeled markers for RViz. Not /detected_objects: coco_detector
        # already publishes vision_msgs/Detection2DArray there, and two
        # message types on one topic break both.
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/detected_objects/markers",
            10
        )

        self.dict_pub = self.create_publisher(
            String,
            "/detected_objects_dict",
            10
        )

        # Persistent memory, positions in self.memory_frame
        self.object_map = {}
        self.memory_frame = None
        # Next ID number per label. It only grows, so an ID is never reused
        # while another object of that class still holds it.
        self.next_index = defaultdict(lambda: 1)
        # (target, source) frames in use, to log only when they change
        self.frames_in_use = None

        # Horizontal camera FOV (approximate)
        self.horizontal_fov = np.deg2rad(90)

        # Object expiry time (seconds)
        self.expiry_time = 20

        self.get_logger().info("RGB object detector with memory ready")

    # ==========================================================
    # Frames
    # ==========================================================
    def choose_frames(self, camera_frame):
        """
        Pick the frame to remember objects in and the frame to measure from.

        Returns (target, source). The source is the camera when TF knows it,
        else the robot body. With no usable TF at all, both are the robot
        body, which is the old robot-relative behavior.
        """
        sources = [f for f in (camera_frame, self.robot_frame) if f]
        for target in (self.fixed_frame, self.fallback_frame):
            for source in sources:
                if self.tf_buffer.can_transform(target, source, Time()):
                    return target, source
        return self.robot_frame, self.robot_frame

    def move_memory(self, target):
        """
        Re-express the memory in a new frame, e.g. map once SLAM starts.

        Objects keep their IDs. Without a TF between the two frames the
        memory is cleared: positions in one frame mean nothing in another.
        """
        old = self.memory_frame
        self.memory_frame = target
        if old is None or not self.object_map:
            return
        try:
            transform = self.tf_buffer.lookup_transform(target, old, Time())
        except TransformException:
            self.get_logger().warn(f"No TF {old} -> {target}: forgetting all objects")
            self.object_map = {}
            return
        for obj in self.object_map.values():
            point = PointStamped()
            point.point.x, point.point.y, point.point.z = obj["x"], obj["y"], obj["z"]
            point = do_transform_point(point, transform)
            obj["x"] = round(point.point.x, 2)
            obj["y"] = round(point.point.y, 2)
            obj["z"] = round(point.point.z, 2)

    def lookup_transform(self, target, source, stamp):
        """TF at the image's timestamp, or the latest one if that is not available."""
        if Time.from_msg(stamp).nanoseconds > 0:
            try:
                return self.tf_buffer.lookup_transform(
                    target, source, Time.from_msg(stamp), timeout=self.tf_timeout
                )
            except TransformException:
                pass
        return self.tf_buffer.lookup_transform(target, source, Time())

    # ==========================================================
    # Main Callback
    # ==========================================================
    def rgb_callback(self, msg):

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_inference_sec < self.min_period_sec:
            return
        self.last_inference_sec = now_sec

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # verbose=False: ultralytics otherwise prints a line per frame.
        # conf: boxes below min_confidence are dropped here, so they are
        # neither remembered nor drawn.
        results = self.model(frame, conf=self.min_confidence, verbose=False)

        # Drawing and sending a full-size image is costly, so skip it when
        # nobody (e.g. an RViz Image display) is watching.
        if self.annotated_pub.get_subscription_count() > 0:
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.annotated_pub.publish(annotated_msg)

        target, source = self.choose_frames(msg.header.frame_id)
        if (target, source) != self.frames_in_use:
            self.frames_in_use = (target, source)
            if target == self.robot_frame:
                self.get_logger().warn(
                    f"No TF from {self.fixed_frame} or {self.fallback_frame} to "
                    f"{self.robot_frame}: remembering objects relative to the robot, "
                    "so they move when it does"
                )
            else:
                self.get_logger().info(
                    f"Remembering objects in frame {target}, measured from {source}"
                )
        if target != self.memory_frame:
            self.move_memory(target)

        transform = None
        if target != source:
            try:
                transform = self.lookup_transform(target, source, msg.header.stamp)
            except TransformException as e:
                self.get_logger().warn(
                    f"Skipping frame, no TF {source} -> {target}: {e}",
                    throttle_duration_sec=MEMORY_LOG_PERIOD_SEC,
                )
                return
        # sensor_msgs/Image frames are optical: x right, y down, z forward.
        # The robot body is x forward, y left, z up.
        source_is_optical = source != self.robot_frame

        image_width = frame.shape[1]
        image_height = frame.shape[0]

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Process detections
        for box in results[0].boxes:

            cls = int(box.cls[0])
            label = self.model.names[cls]
            confidence = float(box.conf[0])

            x1, y1, x2, y2 = box.xyxy[0]

            center_x = (x1 + x2) / 2
            box_height = (y2 - y1)

            # Bearing, positive when the object is right of the image center
            normalized_x = (center_x - image_width / 2) / (image_width / 2)
            angle = normalized_x * (self.horizontal_fov / 2)

            # Rough distance estimate
            distance_estimate = float(1.0 / (box_height / image_height + 1e-6))

            point = PointStamped()
            point.header.frame_id = source
            right = distance_estimate * math.sin(angle)
            forward = distance_estimate * math.cos(angle)
            if source_is_optical:
                point.point.x = float(right)
                point.point.z = float(forward)
            else:
                point.point.x = float(forward)
                point.point.y = float(-right)
            if transform is not None:
                point = do_transform_point(point, transform)
            x_pos = point.point.x
            y_pos = point.point.y

            # Match the nearest remembered object of the same class
            matched_id = None
            best_dist = MATCH_RADIUS_M
            for obj_id, obj_data in self.object_map.items():

                if obj_data["label"] != label:
                    continue

                dist = math.hypot(obj_data["x"] - x_pos, obj_data["y"] - y_pos)
                if dist < best_dist:
                    matched_id = obj_id
                    best_dist = dist

            # If no match, create new ID
            if matched_id is None:
                matched_id = f"{label}_{self.next_index[label]}"
                self.next_index[label] += 1

            # Update memory
            self.object_map[matched_id] = {
                "x": round(x_pos, 2),
                "y": round(y_pos, 2),
                "z": round(point.point.z, 2),
                "label": label,
                "confidence": round(confidence, 2),
                "last_seen": current_time
            }

        # Expire old objects
        to_delete = []

        for obj_id, obj_data in self.object_map.items():
            if current_time - obj_data["last_seen"] > self.expiry_time:
                to_delete.append(obj_id)

        for obj_id in to_delete:
            del self.object_map[obj_id]

        self.publish_markers()
        self.publish_dict()

    def publish_dict(self):
        """
        Publish the memory as JSON, x/y relative to the robot as it is now.

        Consumers (object_navigation, the dashboard bridge) read x/y as
        robot-relative, so they are recomputed from the remembered map
        position with the latest TF. map_x, map_y, frame, label and
        confidence were added later; older readers ignore them.
        """
        to_robot = None
        if self.memory_frame != self.robot_frame:
            try:
                to_robot = self.tf_buffer.lookup_transform(
                    self.robot_frame, self.memory_frame, Time()
                )
            except TransformException as e:
                self.get_logger().warn(
                    f"Not publishing memory, no TF {self.memory_frame} -> "
                    f"{self.robot_frame}: {e}",
                    throttle_duration_sec=MEMORY_LOG_PERIOD_SEC,
                )
                return

        clean_dict = {}
        for obj_id, obj in self.object_map.items():
            point = PointStamped()
            point.point.x = obj["x"]
            point.point.y = obj["y"]
            point.point.z = obj["z"]
            if to_robot is not None:
                point = do_transform_point(point, to_robot)
            clean_dict[obj_id] = {
                "x": round(point.point.x, 2),
                "y": round(point.point.y, 2),
                "map_x": obj["x"],
                "map_y": obj["y"],
                "frame": self.memory_frame,
                "label": obj["label"],
                "confidence": obj["confidence"],
            }

        json_msg = String()
        json_msg.data = json.dumps(clean_dict)
        self.dict_pub.publish(json_msg)

        if clean_dict:
            positions = {k: (v["map_x"], v["map_y"]) for k, v in clean_dict.items()}
            self.get_logger().info(
                f"Memory ({self.memory_frame}): {positions}",
                throttle_duration_sec=MEMORY_LOG_PERIOD_SEC,
            )

    def publish_markers(self):
        """Show the memory in RViz: a sphere plus an ID label per object."""
        markers = MarkerArray()
        # Clear last round's markers first, so forgotten objects disappear.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (obj_id, obj) in enumerate(sorted(self.object_map.items())):
            sphere = Marker()
            # Stamp 0: draw with the latest TF; the positions do not move.
            sphere.header.frame_id = self.memory_frame
            sphere.ns = "objects"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = obj["x"]
            sphere.pose.position.y = obj["y"]
            sphere.pose.position.z = obj["z"]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color.r, sphere.color.g, sphere.color.b = 1.0, 0.5, 0.0
            sphere.color.a = 0.8
            markers.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.memory_frame
            text.ns = "labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = obj["x"]
            text.pose.position.y = obj["y"]
            text.pose.position.z = obj["z"] + 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{obj_id} ({obj['confidence']:.2f})"
            markers.markers.append(text)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()
