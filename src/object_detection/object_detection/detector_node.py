import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

import numpy as np
import json
import math
import os

from object_detection.people import PeopleTracker, people_dict, to_odom

# How often the object memory is logged. Logging it on every frame floods
# the console and the /rosout topic.
MEMORY_LOG_PERIOD_SEC = 5.0
# People are tracked in this frame so their walking speed does not include
# Oak's own motion.
PEOPLE_FRAME = "odom"
TF_WARN_PERIOD_SEC = 10.0


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
        # Minimum YOLO confidence for a detection. Below YOLO's own default
        # (0.25) so more objects are found, at the cost of a few more false
        # positives; raise it if you see too many.
        self.confidence = float(self.declare_parameter("confidence", 0.15).value)
        self.min_period_sec = 1.0 / max_rate_hz if max_rate_hz > 0 else 0.0
        self.last_inference_sec = 0.0

        torch.set_num_threads(num_threads)
        self.get_logger().info(
            f"Loading YOLO model from {model_path} "
            f"({num_threads} CPU threads, max {max_rate_hz} Hz, "
            f"confidence >= {self.confidence})"
        )
        self.model = YOLO(model_path)
        # Open-vocabulary weights (YOLOE, as in SocialNav) look for any names
        # you give them, e.g. "person,chair,red box"; empty keeps the
        # model's own classes.
        classes = [
            c.strip() for c in self.declare_parameter("classes", "").value.split(",")
            if c.strip()
        ]
        if classes and hasattr(self.model, "set_classes"):
            self.model.set_classes(classes)
            self.get_logger().info(f"Looking for: {classes}")
        elif classes:
            self.get_logger().error(
                f"{model_path} has fixed classes; `classes` needs YOLOE weights, "
                "e.g. model_path:=yoloe-26s-seg.pt"
            )

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

        # Not /detected_objects: coco_detector already publishes
        # vision_msgs/Detection2DArray there, and two message types on one
        # topic break both.
        self.object_pose_pub = self.create_publisher(
            PoseStamped,
            "/detected_objects/poses",
            10
        )

        self.dict_pub = self.create_publisher(
            String,
            "/detected_objects_dict",
            10
        )

        # Persistent memory
        self.object_map = {}

        # Horizontal camera FOV (approximate)
        self.horizontal_fov = np.deg2rad(90)

        # Object expiry time (seconds)
        self.expiry_time = 20

        # People tracking with walking forecasts and groups, from SocialNav.
        self.people = None
        if self.declare_parameter("track_people", True).value:
            self.people = PeopleTracker()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.people_pub = self.create_publisher(String, "/people", 10)
            self.people_markers_pub = self.create_publisher(
                MarkerArray, "/people/markers", 10
            )

        self.get_logger().info("RGB object detector with memory ready")

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
        results = self.model(frame, conf=self.confidence, verbose=False)

        # Drawing and sending a full-size image is costly, so skip it when
        # nobody (e.g. an RViz Image display) is watching.
        if self.annotated_pub.get_subscription_count() > 0:
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.annotated_pub.publish(annotated_msg)

        image_width = frame.shape[1]
        image_height = frame.shape[0]

        current_time = self.get_clock().now().nanoseconds / 1e9
        person_points = []

        # Process detections
        for box in results[0].boxes:

            cls = int(box.cls[0])
            label = self.model.names[cls]

            x1, y1, x2, y2 = box.xyxy[0]

            center_x = (x1 + x2) / 2
            box_height = (y2 - y1)

            # Angle calculation
            normalized_x = (center_x - image_width / 2) / (image_width / 2)
            angle = normalized_x * (self.horizontal_fov / 2)

            # Rough distance estimate
            distance_estimate = 1.0 / (box_height / image_height + 1e-6)

            x_pos = float(distance_estimate * math.cos(angle))
            y_pos = float(distance_estimate * math.sin(angle))
            if label == "person":
                person_points.append((x_pos, y_pos))

            matched_id = None

            # Try matching existing objects of same class
            for obj_id, obj_data in self.object_map.items():

                if not obj_id.startswith(label):
                    continue

                dx = obj_data["x"] - x_pos
                dy = obj_data["y"] - y_pos
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < 0.5:  # match threshold (meters approx)
                    matched_id = obj_id
                    break

            # If no match, create new ID
            if matched_id is None:
                index = sum(1 for k in self.object_map if k.startswith(label)) + 1
                matched_id = f"{label}_{index}"

            # Update memory
            self.object_map[matched_id] = {
                "x": round(x_pos, 2),
                "y": round(y_pos, 2),
                "last_seen": current_time
            }

            # Publish pose for RViz
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos
            pose.pose.position.z = 0.0

            self.object_pose_pub.publish(pose)

        if self.people is not None:
            self.update_people(person_points, current_time)

        # Expire old objects
        to_delete = []

        for obj_id, obj_data in self.object_map.items():
            if current_time - obj_data["last_seen"] > self.expiry_time:
                to_delete.append(obj_id)

        for obj_id in to_delete:
            del self.object_map[obj_id]

        # Publish dictionary without timestamps
        clean_dict = {
            k: {"x": v["x"], "y": v["y"]}
            for k, v in self.object_map.items()
        }

        json_msg = String()
        json_msg.data = json.dumps(clean_dict)
        self.dict_pub.publish(json_msg)

        if clean_dict:
            self.get_logger().info(
                f"Memory: {clean_dict}", throttle_duration_sec=MEMORY_LOG_PERIOD_SEC
            )


    # ==========================================================
    # People
    # ==========================================================
    def update_people(self, points, now):
        try:
            tf = self.tf_buffer.lookup_transform(
                PEOPLE_FRAME, "base_link", Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f"No {PEOPLE_FRAME} pose, people not tracked: {e}",
                throttle_duration_sec=TF_WARN_PERIOD_SEC,
            )
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.people.update([to_odom(x, y, t.x, t.y, yaw) for x, y in points], now)
        people = self.people.people(now)

        msg = String()
        msg.data = json.dumps(people_dict(people))
        self.people_pub.publish(msg)
        if self.people_markers_pub.get_subscription_count() > 0:
            self.people_markers_pub.publish(self.people_markers(people))

    def people_markers(self, people):
        """Sphere per person, a line along their forecast, a ring per group."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()

        def marker(kind, marker_id, r, g, b):
            m = Marker()
            m.header.frame_id = PEOPLE_FRAME
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = "people", marker_id, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.8
            markers.markers.append(m)
            return m

        groups = {}
        for i, p in enumerate(people):
            dot = marker(Marker.SPHERE, 3 * i, 1.0, 0.5, 0.0)
            dot.pose.position.x, dot.pose.position.y = p.x, p.y
            dot.scale.x = dot.scale.y = dot.scale.z = 0.3
            path = marker(Marker.LINE_STRIP, 3 * i + 1, 1.0, 1.0, 0.0)
            path.scale.x = 0.05
            path.points = [Point(x=x, y=y) for x, y in [(p.x, p.y)] + p.path]
            if p.group is not None:
                groups.setdefault(p.group, []).append((p.x, p.y))
        for g, members in groups.items():
            cx = sum(x for x, _ in members) / len(members)
            cy = sum(y for _, y in members) / len(members)
            radius = max(math.hypot(x - cx, y - cy) for x, y in members) + 0.5
            ring = marker(Marker.CYLINDER, 3 * len(people) + g, 0.2, 0.6, 1.0)
            ring.pose.position.x, ring.pose.position.y = cx, cy
            ring.scale.x = ring.scale.y = 2.0 * radius
            ring.scale.z = 0.05
            ring.color.a = 0.3
        return markers


def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()