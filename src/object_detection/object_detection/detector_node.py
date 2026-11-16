import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
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
import cv2
import numpy as np
import json
import math
import os
import time

from object_detection.people import PeopleTracker, people_dict
from object_detection.remote_yolo import (
    build_request,
    DeadlineCaller,
    Detection,
    JPEG_QUALITY,
    parse_detections,
    post_json,
    RETRY_SEC,
    send_size,
    ServerGate,
)

# How often the object memory is logged. Logging it on every frame floods
# the console and the /rosout topic.
MEMORY_LOG_PERIOD_SEC = 5.0

# A detection this close (meters) to a remembered object of the same class is
# taken to be that object.
MATCH_RADIUS_M = 0.5
# People are tracked in this frame so their walking speed does not include
# Oak's own motion; the auto mapper reads /people in it.
PEOPLE_FRAME = "odom"


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
        self.model_path = model_path
        # Open-vocabulary weights (YOLOE, as in SocialNav) look for any names
        # you give them, e.g. "person,chair,red box"; empty keeps the
        # model's own classes.
        self.classes = [
            c.strip() for c in self.declare_parameter("classes", "").value.split(",")
            if c.strip()
        ]
        # A YOLO server on a GPU machine (host_tools/yolo_server/server.py),
        # usually through an SSH tunnel. Empty runs YOLO here on the CPU. When
        # the server fails, the CPU model takes over for RETRY_SEC.
        self.inference_url = self.declare_parameter(
            "inference_url", os.environ.get("YOLO_URL", "")
        ).value
        timeout = float(self.declare_parameter("inference_timeout_sec", 1.0).value)
        if not (math.isfinite(timeout) and timeout > 0):
            self.get_logger().warn(f"inference_timeout_sec {timeout} is not > 0; using 1.0")
            timeout = 1.0
        self.inference_timeout = timeout
        self.server_gate = ServerGate()
        self.model = None
        # When loading or running the CPU model failed, when to try again.
        self.local_retry_after = 0.0
        # Runs server requests with a hard deadline (see detect_on_server).
        self.server_caller = DeadlineCaller()
        if self.inference_url:
            self.get_logger().info(
                f"Asking the YOLO server at {self.inference_url} (max {max_rate_hz} Hz, "
                f"min confidence {self.min_confidence}); the CPU model loads only "
                "if it fails"
            )
        else:
            self.get_logger().info(
                f"{num_threads} CPU threads, max {max_rate_hz} Hz, "
                f"min confidence {self.min_confidence}"
            )
            self.load_local_model()

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

        # People tracking with walking forecasts and groups, from SocialNav.
        self.people = None
        if self.declare_parameter("track_people", True).value:
            self.people = PeopleTracker()
            self.people_pub = self.create_publisher(String, "/people", 10)
            self.people_markers_pub = self.create_publisher(
                MarkerArray, "/people/markers", 10
            )

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
    # Detecting: on the YOLO server or here
    # ==========================================================
    def load_local_model(self):
        """Load and configure the CPU model; self.model is set only once it is ready."""
        self.get_logger().info(f"Loading YOLO model from {self.model_path}")
        model = YOLO(self.model_path)
        if self.classes and hasattr(model, "set_classes"):
            model.set_classes(self.classes)
            self.get_logger().info(f"Looking for: {self.classes}")
        elif self.classes:
            self.get_logger().error(
                f"{self.model_path} has fixed classes; `classes` needs YOLOE weights, "
                "e.g. model_path:=yoloe-26s-seg.pt"
            )
        self.model = model

    def detect(self, frame):
        """
        Return the boxes in frame (full-size pixels) and a way to draw them.

        Asks the server when one is set and answering, else runs YOLO here.
        The second value draws ultralytics' own picture for the local model,
        and is None for the server's boxes. Returns ([], None) when neither
        works, so a bad frame never ends the node.
        """
        if self.inference_url and self.server_gate.use_server(time.monotonic()):
            try:
                detections = self.detect_on_server(frame)
            except Exception as e:   # a down server or tunnel must not end the node
                if self.server_gate.failed(time.monotonic()):
                    self.get_logger().warn(
                        f"YOLO server not answering ({e}); using the CPU model, "
                        f"trying the server again every {RETRY_SEC:.0f} s"
                    )
            else:
                if self.server_gate.succeeded():
                    self.get_logger().info("YOLO server answering again")
                return detections, None
        return self.detect_locally(frame)

    def detect_locally(self, frame):
        if time.monotonic() < self.local_retry_after:
            return [], None
        try:
            if self.model is None:
                self.load_local_model()
            # verbose=False: ultralytics otherwise prints a line per frame.
            # conf: boxes below min_confidence are dropped here, so they are
            # neither remembered nor drawn.
            result = self.model(frame, conf=self.min_confidence, verbose=False)[0]
            detections = [
                Detection(self.model.names[int(box.cls[0])], float(box.conf[0]),
                          *(float(v) for v in box.xyxy[0]))
                for box in result.boxes
            ]
        except Exception as e:   # e.g. weights missing or not downloadable
            self.local_retry_after = time.monotonic() + RETRY_SEC
            self.get_logger().error(
                f"CPU YOLO failed ({e}); no detections for {RETRY_SEC:.0f} s"
            )
            return [], None
        return detections, result.plot

    def detect_on_server(self, frame):
        """
        Ask the server, waiting at most inference_timeout seconds.

        The request runs on a daemon thread (DeadlineCaller) so a server that
        trickles its reply can't hold up this callback or the node's exit.
        """
        full_size = (frame.shape[1], frame.shape[0])
        sent_size = send_size(*full_size)
        small = frame if sent_size == full_size else cv2.resize(frame, sent_size)
        ok, jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise ValueError("JPEG encoding failed")
        body = build_request(jpeg.tobytes(), self.min_confidence, self.classes)
        reply = self.server_caller.call(
            post_json, self.inference_url, body, self.inference_timeout,
            timeout=self.inference_timeout,
        )
        return parse_detections(reply, sent_size, full_size)

    def draw(self, frame, detections):
        """Return a copy of frame with each server box and its label drawn on."""
        annotated = frame.copy()
        for d in detections:
            corner = (int(d.x1), int(d.y1))
            cv2.rectangle(annotated, corner, (int(d.x2), int(d.y2)), (0, 165, 255), 2)
            cv2.putText(
                annotated, f"{d.label} {d.confidence:.2f}", (corner[0], max(12, corner[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1,
            )
        return annotated

    # ==========================================================
    # Main Callback
    # ==========================================================
    def rgb_callback(self, msg):

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_inference_sec < self.min_period_sec:
            return
        self.last_inference_sec = now_sec

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detections, plot = self.detect(frame)

        # Drawing and sending a full-size image is costly, so skip it when
        # nobody (e.g. an RViz Image display) is watching.
        if self.annotated_pub.get_subscription_count() > 0:
            annotated = None
            if plot is not None:
                try:
                    annotated = plot()
                except Exception as e:   # e.g. the label font can't be downloaded
                    self.get_logger().warn(
                        f"YOLO's own drawing failed ({e}); drawing plain boxes",
                        throttle_duration_sec=MEMORY_LOG_PERIOD_SEC,
                    )
            if annotated is None:
                annotated = self.draw(frame, detections)
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
        # Each person's position as measured, in the source frame, for the
        # people tracker (which needs PEOPLE_FRAME, not the memory's frame).
        person_points = []

        # Process detections
        for detection in detections:

            label = detection.label
            confidence = detection.confidence
            x1, y1, x2, y2 = detection.x1, detection.y1, detection.x2, detection.y2

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
            if label == "person":
                person_points.append(PointStamped(header=point.header, point=point.point))
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
        if self.people is not None:
            self.update_people(person_points, source, msg.header.stamp, current_time)

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

    # ==========================================================
    # People
    # ==========================================================

    def update_people(self, points, source, stamp, now):
        """Track this frame's people in PEOPLE_FRAME and publish /people."""
        if source == self.robot_frame and not self.tf_buffer.can_transform(
            PEOPLE_FRAME, source, Time()
        ):
            self.get_logger().warn(
                f"No {PEOPLE_FRAME} pose, people not tracked",
                throttle_duration_sec=MEMORY_LOG_PERIOD_SEC,
            )
            return
        try:
            transform = self.lookup_transform(PEOPLE_FRAME, source, stamp)
        except TransformException as e:
            self.get_logger().warn(
                f"No TF {source} -> {PEOPLE_FRAME}, people not tracked: {e}",
                throttle_duration_sec=MEMORY_LOG_PERIOD_SEC,
            )
            return
        odom = [do_transform_point(p, transform).point for p in points]
        self.people.update([(p.x, p.y) for p in odom], now)
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
