"""ROS 2 to HTTP bridge for the Oak Control dashboard."""

import asyncio
import json
import queue
import threading
import time

from aiohttp import web
import cv2
from cv_bridge import CvBridge
import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from go2_interfaces.msg import Go2State, WebRtcReq

from .protocol import SPORT_ACTIONS, normalize_objects, parse_voice_command


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("go2_dashboard_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("allow_commands", False)
        self.declare_parameter("state_topic", "/go2_states")
        self.declare_parameter("objects_topic", "/detected_objects_dict")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("annotated_camera_topic", "/detections/image_annotated")
        self.declare_parameter("sport_topic", "/webrtc_req")
        self.declare_parameter("tts_topic", "/tts")
        self.declare_parameter("driver_node", "/go2_driver_node")

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.allow_commands = bool(self.get_parameter("allow_commands").value)
        self.state_topic = self.get_parameter("state_topic").value
        self.objects_topic = self.get_parameter("objects_topic").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.annotated_camera_topic = self.get_parameter("annotated_camera_topic").value
        self.sport_topic = self.get_parameter("sport_topic").value
        self.tts_topic = self.get_parameter("tts_topic").value
        self.driver_node = self.get_parameter("driver_node").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Go2State, self.state_topic, self._state_callback, sensor_qos)
        self.create_subscription(String, self.objects_topic, self._objects_callback, 10)
        self.create_subscription(Image, self.camera_topic, self._raw_camera_callback, sensor_qos)
        self.create_subscription(Image, self.annotated_camera_topic, self._annotated_camera_callback, sensor_qos)
        self.sport_publisher = self.create_publisher(WebRtcReq, self.sport_topic, 10)
        self.tts_publisher = self.create_publisher(String, self.tts_topic, 10)
        parameter_prefix = self.driver_node.rstrip("/")
        self.get_driver_parameters = self.create_client(
            GetParameters, f"{parameter_prefix}/get_parameters"
        )
        self.set_driver_parameters = self.create_client(
            SetParameters, f"{parameter_prefix}/set_parameters"
        )

        self._last_state = 0.0
        self._last_objects = 0.0
        self._objects = []
        self._object_error = None
        self._cv_bridge = CvBridge()
        self._camera_lock = threading.Lock()
        self._camera_jpeg = None
        self._camera_source = None
        self._last_camera_encode = 0.0
        self._last_annotated_message = 0.0
        self._camera_error = None
        self._obstacle_avoidance = None
        self._obstacle_pending = False
        self._obstacle_query_in_flight = False
        self._obstacle_error = None
        self._commands = queue.SimpleQueue()
        self.create_timer(0.05, self._drain_commands)
        self.create_timer(1.0, self._refresh_obstacle_avoidance)

        self._loop = None
        self._runner = None
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()
        mode = "ENABLED" if self.allow_commands else "DISABLED"
        self.get_logger().info(f"Dashboard API listening on {self.host}:{self.port}; commands {mode}")

    def _state_callback(self, _message):
        self._last_state = time.monotonic()

    def _objects_callback(self, message):
        try:
            self._objects = normalize_objects(message.data)
            self._object_error = None
            self._last_objects = time.monotonic()
        except (ValueError, json.JSONDecodeError) as exc:
            self._object_error = str(exc)

    def _raw_camera_callback(self, message):
        self._camera_callback(message, "raw")

    def _annotated_camera_callback(self, message):
        self._last_annotated_message = time.monotonic()
        self._camera_callback(message, "annotated")

    def _camera_callback(self, message, source):
        now = time.monotonic()
        if source == "raw" and now - self._last_annotated_message < 1.0:
            return
        if now - self._last_camera_encode < 0.2:
            return
        self._last_camera_encode = now
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            encoded, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not encoded:
                raise ValueError("OpenCV could not encode the camera frame")
            with self._camera_lock:
                self._camera_jpeg = jpeg.tobytes()
                self._camera_source = source
                self._camera_error = None
        except Exception as exc:
            self._camera_error = str(exc)

    def _refresh_obstacle_avoidance(self):
        if self._obstacle_pending or self._obstacle_query_in_flight:
            return
        if not self.get_driver_parameters.service_is_ready():
            return
        self._obstacle_query_in_flight = True
        request = GetParameters.Request()
        request.names = ["obstacle_avoidance"]
        future = self.get_driver_parameters.call_async(request)
        future.add_done_callback(self._obstacle_query_done)

    def _obstacle_query_done(self, future):
        self._obstacle_query_in_flight = False
        try:
            response = future.result()
            if not response.values:
                raise RuntimeError("Driver did not return obstacle_avoidance")
            self._obstacle_avoidance = bool(response.values[0].bool_value)
            self._obstacle_error = None
        except Exception as exc:
            self._obstacle_error = str(exc)

    def _set_obstacle_avoidance(self, enabled):
        if not self.set_driver_parameters.service_is_ready():
            self._obstacle_pending = False
            self._obstacle_error = f"Parameter service for {self.driver_node} is unavailable"
            return
        self._obstacle_pending = True
        self._obstacle_error = None
        request = SetParameters.Request()
        request.parameters = [Parameter("obstacle_avoidance", value=enabled).to_parameter_msg()]
        future = self.set_driver_parameters.call_async(request)
        future.add_done_callback(lambda result: self._obstacle_set_done(result, enabled))

    def _obstacle_set_done(self, future, enabled):
        self._obstacle_pending = False
        try:
            response = future.result()
            if not response.results or not response.results[0].successful:
                reason = response.results[0].reason if response.results else "No result returned"
                raise RuntimeError(reason)
            self._obstacle_avoidance = enabled
            self._obstacle_error = None
            self.get_logger().info(f"Dashboard set obstacle avoidance to {enabled}")
        except Exception as exc:
            self._obstacle_error = str(exc)

    def _topic_online(self, topic):
        return self.count_publishers(topic) > 0

    def _status(self):
        now = time.monotonic()
        state_age = None if not self._last_state else round(now - self._last_state, 2)
        object_age = None if not self._last_objects else round(now - self._last_objects, 2)
        return {
            "robot_connected": state_age is not None and state_age < 3.0,
            "command_enabled": self.allow_commands,
            "state_age_sec": state_age,
            "object_age_sec": object_age,
            "object_error": self._object_error,
            "camera": {
                "frame_ready": self._camera_jpeg is not None,
                "source": self._camera_source,
                "error": self._camera_error,
            },
            "obstacle_avoidance": {
                "enabled": self._obstacle_avoidance,
                "pending": self._obstacle_pending,
                "error": self._obstacle_error,
            },
            "topics": {
                "camera": self._topic_online("/camera/image_raw"),
                "lidar": self._topic_online("/point_cloud2"),
                "imu": self._topic_online("/imu"),
                "odom": self._topic_online("/odom"),
                "objects": self._topic_online(self.objects_topic),
                "tts": self.tts_publisher.get_subscription_count() > 0,
            },
        }

    def _enqueue_action(self, action):
        self._commands.put(("action", action))

    def _enqueue_tts(self, text):
        self._commands.put(("tts", text))

    def _enqueue_obstacle_avoidance(self, enabled):
        self._commands.put(("obstacle_avoidance", enabled))

    def _drain_commands(self):
        while True:
            try:
                command_type, value = self._commands.get_nowait()
            except queue.Empty:
                return
            if command_type == "action":
                action = SPORT_ACTIONS[value]
                commands = [
                    {"api_id": action["api_id"], "parameter": action.get("parameter", {})},
                    *action.get("followups", []),
                ]
                for command in commands:
                    message = WebRtcReq()
                    message.id = 0
                    message.topic = "rt/api/sport/request"
                    message.api_id = command["api_id"]
                    message.parameter = json.dumps(command.get("parameter", {}))
                    message.priority = 1 if value == "stop" else 0
                    self.sport_publisher.publish(message)
                command_ids = ", ".join(str(command["api_id"]) for command in commands)
                self.get_logger().info(f"Dashboard action: {value} ({command_ids})")
            elif command_type == "tts":
                message = String()
                message.data = value
                self.tts_publisher.publish(message)
                self.get_logger().info("Dashboard published a TTS request")
            elif command_type == "obstacle_avoidance":
                self._set_obstacle_avoidance(value)

    @web.middleware
    async def _cors_middleware(self, request, handler):
        try:
            if request.method == "OPTIONS":
                response = web.Response(status=204)
            else:
                response = await handler(request)
        except web.HTTPException as error:
            # aiohttp turns HTTP exceptions into responses after middleware
            # unwinds. Catch them here so browser clients receive CORS headers
            # and can show the API's useful error instead of "Failed to fetch".
            response = error
        origin = request.headers.get("Origin")
        if origin in {"http://localhost:3000", "http://127.0.0.1:3000"}:
            response.headers["Access-Control-Allow-Origin"] = origin
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        return response

    async def _health_handler(self, _request):
        return web.json_response({"ok": True, "service": "go2_dashboard_bridge"})

    async def _status_handler(self, _request):
        return web.json_response(self._status())

    async def _objects_handler(self, _request):
        return web.json_response({"objects": self._objects, "error": self._object_error})

    async def _camera_handler(self, _request):
        with self._camera_lock:
            jpeg = self._camera_jpeg
        if jpeg is None:
            raise web.HTTPServiceUnavailable(
                text=json.dumps({"error": self._camera_error or "No camera frame received yet"}),
                content_type="application/json",
            )
        return web.Response(
            body=jpeg,
            content_type="image/jpeg",
            headers={"Cache-Control": "no-store, max-age=0"},
        )

    async def _read_json(self, request):
        try:
            return await request.json()
        except (json.JSONDecodeError, TypeError):
            raise web.HTTPBadRequest(text=json.dumps({"error": "Expected a JSON body"}), content_type="application/json")

    def _require_commands(self):
        if not self.allow_commands:
            raise web.HTTPForbidden(
                text=json.dumps({"error": "Live commands are disabled. Start the bridge with allow_commands:=true."}),
                content_type="application/json",
            )

    async def _action_handler(self, request):
        self._require_commands()
        body = await self._read_json(request)
        action = body.get("action")
        if action not in SPORT_ACTIONS:
            raise web.HTTPBadRequest(
                text=json.dumps({"error": "Unsupported action", "supported": sorted(SPORT_ACTIONS)}),
                content_type="application/json",
            )
        if self.sport_publisher.get_subscription_count() < 1:
            raise web.HTTPServiceUnavailable(
                text=json.dumps({"error": "The robot driver is not subscribed to /webrtc_req"}),
                content_type="application/json",
            )
        self._enqueue_action(action)
        return web.json_response({"queued": True, "action": action, **SPORT_ACTIONS[action]})

    async def _tts_handler(self, request):
        self._require_commands()
        body = await self._read_json(request)
        text = str(body.get("text", "")).strip()
        if not text or len(text) > 500:
            raise web.HTTPBadRequest(text=json.dumps({"error": "Text must contain 1 to 500 characters"}), content_type="application/json")
        if self.tts_publisher.get_subscription_count() < 1:
            raise web.HTTPServiceUnavailable(
                text=json.dumps({"error": "The TTS node is not running or configured"}),
                content_type="application/json",
            )
        self._enqueue_tts(text)
        return web.json_response({"queued": True, "text": text})

    async def _obstacle_avoidance_handler(self, request):
        self._require_commands()
        body = await self._read_json(request)
        enabled = body.get("enabled")
        if not isinstance(enabled, bool):
            raise web.HTTPBadRequest(
                text=json.dumps({"error": "enabled must be true or false"}),
                content_type="application/json",
            )
        if self._obstacle_pending:
            raise web.HTTPConflict(
                text=json.dumps({"error": "An obstacle avoidance update is already pending"}),
                content_type="application/json",
            )
        if not self.set_driver_parameters.service_is_ready():
            raise web.HTTPServiceUnavailable(
                text=json.dumps({"error": f"Parameter service for {self.driver_node} is unavailable"}),
                content_type="application/json",
            )
        self._obstacle_pending = True
        self._enqueue_obstacle_avoidance(enabled)
        return web.json_response({"queued": True, "enabled": enabled})

    async def _command_handler(self, request):
        self._require_commands()
        body = await self._read_json(request)
        parsed = parse_voice_command(body.get("text", ""))
        if parsed["type"] == "action":
            action = parsed["action"]
            if self.sport_publisher.get_subscription_count() < 1:
                raise web.HTTPServiceUnavailable(text=json.dumps({"error": "Robot driver unavailable"}), content_type="application/json")
            self._enqueue_action(action)
            return web.json_response({"queued": True, **parsed, **SPORT_ACTIONS[action]})
        if parsed["type"] == "tts":
            if self.tts_publisher.get_subscription_count() < 1:
                raise web.HTTPServiceUnavailable(text=json.dumps({"error": "TTS node unavailable"}), content_type="application/json")
            self._enqueue_tts(parsed["text"][:500])
            return web.json_response({"queued": True, **parsed})
        if parsed["type"] == "movement_disabled":
            raise web.HTTPConflict(text=json.dumps({"error": "Dashboard movement is not enabled yet"}), content_type="application/json")
        raise web.HTTPBadRequest(text=json.dumps({"error": "Command not recognized"}), content_type="application/json")

    def _run_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        app = web.Application(middlewares=[self._cors_middleware])
        app.add_routes([
            web.get("/health", self._health_handler),
            web.get("/api/status", self._status_handler),
            web.get("/api/objects", self._objects_handler),
            web.get("/api/camera.jpg", self._camera_handler),
            web.post("/api/action", self._action_handler),
            web.post("/api/obstacle-avoidance", self._obstacle_avoidance_handler),
            web.post("/api/tts", self._tts_handler),
            web.post("/api/command", self._command_handler),
            web.options("/{path:.*}", self._health_handler),
        ])
        self._runner = web.AppRunner(app)
        self._loop.run_until_complete(self._runner.setup())
        site = web.TCPSite(self._runner, self.host, self.port)
        self._loop.run_until_complete(site.start())
        self._loop.run_forever()

    def destroy_node(self):
        if self._loop and self._runner:
            asyncio.run_coroutine_threadsafe(self._runner.cleanup(), self._loop).result(timeout=3)
            self._loop.call_soon_threadsafe(self._loop.stop)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
