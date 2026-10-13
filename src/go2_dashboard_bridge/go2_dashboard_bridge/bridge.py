"""ROS 2 to HTTP bridge for the Oak Control dashboard."""

import asyncio
import json
import queue
import threading
import time

from aiohttp import web
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("sport_topic", "/webrtc_req")
        self.declare_parameter("tts_topic", "/tts")

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.allow_commands = bool(self.get_parameter("allow_commands").value)
        self.state_topic = self.get_parameter("state_topic").value
        self.objects_topic = self.get_parameter("objects_topic").value
        self.sport_topic = self.get_parameter("sport_topic").value
        self.tts_topic = self.get_parameter("tts_topic").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Go2State, self.state_topic, self._state_callback, sensor_qos)
        self.create_subscription(String, self.objects_topic, self._objects_callback, 10)
        self.sport_publisher = self.create_publisher(WebRtcReq, self.sport_topic, 10)
        self.tts_publisher = self.create_publisher(String, self.tts_topic, 10)

        self._last_state = 0.0
        self._last_objects = 0.0
        self._objects = []
        self._object_error = None
        self._commands = queue.SimpleQueue()
        self.create_timer(0.05, self._drain_commands)

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

    def _drain_commands(self):
        while True:
            try:
                command_type, value = self._commands.get_nowait()
            except queue.Empty:
                return
            if command_type == "action":
                action = SPORT_ACTIONS[value]
                message = WebRtcReq()
                message.id = 0
                message.topic = "rt/api/sport/request"
                message.api_id = action["api_id"]
                message.parameter = "{}"
                message.priority = 1 if value == "stop" else 0
                self.sport_publisher.publish(message)
                self.get_logger().info(f"Dashboard action: {value} ({action['api_id']})")
            elif command_type == "tts":
                message = String()
                message.data = value
                self.tts_publisher.publish(message)
                self.get_logger().info("Dashboard published a TTS request")

    @web.middleware
    async def _cors_middleware(self, request, handler):
        if request.method == "OPTIONS":
            response = web.Response(status=204)
        else:
            response = await handler(request)
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
            web.post("/api/action", self._action_handler),
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
