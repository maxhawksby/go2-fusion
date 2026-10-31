# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
import time
from typing import Callable, Dict, Any

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotData, RobotConfig
from .go2_connection import Go2Connection
from .connection_health import (
    DATA_STALL_TIMEOUT_SEC, GIVE_UP, NUDGE, StreamWatch, data_stalled,
)

# How often to log which streams are arriving and how fast.
STREAM_LOG_PERIOD_SEC = 30.0
from ...application.utils.command_generator import gen_command, gen_mov_command
from ...domain.constants import ROBOT_CMD, RTC_TOPIC

logger = logging.getLogger(__name__)
# The driver logs Python modules at WARN; the periodic stream summary is
# meant to be seen, so it gets its own INFO-level logger (as the node does).
stream_logger = logging.getLogger(__name__ + '.streams')
stream_logger.setLevel(logging.INFO)


class WebRTCAdapter(IRobotDataReceiver, IRobotController):
    """WebRTC adapter for robot communication"""

    def __init__(self, config: RobotConfig, on_validated_callback: Callable, on_video_frame_callback: Callable = None, event_loop=None):
        self.config = config
        self.connections: Dict[str, Go2Connection] = {}
        self.disconnected_events: Dict[str, asyncio.Event] = {}
        self.data_callback: Callable[[RobotData], None] = None
        self.webrtc_msgs = asyncio.Queue()
        self.on_validated_callback = on_validated_callback
        self.on_video_frame_callback = on_video_frame_callback
        # Per robot: lidar cloud and camera liveness (see connection_health),
        # and when stream rates were last logged.
        self.lidar_watch: Dict[str, StreamWatch] = {}
        self.video_watch: Dict[str, StreamWatch] = {}
        self.stream_log_at: Dict[str, float] = {}
        # Store the event loop (passed from main thread or detect current)
        if event_loop:
            self.main_loop = event_loop
        else:
            try:
                self.main_loop = asyncio.get_running_loop()
            except RuntimeError:
                self.main_loop = None

    async def connect(self, robot_id: str) -> None:
        """Connect to robot via WebRTC"""
        try:
            # A retry after a previous drop (main.py now retries instead of
            # dying) would otherwise build a brand-new Go2Connection over a
            # still-registered stale one, leaking its RTCPeerConnection and
            # HTTP session instead of releasing them.
            if robot_id in self.connections:
                await self.disconnect(robot_id)

            robot_idx = int(robot_id)
            robot_ip = self.config.robot_ip_list[robot_idx]

            disconnected_event = asyncio.Event()
            self.disconnected_events[robot_id] = disconnected_event

            conn = Go2Connection(
                robot_ip=robot_ip,
                robot_num=robot_id,
                token=self.config.token,
                aes_key=self.config.aes_key,
                on_validated=self._on_validated,
                on_message=self._on_data_channel_message,
                on_video_frame=self.on_video_frame_callback if self.config.enable_video else None,
                decode_lidar=self.config.decode_lidar,
                # Bound to this connection's own event: a late close from the
                # previous connection must not mark the new one as dropped.
                on_disconnected=lambda rid, ev=disconnected_event: ev.set(),
            )

            self.connections[robot_id] = conn
            await conn.connect()
            # Traffic saving and the lidar are switched in _on_validated:
            # here the data channel isn't open yet, and a request sent now
            # was silently dropped.

            logger.info(f"Connected to robot {robot_id} at {robot_ip}")

        except Exception as e:
            logger.error(f"Failed to connect to robot {robot_id}: {e}")
            raise

    def is_connected(self, robot_id: str) -> bool:
        """False once a mid-session drop has been detected for this robot
        (see Go2Connection.on_disconnected). Polled by the control loop so a
        drop that happens well after the initial connect still raises and
        reaches main.py's retry logic, instead of going unnoticed forever."""
        event = self.disconnected_events.get(robot_id)
        if event is None or event.is_set():
            return False
        # A data channel can also go quiet without closing (video still
        # arriving, odometry and lidar not). Count that as a drop too, so the
        # same retry logic reconnects instead of running blind forever.
        conn = self.connections.get(robot_id)
        if conn is not None and data_stalled(
            getattr(conn, 'last_message_mono', None), time.monotonic()
        ):
            logger.error(
                f"No data from robot {robot_id} for over "
                f"{DATA_STALL_TIMEOUT_SEC:.0f} s (odometry, lidar, state); "
                "treating the connection as lost"
            )
            event.set()
            return False
        if conn is not None and not self._check_streams(robot_id, conn):
            event.set()
            return False
        return True

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot"""
        if robot_id in self.connections:
            try:
                # Используем правильный метод для закрытия WebRTC соединения
                connection = self.connections[robot_id]
                if hasattr(connection, 'disconnect'):
                    await connection.disconnect()
                elif hasattr(connection, 'pc') and connection.pc:
                    await connection.pc.close()
                del self.connections[robot_id]
                logger.info(f"Disconnected from robot {robot_id}")
            except Exception as e:
                logger.error(f"Error disconnecting from robot {robot_id}: {e}")

    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Set callback for data reception"""
        self.data_callback = callback

    def send_command(self, robot_id: str, command: str) -> None:
        """Send command to robot"""
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'data_channel') and connection.data_channel:
                    # Use asyncio.run_coroutine_threadsafe to handle cross-thread calls
                    loop = self._get_or_create_event_loop()
                    if loop and loop.is_running():
                        # Schedule the coroutine in the existing loop
                        asyncio.run_coroutine_threadsafe(
                            self._async_send_command(connection, command),
                            loop
                        )
                    else:
                        # Fallback to synchronous send
                        connection.data_channel.send(command)
                    logger.debug(f"Command sent to robot {robot_id}: {command[:50]}")
                else:
                    logger.warning(f"No data channel available for robot {robot_id}")
            except Exception as e:
                logger.error(f"Error sending command to robot {robot_id}: {e}")

    def _get_or_create_event_loop(self):
        """Get existing event loop or return the main loop"""
        # First try to get the current loop
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            # If no current loop, return the main loop stored during init
            return self.main_loop

    async def _async_send_command(self, connection, command: str):
        """Async wrapper for sending commands"""
        try:
            if hasattr(connection, 'data_channel') and connection.data_channel:
                connection.data_channel.send(command)
        except Exception as e:
            logger.error(f"Error in async send command: {e}")

    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        try:
            command = gen_mov_command(
                round(x, 2), 
                round(y, 2), 
                round(z, 2), 
                self.config.obstacle_avoidance
            )
            self.send_command(robot_id, command)
        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        try:
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.send_command(robot_id, stand_up_cmd)
            
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.send_command(robot_id, move_cmd)
        except Exception as e:
            logger.error(f"Error sending stand up command: {e}")

    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        try:
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.send_command(robot_id, stand_down_cmd)
        except Exception as e:
            logger.error(f"Error sending stand down command: {e}")

    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: Any, topic: str) -> None:
        """Send WebRTC request"""
        try:
            payload = gen_command(api_id, parameter, topic)
            self.webrtc_msgs.put_nowait(payload)
            logger.debug(f"WebRTC request queued for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error sending WebRTC request: {e}")

    def process_webrtc_commands(self, robot_id: str) -> None:
        """Process WebRTC commands from queue"""
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.send_command(robot_id, message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def _on_validated(self, robot_id: str) -> None:
        """Callback after connection validation"""
        try:
            if robot_id in self.connections:
                conn = self.connections[robot_id]
                for topic in RTC_TOPIC.values():
                    conn.data_channel.send(
                        json.dumps({"type": "subscribe", "topic": topic}))
                self._enable_lidar(conn)
                now = time.monotonic()
                self.lidar_watch[robot_id] = StreamWatch()
                self.lidar_watch[robot_id].start(now)
                if self.config.enable_video:
                    self.video_watch[robot_id] = StreamWatch()
                    self.video_watch[robot_id].start(now)
                self.stream_log_at[robot_id] = now
            
            if self.on_validated_callback:
                self.on_validated_callback(robot_id)
                
        except Exception as e:
            logger.error(f"Error in validated callback: {e}")

    def _enable_lidar(self, conn) -> None:
        """Ask the robot for the lidar cloud: traffic saving off (the code's
        own note says the lidar needs it) and the lidar switched on. Neither
        moves the robot."""
        conn.publish("", {"req_type": "disable_traffic_saving",
                          "instruction": "on"}, "rtc_inner_req")
        conn.publish(RTC_TOPIC["ULIDAR_SWITCH"], "on")

    def note_video_frame(self, robot_id: str) -> None:
        """Called by the driver node for each decoded camera frame."""
        watch = self.video_watch.get(robot_id)
        if watch is not None:
            watch.seen(time.monotonic())

    def _check_streams(self, robot_id: str, conn) -> bool:
        """Nudge a quiet lidar or camera stream; False when the lidar stays
        quiet after a nudge, so the caller reconnects."""
        now = time.monotonic()
        lidar = self.lidar_watch.get(robot_id)
        if lidar is not None:
            action = lidar.check(now)
            if action == NUDGE:
                logger.warning(
                    f"No lidar from robot {robot_id} for {lidar.stall_sec:.0f} s "
                    "while other data arrives; asking for it again"
                )
                self._enable_lidar(conn)
            elif action == GIVE_UP:
                logger.error(
                    f"Lidar from robot {robot_id} still missing after asking "
                    "again; treating the connection as lost"
                )
                return False

        video = self.video_watch.get(robot_id)
        if video is not None and video.check(now) in (NUDGE, GIVE_UP):
            # A new stream starts with a key frame; without one the decoder
            # can't show anything ("non-existing PPS"). Camera trouble alone
            # never forces a reconnect.
            logger.warning(
                f"No camera frames from robot {robot_id} for "
                f"{video.stall_sec:.0f} s; asking for the video again"
            )
            conn.publish("", "on", "vid")

        if now - self.stream_log_at.get(robot_id, now) >= STREAM_LOG_PERIOD_SEC:
            period = now - self.stream_log_at[robot_id]
            rates = [
                f"{name} {watch.take_count() / period:.1f}/s"
                for name, watch in (("lidar", lidar), ("camera", video))
                if watch is not None
            ]
            stream_logger.info(f"Robot {robot_id} streams: {', '.join(rates)}")
            self.stream_log_at[robot_id] = now
        return True

    def _on_data_channel_message(self, _, msg: Dict[str, Any], robot_id: str) -> None:
        """Handle incoming data channel messages"""
        try:
            if isinstance(msg, dict) and msg.get("topic") == RTC_TOPIC["ULIDAR_ARRAY"]:
                watch = self.lidar_watch.get(robot_id)
                if watch is not None:
                    watch.seen(time.monotonic())
            if self.data_callback:
                # Создаем объект RobotData для передачи в callback
                # Фактическая обработка будет в RobotDataService
                robot_data = RobotData(robot_id=robot_id, timestamp=0.0)
                self.data_callback(msg, robot_id)  # Передаем сырые данные для обработки
                
        except Exception as e:
            logger.error(f"Error processing data channel message: {e}") 