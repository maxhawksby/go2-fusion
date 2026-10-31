# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import logging
import os
from typing import Dict, Any

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from go2_interfaces.msg import LowState, VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import PointCloud2, JointState, Joy, Image, CameraInfo
from nav_msgs.msg import Odometry

from ..domain.entities import RobotConfig, RobotData, CameraData
from ..application.services import RobotDataService, RobotControlService
from ..infrastructure.ros2 import ROS2Publisher
from ..infrastructure.webrtc import WebRTCAdapter

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class Go2DriverNode(Node):
    """Main Go2 driver node - entry point to the application"""

    def __init__(self, event_loop=None):
        super().__init__('go2_driver_node')  # Clean architecture main driver
        self.event_loop = event_loop
        
        # Configuration initialization
        self.config = self._setup_configuration()
        
        # Infrastructure initialization
        self.publishers_dict = self._setup_publishers()
        self.broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))
        self.bridge = CvBridge()
        
        # Architecture layers initialization
        self.ros2_publisher = ROS2Publisher(
            node=self,
            config=self.config,
            publishers=self.publishers_dict,
            broadcaster=self.broadcaster
        )
        
        self.robot_data_service = RobotDataService(self.ros2_publisher)
        
        self.webrtc_adapter = WebRTCAdapter(
            config=self.config,
            on_validated_callback=self._on_robot_validated,
            on_video_frame_callback=self._on_video_frame if self.config.enable_video else None,
            event_loop=self.event_loop
        )
        
        self.robot_control_service = RobotControlService(self.webrtc_adapter)
        
        # Set callback for data
        self.webrtc_adapter.set_data_callback(self._on_robot_data_received)
        
        # Subscribers initialization
        self._setup_subscribers()
        
        # State
        self.joy_state = Joy()

    def _setup_configuration(self) -> RobotConfig:
        """Configuration setup"""
        robot_ip = os.getenv('ROBOT_IP', os.getenv('GO2_IP', ''))
        token = os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN', ''))
        aes_key = os.getenv('ROBOT_AES_KEY', os.getenv('GO2_AES_KEY', ''))
        conn_type = os.getenv('CONN_TYPE', '')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', robot_ip),
                ('token', token),
                ('aes_key', aes_key),
                ('conn_type', conn_type),
                ('enable_video', True),
                ('decode_lidar', True),
                ('publish_raw_voxel', False),
                ('obstacle_avoidance', False),
            ]
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Get parameter values
        config = RobotConfig.from_params(
            robot_ip=self.get_parameter('robot_ip').get_parameter_value().string_value,
            token=self.get_parameter('token').get_parameter_value().string_value,
            aes_key=self.get_parameter('aes_key').get_parameter_value().string_value,
            conn_type=self.get_parameter('conn_type').get_parameter_value().string_value,
            enable_video=self.get_parameter('enable_video').get_parameter_value().bool_value,
            decode_lidar=self.get_parameter('decode_lidar').get_parameter_value().bool_value,
            publish_raw_voxel=self.get_parameter('publish_raw_voxel').get_parameter_value().bool_value,
            obstacle_avoidance=self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value
        )

        # Log configuration
        self.get_logger().info(f"Robot IPs: {config.robot_ip_list}")
        self.get_logger().info(f"Connection type: {config.conn_type}")
        self.get_logger().info(f"Connection mode: {config.conn_mode}")
        self.get_logger().info(f"Enable video: {config.enable_video}")
        self.get_logger().info(f"Decode lidar: {config.decode_lidar}")
        self.get_logger().info(f"Publish raw voxel: {config.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance: {config.obstacle_avoidance}")

        return config

    def _setup_publishers(self) -> Dict[str, list]:
        """ROS2 publishers setup"""
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        publishers = {
            'joint_state': [],
            'robot_state': [],
            'lidar': [],
            'odometry': [],
            'imu': [],
            'camera': [],
            'camera_info': [],
            'voxel': []
        }

        num_robots = len(self.config.robot_ip_list)
        
        for i in range(num_robots):
            # Define topics depending on connection mode
            if self.config.conn_mode == 'single':
                joint_topic = 'joint_states'
                robot_state_topic = 'go2_states'
                lidar_topic = 'point_cloud2'
                odom_topic = 'odom'
                imu_topic = 'imu'
                camera_topic = 'camera/image_raw'
                camera_info_topic = 'camera/camera_info'
                voxel_topic = '/utlidar/voxel_map_compressed'
            else:
                prefix = f'robot{i}'
                joint_topic = f'{prefix}/joint_states'
                robot_state_topic = f'{prefix}/go2_states'
                lidar_topic = f'{prefix}/point_cloud2'
                odom_topic = f'{prefix}/odom'
                imu_topic = f'{prefix}/imu'
                camera_topic = f'{prefix}/camera/image_raw'
                camera_info_topic = f'{prefix}/camera/camera_info'
                voxel_topic = f'{prefix}/utlidar/voxel_map_compressed'

            # Create publishers
            publishers['joint_state'].append(
                self.create_publisher(JointState, joint_topic, qos_profile))
            publishers['robot_state'].append(
                self.create_publisher(Go2State, robot_state_topic, qos_profile))
            publishers['lidar'].append(
                self.create_publisher(
                    PointCloud2, lidar_topic, best_effort_qos,
                    qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            publishers['odometry'].append(
                self.create_publisher(Odometry, odom_topic, qos_profile))
            publishers['imu'].append(
                self.create_publisher(IMU, imu_topic, qos_profile))

            if self.config.enable_video:
                publishers['camera'].append(
                    self.create_publisher(
                        Image, camera_topic, best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                publishers['camera_info'].append(
                    self.create_publisher(
                        CameraInfo, camera_info_topic, best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))

            if self.config.publish_raw_voxel:
                publishers['voxel'].append(
                    self.create_publisher(VoxelMapCompressed, voxel_topic, best_effort_qos))

        return publishers

    def _setup_subscribers(self) -> None:
        """ROS2 subscribers setup"""
        qos_profile = QoSProfile(depth=10)

        # Command subscribers
        num_robots = len(self.config.robot_ip_list)
        
        if self.config.conn_mode == 'single':
            self.create_subscription(
                Twist, 'cmd_vel_out',
                lambda msg: self._on_cmd_vel(msg, "0"), qos_profile)
            self.create_subscription(
                WebRtcReq, 'webrtc_req',
                lambda msg: self._on_webrtc_req(msg, "0"), qos_profile)
        else:
            for i in range(num_robots):
                self.create_subscription(
                    Twist, f'robot{i}/cmd_vel_out',
                    lambda msg, robot_id=str(i): self._on_cmd_vel(msg, robot_id), qos_profile)
                self.create_subscription(
                    WebRtcReq, f'robot{i}/webrtc_req',
                    lambda msg, robot_id=str(i): self._on_webrtc_req(msg, robot_id), qos_profile)

        # Joystick subscriber
        self.create_subscription(Joy, 'joy', self._on_joy, qos_profile)

        # CycloneDDS support
        if self.config.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState, 'lowstate',
                self._on_cyclonedds_low_state, qos_profile)
            self.create_subscription(
                PoseStamped, '/utlidar/robot_pose',
                self._on_cyclonedds_pose, qos_profile)
            self.create_subscription(
                PointCloud2, '/utlidar/cloud',
                self._on_cyclonedds_lidar, qos_profile)

    # The config-backed parameters declared in _setup_configuration() that
    # are only ever read once, at construction time. Deliberately NOT a
    # catch-all for every unrecognized name: this callback also fires for
    # parameters rclpy declares on this node automatically (e.g. per-topic
    # `qos_overrides.*` parameters created inside create_publisher()), and
    # rejecting those broke node construction outright.
    _CONSTRUCTION_TIME_ONLY_PARAMS = frozenset({
        'robot_ip', 'token', 'aes_key', 'conn_type',
        'enable_video', 'decode_lidar', 'publish_raw_voxel',
    })

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Callback for parameter changes"""
        result = SetParametersResult(successful=True)

        try:
            for p in params:
                if p.name == 'obstacle_avoidance':
                    self.get_logger().info(f'New obstacle_avoidance value: {p.value}')
                    self.config.obstacle_avoidance = p.value

                    try:
                        self.robot_control_service.set_obstacle_avoidance(p.value, "0")
                    except Exception as e:
                        self.get_logger().error(f"Failed to set obstacle avoidance: {e}")
                        result.successful = False
                        result.reason = str(e)
                        return result

                    result.successful = True
                    result.reason = 'Updated obstacle_avoidance'
                elif p.name in self._CONSTRUCTION_TIME_ONLY_PARAMS:
                    # Reporting success here without actually reconnecting/
                    # re-setting-up would let self.config and the live
                    # connection silently disagree - e.g.
                    # `ros2 param set conn_type cyclonedds` would report
                    # success while nothing actually changed.
                    result.successful = False
                    result.reason = (
                        f"'{p.name}' cannot be changed at runtime; "
                        f"restart the node instead"
                    )
                    return result
                # Any other parameter name (including rclpy/ROS2-internal
                # ones this node didn't declare itself) is left alone.
        except Exception as e:
            self.get_logger().error(f"Error setting parameters: {e}")
            result.successful = False
            result.reason = str(e)

        return result

    def _on_cmd_vel(self, msg: Twist, robot_id: str) -> None:
        """Callback for movement commands"""
        self.robot_control_service.handle_cmd_vel(
            msg.linear.x, msg.linear.y, msg.angular.z, robot_id
        )

    def _on_webrtc_req(self, msg: WebRtcReq, robot_id: str) -> None:
        """Callback for WebRTC requests"""
        self.robot_control_service.handle_webrtc_request(
            msg.api_id, msg.parameter, msg.topic, msg.id, robot_id
        )

    def _on_joy(self, msg: Joy) -> None:
        """Callback for joystick"""
        self.joy_state = msg

    def _on_robot_validated(self, robot_id: str) -> None:
        """Callback after robot validation"""
        self.get_logger().info(f"Robot {robot_id} validated and ready")

    def _on_robot_data_received(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Callback for receiving data from robot"""
        self.robot_data_service.process_webrtc_message(msg, robot_id)

    async def _on_video_frame(self, track: MediaStreamTrack, robot_id: str) -> None:
        """Callback for processing video frames"""
        logger.info(f"Video frame received for robot {robot_id}")

        # A single bad/corrupted frame (expected occasionally on a flaky
        # wifi link) used to permanently end camera publishing for the rest
        # of the node's life. Retry transient errors; only give up after
        # enough consecutive failures to indicate the track itself is dead.
        consecutive_errors = 0
        max_consecutive_errors = 10

        while True:
            try:
                frame = await track.recv()
                consecutive_errors = 0
                img = frame.to_ndarray(format="bgr24")

                # Create camera data
                camera_data = CameraData(
                    image=img,
                    height=img.shape[0],
                    width=img.shape[1],
                    encoding="bgr8"
                )

                robot_data = RobotData(
                    robot_id=robot_id,
                    timestamp=0.0,
                    camera_data=camera_data
                )

                # Publish via ROS2Publisher
                self.ros2_publisher.publish_camera_data(robot_data)
                self.webrtc_adapter.note_video_frame(robot_id)
                await asyncio.sleep(0)

            except Exception as e:
                consecutive_errors += 1
                logger.error(
                    f"Error processing video frame for robot {robot_id} "
                    f"({consecutive_errors}/{max_consecutive_errors}): {e}"
                )
                if consecutive_errors >= max_consecutive_errors:
                    logger.error(
                        f"Too many consecutive video errors for robot "
                        f"{robot_id}, giving up on this track."
                    )
                    break
                await asyncio.sleep(0.1)

    # CycloneDDS callbacks
    def _on_cyclonedds_low_state(self, msg: LowState) -> None:
        """Processing LowState for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_pose(self, msg: PoseStamped) -> None:
        """Processing pose for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_lidar(self, msg: PointCloud2) -> None:
        """Processing lidar for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    async def connect_robots(self) -> None:
        """Connect to robots"""
        if self.config.conn_type == 'webrtc':
            for i, robot_ip in enumerate(self.config.robot_ip_list):
                try:
                    await self.webrtc_adapter.connect(str(i))
                except Exception as e:
                    self.get_logger().error(f"Failed to connect to robot {i}: {e}")
                    raise

    async def run_robot_control_loop(self, robot_id: str) -> None:
        """Main robot control loop"""
        while True:
            try:
                # A mid-session WebRTC drop (ICE failure, robot reboot, wifi
                # blip) used to go completely unnoticed here: this loop just
                # kept calling process_webrtc_commands() and sleeping, with
                # nothing checking whether the connection was actually still
                # alive, so the node looked healthy while silently receiving
                # nothing. Raising here instead lets main.py's retry/backoff
                # logic actually engage for a drop, not just a failed
                # initial connect.
                if (
                    self.config.conn_type == 'webrtc'
                    and not self.webrtc_adapter.is_connected(robot_id)
                ):
                    raise ConnectionError(
                        f"WebRTC connection to robot {robot_id} was lost"
                    )

                # Process joystick commands
                if self.joy_state.buttons:
                    self.robot_control_service.handle_joy_command(
                        self.joy_state.buttons, robot_id
                    )

                # Process WebRTC commands
                self.webrtc_adapter.process_webrtc_commands(robot_id)
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error in control loop for robot {robot_id}: {e}")
                raise 