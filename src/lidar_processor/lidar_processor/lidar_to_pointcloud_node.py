# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
LiDAR to PointCloud Node

Enhanced LiDAR data processing node with better architecture and performance.
"""

import os
from typing import List, Tuple
from dataclasses import dataclass
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException
import open3d as o3d

from lidar_processor.frame_transform import transform_points
from lidar_processor.point_map import CellPointMap


@dataclass
class LidarConfig:
    """Configuration for LiDAR processing"""
    robot_ip_list: List[str]
    map_name: str
    save_map: bool
    save_interval: float = 10.0  # seconds
    max_points: int = 1000000    # Maximum points to keep in memory
    voxel_size: float = 0.01     # Voxel downsampling size
    publish_rate: float = 5.0    # Hz, independent of incoming scan rate
    # Size of the 2D cells a fresh cloud refreshes (see point_map.py).
    refresh_cell_size: float = 0.25


class PointCloudAggregator:
    """Thread-safe accumulated cloud that follows the environment.

    Each incoming cloud replaces what the map held where it can see, and
    leaves the rest alone (see point_map.CellPointMap). Over max_points, the
    areas seen longest ago are dropped first.
    """

    def __init__(self, config: LidarConfig):
        self.config = config
        self.points = CellPointMap(config.refresh_cell_size, config.max_points)
        self._lock = Lock()
        self._points_changed = False

    def add_points(self, new_points: List[Tuple[float, float, float]]) -> int:
        """Merge a fresh cloud in; returns how many stale points it removed."""
        with self._lock:
            dropped = self.points.update(new_points)
            self._points_changed = True
            return dropped

    def get_points_copy(self):
        """Copy of all points, as an (N, 3) numpy array"""
        with self._lock:
            return self.points.points()

    def has_changes(self) -> bool:
        """Check if points have changed since last check"""
        with self._lock:
            return self._points_changed

    def mark_saved(self) -> None:
        """Mark current state as saved"""
        with self._lock:
            self._points_changed = False

    def get_point_count(self) -> int:
        """Get current point count"""
        with self._lock:
            return len(self.points)

    def clear(self) -> int:
        """Drop every point; returns how many there were"""
        with self._lock:
            count = self.points.clear()
            self._points_changed = True
            return count


class LidarToPointCloudNode(Node):
    """Enhanced LiDAR to PointCloud processing node"""
    
    def __init__(self):
        super().__init__('lidar_to_pointcloud')
        
        # Declare and get parameters
        self._declare_parameters()
        self.config = self._load_configuration()
        
        # Initialize components
        self.aggregator = PointCloudAggregator(self.config)
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Setup QoS profile for high-frequency data
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setup subscriptions and publishers
        self._last_header = None
        self._setup_subscriptions()
        self._setup_publishers()

        # Setup timers
        if self.config.save_map:
            self.save_timer = self.create_timer(
                self.config.save_interval,
                self._save_map_callback
            )
        # Publishing on its own timer, independent of the incoming scan
        # rate, so the full aggregated cloud (up to max_points) isn't
        # copied/serialized/published once per incoming scan - that cost
        # grows as the map fills toward max_points during a long mapping
        # session.
        self.publish_timer = self.create_timer(
            1.0 / self.config.publish_rate,
            self._publish_timer_callback
        )

        # /lidar_to_pointcloud/reset: start the 3D map over (the RViz
        # AutoMapper panel's "Reset 3D map" button, or `ros2 service call`).
        self.reset_service = self.create_service(
            Trigger, '~/reset', self._reset_callback
        )

        # Log configuration
        self._log_configuration()
    
    def _declare_parameters(self) -> None:
        """Declare all node parameters"""
        # An empty-list default leaves rclpy unable to infer the array
        # element type, so a later attempt to actually set this parameter
        # to a real string list (e.g. from the launch file) was rejected
        # outright as a type mismatch ("expecting type 'BYTE_ARRAY'") -
        # crashing this node on every single launch. An explicit
        # STRING_ARRAY descriptor removes the ambiguity.
        self.declare_parameter(
            'robot_ip_lst',
            value=None,
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter('map_name', '3d_map')
        self.declare_parameter('map_save', 'true')
        self.declare_parameter('save_interval', 10.0)
        self.declare_parameter('max_points', 1000000)
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('publish_rate', 5.0)
        # Each fresh cloud replaces the map in the 2D cells of this size it
        # covers, so moved or removed objects drop out (see point_map.py).
        self.declare_parameter('refresh_cell_size', 0.25)
        # Saved .ply coordinates are in this frame. The live cloud stays in
        # odom; each save applies the latest transform, so a loop closure
        # moves the file with the map.
        self.declare_parameter('map_frame', 'map')
    
    def _load_configuration(self) -> LidarConfig:
        """Load configuration from parameters"""
        robot_ip_lst = self.get_parameter('robot_ip_lst').get_parameter_value().string_array_value
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        save_map_str = self.get_parameter('map_save').get_parameter_value().string_value
        save_interval = self.get_parameter('save_interval').get_parameter_value().double_value
        max_points = self.get_parameter('max_points').get_parameter_value().integer_value
        voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        refresh_cell_size = self.get_parameter(
            'refresh_cell_size').get_parameter_value().double_value

        return LidarConfig(
            robot_ip_list=robot_ip_lst,
            map_name=map_name,
            save_map=save_map_str.lower() == 'true',
            save_interval=save_interval,
            max_points=max_points,
            voxel_size=voxel_size,
            publish_rate=publish_rate,
            refresh_cell_size=refresh_cell_size,
        )

    def _setup_subscriptions(self) -> None:
        """Setup LiDAR data subscriptions"""
        # Not self.subscriptions: rclpy.node.Node already exposes a
        # read-only `subscriptions` property backed by its own internal
        # list (used by destroy_node() for cleanup), so assigning a plain
        # list to that name collides with it.
        self._lidar_subscriptions = []

        if not self.config.robot_ip_list:
            self.get_logger().warn(
                "robot_ip_lst is empty; not subscribing to any lidar topic."
            )
            return

        if len(self.config.robot_ip_list) == 1:
            # Single robot mode
            subscription = self.create_subscription(
                PointCloud2,
                '/robot0/point_cloud2',
                self._lidar_callback,
                self.qos_profile
            )
            self._lidar_subscriptions.append(subscription)
        else:
            # Multi-robot mode
            for i, _ in enumerate(self.config.robot_ip_list):
                subscription = self.create_subscription(
                    PointCloud2,
                    f'/robot{i}/point_cloud2',
                    self._lidar_callback,
                    self.qos_profile
                )
                self._lidar_subscriptions.append(subscription)
    
    def _setup_publishers(self) -> None:
        """Setup publishers for processed data"""
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 
            '/pointcloud/aggregated', 
            self.qos_profile
        )
    
    def _lidar_callback(self, msg: PointCloud2) -> None:
        """Process incoming LiDAR data"""
        try:
            # Extract points from message
            points = []
            for point in point_cloud2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            ):
                points.append(tuple(point))
            
            # Add to aggregator
            self.aggregator.add_points(points)
            self._last_header = msg.header

        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR data: {e}")

    def _publish_timer_callback(self) -> None:
        """Publish the aggregated cloud at a fixed rate, decoupled from the
        incoming scan rate."""
        if self._last_header is not None:
            self._publish_aggregated_pointcloud(self._last_header)

    def _publish_aggregated_pointcloud(self, header) -> None:
        """Publish aggregated point cloud"""
        try:
            points = self.aggregator.get_points_copy()
            if len(points):
                pointcloud_msg = point_cloud2.create_cloud_xyz32(header, points)
                self.pointcloud_pub.publish(pointcloud_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing point cloud: {e}")
    
    def _reset_callback(self, request, response):
        """Clear the aggregated 3D map. The saved .ply keeps the old map
        until the next save after new points arrive (within save_interval),
        since writing an empty cloud would fail."""
        dropped = self.aggregator.clear()
        self.get_logger().info(f"🧹 3D map reset ({dropped} points dropped)")
        response.success = True
        response.message = f"3D map cleared ({dropped} points)."
        return response

    def _save_map_callback(self) -> None:
        """Periodic map saving callback"""
        try:
            if not self.aggregator.has_changes():
                return
            
            points = self.aggregator.get_points_copy()
            if not len(points):
                return

            source_frame = 'odom'
            if self._last_header is not None and self._last_header.frame_id:
                source_frame = self._last_header.frame_id
            points = self._points_in_map(points, source_frame)
            if points is None:
                return

            # Create Open3D point cloud
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(points)
            
            # Apply voxel downsampling for saving
            if self.config.voxel_size > 0:
                point_cloud = point_cloud.voxel_down_sample(self.config.voxel_size)
            
            # Save to file. Resolved to an absolute path so where it lands
            # doesn't depend on the process's current working directory,
            # and the directory is created if it doesn't exist yet.
            map_filename = os.path.abspath(f"{self.config.map_name}.ply")
            os.makedirs(os.path.dirname(map_filename), exist_ok=True)
            if o3d.io.write_point_cloud(map_filename, point_cloud):
                point_count = len(point_cloud.points)
                total_points = self.aggregator.get_point_count()
                self.aggregator.mark_saved()
                
                self.get_logger().info(
                    f"💾 Saved map: {map_filename} "
                    f"({point_count} downsampled / {total_points} total points, "
                    f"{self.map_frame} frame)"
                )
            else:
                self.get_logger().error(f"Failed to save map: {map_filename}")
                
        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")

    def _points_in_map(self, points, source_frame):
        """Copy of `points` in map_frame, or None if that transform is not up.

        Stored points stay in the cloud frame. The file is rewritten from
        them with the transform as it is now, so a loop closure shifts the
        whole .ply. Moved obstacles drop out as fresh clouds replace the
        area they cover (see point_map.py).
        """

        if source_frame == self.map_frame:
            return points
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, rclpy.time.Time()
            )
        except TransformException:
            self.get_logger().warn(
                f'No {self.map_frame} <- {source_frame} yet; '
                'not writing the 3D map in the cloud frame.',
                throttle_duration_sec=5.0,
            )
            return None
        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        return transform_points(
            points,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _log_configuration(self) -> None:
        """Log current configuration"""
        self.get_logger().info("🗺️  LiDAR Processor Configuration:")
        self.get_logger().info(f"   Robot IPs: {self.config.robot_ip_list}")
        self.get_logger().info(f"   Map name: {self.config.map_name}")
        self.get_logger().info(f"   Map frame: {self.map_frame}")
        self.get_logger().info(f"   Save map: {self.config.save_map}")
        self.get_logger().info(f"   Publish rate: {self.config.publish_rate} Hz")
        if self.config.save_map:
            self.get_logger().info(f"   Save interval: {self.config.save_interval}s")
            self.get_logger().info(f"   Max points: {self.config.max_points:,}")
            self.get_logger().info(f"   Voxel size: {self.config.voxel_size}m")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = LidarToPointCloudNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running lidar processor: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 