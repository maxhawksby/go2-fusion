#!/usr/bin/env python3

import ctypes
import math
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.signals import SignalHandlerOptions

from action_msgs.srv import CancelGoal
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String
from go2_interfaces.msg import AutoMapperEvent
from go2_auto_mapper.recovery import (
    RECOVERY_SPEED_MPS,
    RECOVERY_TIME_LIMIT_SEC,
    REFRESH_TIMEOUT_SEC,
    ROBOT_FRONT_M,
    SCAN_MAX_AGE_SEC,
    SENSING_MAX_AGE_SEC,
    SLAM_MIN_TRAVEL_M,
    assess_recovery_steps,
    choose_turn,
    drift_limits,
    latch_decision,
    recovery_distances,
    refresh_status,
    scan_age,
    scan_covers_move,
    stale_sources,
    swept_clear,
    translation_open,
)

from builtin_interfaces.msg import Duration
from nav2_msgs.action import BackUp, NavigateToPose, Spin

import tf2_ros
from tf2_ros import TransformException

from go2_auto_mapper.obstacle_sensing import (
    angle_difference,
    free_distance_ahead,
    straight_line_clear,
)
from go2_auto_mapper.open_space import OCCUPIED_THRESHOLD, find_open_space
from go2_auto_mapper.frontier_scoring import (
    count_revealed_unknown,
    find_frontier_clusters,
    overlaps_dead,
    remaining_size,
    visit_utility,
    world_key,
)
from go2_auto_mapper.social_space import line_crosses, parse_people, social_zones, zone_at
from go2_auto_mapper.nav_log import NavLog, known_cell_count
from go2_auto_mapper.vlm_ranking import (
    ViewMemory,
    VlmClient,
    frontier_fact,
    match_score,
    suggestion_bonus,
    usable_heading,
    vlm_bonus,
    vlm_mode_name,
    vlm_shortlist,
)


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # -----------------------------
        # Parameters
        # -----------------------------

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('goal_distance', 0.8)
        # Frontiers closer than this are only used once no farther one is
        # left, so each leg covers new ground instead of shuffling in place.
        self.declare_parameter('min_goal_distance', 1.5)
        # Goals closer than this to a wall in the SLAM map are skipped:
        # Nav2 can't place the robot's body there, and it scrapes walls.
        self.declare_parameter('wall_clearance', 0.5)

        # Ranking goals by what a visit would reveal (frontier_scoring.py):
        # unknown cells visible from the goal within info_gain_range, by
        # casting info_gain_rays rays, minus info_gain_cells_per_meter for
        # every meter of travel. 400 cells is 1 square meter at 5 cm.
        self.declare_parameter('info_gain_range', 4.0)
        self.declare_parameter('info_gain_rays', 48)
        self.declare_parameter('info_gain_cells_per_meter', 100.0)

        # A vision-language model on a GPU server re-ranks the best goals
        # from the camera's view toward each (vlm_ranking.py). 'off': not
        # used. 'shadow': asked (waiting up to vlm_timeout for the answer),
        # and its pick logged next to ours, but ours is used. 'active': the
        # same wait, then its score is added to ours, worth vlm_weight
        # cells from a sure no to a sure yes. vlm_url is an OpenAI-compatible
        # chat endpoint (vLLM, Ollama); VLM_API_KEY in the environment is
        # sent if the server needs one.
        # VLM_MODE, VLM_URL and VLM_MODEL set the defaults: the RViz panel's Start
        # button runs the explorer with no parameters.
        # dynamic_typing: `-p vlm_mode:=off` arrives as the boolean False
        # (YAML), which a string parameter refuses at startup.
        self.declare_parameter(
            'vlm_mode', os.environ.get('VLM_MODE', 'off'),
            ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter(
            'vlm_url', os.environ.get('VLM_URL', 'http://localhost:8000/v1/chat/completions'))
        self.declare_parameter(
            'vlm_model', os.environ.get('VLM_MODEL', 'Qwen/Qwen2.5-VL-7B-Instruct'))
        self.declare_parameter('vlm_weight', 400.0)
        self.declare_parameter('vlm_timeout', 8.0)
        self.declare_parameter('vlm_candidates', 4)
        # Also ask about the best spot on the longest frontier when the best
        # vlm_candidates goals miss it; the reveal score alone ranks long
        # frontiers a little farther away below short near ones.
        self.declare_parameter('vlm_include_longest', True)
        # In active mode, unknown cells a goal is worth extra for lying the way
        # vlm_view last suggested. Needs the panel's VLM view on.
        self.declare_parameter('vlm_suggest_weight', 300.0)
        # Folder for the navigation log (nav_log.py); empty is off. NAV_LOG_DIR
        # sets the default, since the RViz panel's Start passes no parameters.
        self.declare_parameter('nav_log_dir', os.environ.get('NAV_LOG_DIR', ''))

        # A full turn in place gives SLAM a 360 degree lidar view, so it can
        # place the robot on the map before (and after) each leg of exploring.
        self.declare_parameter('scan_on_start', True)
        self.declare_parameter('scan_after_goal', True)
        self.declare_parameter('scan_yaw', 2.0 * math.pi)
        self.declare_parameter('scan_time_allowance', 30.0)

        # Watching Nav2's local costmap: whenever something is within
        # obstacle_stop_distance of the nose (for blocked_timeout seconds;
        # 0 = at once), back away. While driving, the goal is given up first
        # and the robot then heads somewhere open.
        self.declare_parameter('obstacle_stop_distance', 0.3)
        self.declare_parameter('blocked_timeout', 0.0)
        self.declare_parameter('backup_distance', 0.3)

        # Getting unstuck: after stuck_failure_count failed goals in a row
        # (or an obstacle-ahead abort, or no usable goal), drive to the
        # nearest spot with open_space_clearance meters of room on every
        # side, scan there, then carry on exploring.
        self.declare_parameter('open_space_clearance', 0.8)
        self.declare_parameter('stuck_failure_count', 3)

        # People from object_detection's /people (ideas from SocialNav):
        # don't pick goals in someone's personal space, across where they
        # are walking or inside a group, and stop briefly to let a person
        # ahead pass. Without /people nothing changes.
        self.declare_parameter('avoid_people', True)
        self.declare_parameter('yield_to_people', True)
        self.declare_parameter('personal_space', 1.0)

        self.map_topic = self.get_parameter(
            'map_topic'
        ).value

        self.robot_frame = self.get_parameter(
            'robot_frame'
        ).value

        self.map_frame = self.get_parameter(
            'map_frame'
        ).value

        self.min_frontier_size = self.get_parameter(
            'min_frontier_size'
        ).value

        self.goal_distance = self.get_parameter(
            'goal_distance'
        ).value

        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.wall_clearance = self.get_parameter('wall_clearance').value
        self.info_gain_range = self.get_parameter('info_gain_range').value
        self.info_gain_rays = self.get_parameter('info_gain_rays').value
        self.info_gain_cells_per_meter = self.get_parameter(
            'info_gain_cells_per_meter'
        ).value

        self.scan_after_goal = self.get_parameter('scan_after_goal').value
        self.scan_yaw = self.get_parameter('scan_yaw').value
        self.scan_time_allowance = self.get_parameter(
            'scan_time_allowance'
        ).value

        self.obstacle_stop_distance = self.get_parameter(
            'obstacle_stop_distance'
        ).value
        self.blocked_timeout = self.get_parameter('blocked_timeout').value
        self.backup_distance = self.get_parameter('backup_distance').value
        self.open_space_clearance = self.get_parameter(
            'open_space_clearance'
        ).value
        self.stuck_failure_count = self.get_parameter(
            'stuck_failure_count'
        ).value
        self.avoid_people = self.get_parameter('avoid_people').value
        self.yield_to_people = self.get_parameter('yield_to_people').value
        self.personal_space = self.get_parameter('personal_space').value

        # -----------------------------
        # Map
        # -----------------------------

        self.map = None
        self.observed = {}
        self.tf_stamps = {}
        self.latest_scan = None
        # (stop ROS time, monotonic start, whether a new SLAM /map is required)
        self.refresh_after = None
        self.recovery_move = None
        self.recovery_fault = None
        # (stop ROS time, need a new /map) when the latch was caused by
        # sensing that stopped updating. A clear way ahead isn't enough to
        # release it: that sensing must be fresh again first.
        self.fault_refresh = None
        # Scan stamp already judged for the current latch. A newer scan can
        # clear it; the one that caused the stop cannot.
        self.fault_checked_stamp = None
        # Own twist_mux input (priority 6): above Nav2, below keyboard and joystick.
        self.recovery_pub = self.create_publisher(Twist, '/cmd_vel_mapper', 10)
        # Measurement-only event stream used by the scoreboard. It does not
        # carry commands and has no effect on exploration decisions.
        event_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.event_pub = self.create_publisher(
            AutoMapperEvent, '/auto_mapper/events', event_qos)
        self.goal_sequence = 0
        self.active_goal_sequence = 0
        self.goal_outcome_emitted = True
        self.exploration_terminal_emitted = False
        # The goal Nav2 is driving to, for watchers such as socialnav_view.
        self.goal_pub = self.create_publisher(PoseStamped, '~/goal', 10)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', lambda msg: self.observe('scan', msg), sensor_qos)
        # Live lidar cloud. The accumulated 3D map (lidar_to_pointcloud's .ply)
        # is a different stream and is not reset to refresh nearby obstacles.
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/point_cloud2', lambda msg: self.observe('cloud', msg),
            sensor_qos)

        # /map is latched. A volatile subscription misses the grid that was
        # published before this node started.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        # Nav2's local costmap (odom frame): built from /scan, i.e. what the
        # controller itself avoids.
        self.local_costmap = None
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.local_costmap_callback,
            map_qos
        )

        # (people, monotonic receipt time), from object_detection.
        self.people = ([], None)
        # When the robot started waiting for a person ahead, and whether this
        # person has used up their wait (see yield_for_people).
        self.yield_since = None
        self.yield_spent = False
        if self.avoid_people or self.yield_to_people:
            self.people_sub = self.create_subscription(
                String, '/people',
                lambda msg: setattr(self, 'people', (parse_people(msg.data), time.monotonic())),
                10)

        # -----------------------------
        # TF
        # -----------------------------

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # -----------------------------
        # Nav2
        # -----------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.current_goal = None
        self.goal_active = False
        self.current_goal_handle = None
        self.goal_sent_time = None

        # (x, y) -> time.monotonic() of goals Nav2 rejected or failed,
        # so we don't immediately re-select the same bad spot forever.
        self.failed_goals = []
        # Longer than a few failed attempts (~25 s each), or the same handful
        # of nearby spots just rotate through the list.
        self.failed_goal_cooldown_sec = 120.0
        self.failed_goal_radius = 0.5

        # Frontiers a visit didn't shrink (the scan at the goal couldn't see
        # into the opening), as (world keys, time.monotonic()); skipped for
        # failed_goal_cooldown_sec like failed goals. goal_frontier is the
        # target of the goal in flight; frontier_to_verify is checked once
        # the scan at the goal is done and SLAM has published the result.
        self.dead_frontiers = []
        self.goal_frontier = None
        self.frontier_to_verify = None
        self.verify_not_before = None

        # If Nav2 accepts a goal but then never reports a result (e.g. the
        # action server drops the connection after accepting), neither
        # done-callback fires and goal_active would stay True forever,
        # silently halting exploration. This bounds how long we'll wait.
        self.goal_timeout_sec = 60.0

        # Nav2's behavior server turns or backs the robot up, checking the
        # local costmap for collisions. One maneuver runs at a time.
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.backup_client = ActionClient(self, BackUp, '/backup')
        self.scan_pending = self.get_parameter('scan_on_start').value
        self.backup_pending = False
        self.last_backup_time = None
        self.maneuver_name = None
        self.maneuver_active = False
        self.maneuver_handle = None
        self.maneuver_sent_time = None
        self.maneuver_time_allowance = 0.0
        # Each turn/back-up gets a number, so a Nav2 reply that arrives after
        # it was abandoned can't change the state of a newer one.
        self.maneuver_generation = 0
        # A timed-out maneuver was cancelled but Nav2 hasn't confirmed it
        # ended yet; until then it may still be moving the robot.
        self.maneuver_stopping = False
        # The action client of the current maneuver, and since when (monotonic)
        # its server has been missing while stopping (a server that is gone
        # can't be moving the robot).
        self.maneuver_client = None
        self.maneuver_server_missing_since = None
        # Maneuvers Nav2 accepted after they were abandoned: cancelled, and
        # motion stays blocked until each is confirmed ended.
        self.orphans = []

        # Set when a goal is abandoned for an obstacle ahead: the heading
        # (map frame) that was blocked and when, so the next goals avoid that
        # direction and favor longer walks into open space for a while.
        self.blocked_since = None
        self.blocked_heading = None
        self.blocked_time = None
        # Pose on the previous watch_ahead tick (frame, x, y, yaw, monotonic),
        # to tell driving forward from turning in place.
        self.last_watch_pose = None

        # Turning in place when no translation is safe (see
        # _try_recovery_turn): when the last one started, its angle, and the
        # direction (1 left, -1 right) of one Nav2 aborted, tried last.
        self.last_recovery_turn_time = None
        self.recovery_turn_angle = 0.0
        self.failed_turn_sign = 0

        # Getting unstuck (see request_escape / start_escape).
        self.consecutive_failures = 0
        self.no_goal_cycles = 0
        self.escape_pending = False
        self.goal_is_escape = False
        self.last_escape_time = None

        self.exploration_complete_logged = False

        # Shutdown (see stop()): set once stopping starts, so nothing new moves
        # the robot. pending_submissions holds (future, action name) for goals
        # Nav2 hasn't accepted or rejected yet.
        self.shutting_down = False
        self.pending_submissions = []

        # Don't start immediately.
        self.timer = self.create_timer(
            2.0,
            self.exploration_loop
        )

        # Faster than the exploration loop: at 0.5 m/s the robot covers
        # 1 m in 2 s, too far between looks at what's ahead.
        self.watch_timer = self.create_timer(
            WATCH_PERIOD_SEC,
            self.watch_ahead
        )

        # /frontier_explorer/scan: a 360 degree scan on request (the RViz
        # AutoMapper panel's Scan button, or `ros2 service call`).
        self.goal_cancelled_for_scan = False
        self.scan_service = self.create_service(
            Trigger, '~/scan', self.handle_scan_request
        )

        self.setup_vlm()
        nav_log_dir = self.get_parameter('nav_log_dir').value
        self.nav_log = None
        if nav_log_dir:
            self.nav_log = NavLog(nav_log_dir, warn=self.get_logger().warn)
            self.get_logger().info(f'Keeping a navigation log in {nav_log_dir}.')

        self.get_logger().info(
            'Go2 Frontier Explorer started.'
        )
        self.emit_event('exploration_started', detail='frontier explorer started')

    def emit_event(self, event_type, detail='', x=math.nan, y=math.nan,
                   goal_sequence=None):
        """Publish a structured observation without changing robot state."""
        try:
            event = AutoMapperEvent()
            event.stamp = self.get_clock().now().to_msg()
            event.event_type = event_type
            event.goal_sequence = int(
                self.active_goal_sequence if goal_sequence is None else goal_sequence)
            event.target_x = float(x)
            event.target_y = float(y)
            event.detail = str(detail)
            self.event_pub.publish(event)
        except Exception as error:
            # Scoring is optional. A reporting failure must never change the
            # explorer's goal, recovery, or stop behavior.
            self.get_logger().warn(
                f'Could not publish automapper event {event_type}: {error}',
                throttle_duration_sec=10.0,
            )

    def emit_goal_outcome(self, event_type, detail=''):
        """Emit at most one terminal outcome for the active goal."""
        if self.goal_outcome_emitted or self.active_goal_sequence == 0:
            return
        x, y = self.current_goal if self.current_goal is not None else (math.nan, math.nan)
        self.emit_event(event_type, detail, x, y)
        self.goal_outcome_emitted = True

    # ==========================================================
    # MAP CALLBACK
    # ==========================================================

    def map_callback(self, msg):

        expected = msg.info.width * msg.info.height
        if (
            msg.info.width == 0
            or msg.info.height == 0
            or msg.info.resolution <= 0.0
            or len(msg.data) < expected
        ):
            self.get_logger().warn(
                f'Ignoring occupancy grid: {msg.info.width}x{msg.info.height}, '
                f'resolution={msg.info.resolution}, {len(msg.data)} cells',
                throttle_duration_sec=5.0
            )
            return

        self.map = msg
        self.observe('map', msg)

    def local_costmap_callback(self, msg):

        if msg.info.width and msg.info.height and msg.info.resolution > 0.0:
            if len(msg.data) < msg.info.width * msg.info.height:
                return
            self.local_costmap = msg
            self.observe('costmap', msg)

    # ==========================================================
    # GET ROBOT POSITION
    # ==========================================================

    def get_robot_pose(self, frame):
        """(x, y, yaw) of the robot in `frame`, or None if TF can't tell."""

        try:
            stamped = self.tf_buffer.lookup_transform(
                frame, self.robot_frame, rclpy.time.Time()
            )
            stamp = stamped.header.stamp
            age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
            if age < 0 or age > 1.0:
                return None
            t = stamped.transform
        except TransformException:
            return None

        q = t.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return t.translation.x, t.translation.y, yaw

    def local_view(self):
        """The local costmap plus the robot's pose in its frame, and the yaw
        to add to a map-frame heading to get a costmap-frame one. None until
        Nav2 publishes a costmap and TF has both poses."""

        if self.local_costmap is None or not self.data_fresh(('costmap', 'scan')):
            return None

        local_pose = self.get_robot_pose(self.local_costmap.header.frame_id)
        map_pose = self.get_robot_pose(self.map_frame)
        if local_pose is None or map_pose is None:
            return None

        x, y, yaw = local_pose
        return self.local_costmap, x, y, yaw, yaw - map_pose[2]

    def get_robot_position(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return x, y

        except TransformException:

            return None

    def people_zones(self):
        """Circles around people, their next steps and groups, in the map
        frame; empty when /people is stale or TF can't place it."""

        people, received = self.people
        if not people or received is None or time.monotonic() - received > PEOPLE_MAX_AGE_SEC:
            return []
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, PEOPLE_FRAME, rclpy.time.Time()
            ).transform
        except TransformException:
            return []
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return social_zones(
            people, t.translation.x, t.translation.y, yaw, personal=self.personal_space
        )

    # ==========================================================
    # MAP CELL HELPERS
    # ==========================================================

    def index(self, x, y):

        return y * self.map.info.width + x

    def valid_cell(self, x, y):

        return (
            0 <= x < self.map.info.width
            and
            0 <= y < self.map.info.height
        )

    def cell_value(self, x, y):

        return self.map.data[
            self.index(x, y)
        ]

    # ==========================================================
    # FRONTIER DETECTION
    # ==========================================================

    def find_frontiers(self):
        """Groups of free cells bordering unknown space (see
        frontier_scoring.find_frontier_clusters)."""

        return find_frontier_clusters(
            self.map.data,
            self.map.info.width,
            self.map.info.height,
            self.min_frontier_size,
        )

    # ==========================================================
    # FRONTIER CENTER
    # ==========================================================

    def frontier_center(self, frontier):

        mean_x = sum(p[0] for p in frontier) / len(frontier)
        mean_y = sum(p[1] for p in frontier) / len(frontier)

        # The frontier cell nearest the mean, not the mean itself: a frontier
        # that wraps around the robot (common right after the startup scan)
        # has its mean on the robot, which made every goal a short hop.
        return min(
            frontier,
            key=lambda p: (p[0] - mean_x) ** 2 + (p[1] - mean_y) ** 2
        )

    def frontier_targets(self, frontier):
        """Points to aim at on one frontier: its center, plus a cell about
        every FRONTIER_SAMPLE_SPACING_M for long frontiers. A long frontier
        (e.g. the whole edge of a room) then offers its far parts too, not
        just the part nearest its middle."""

        step = max(
            1,
            int(FRONTIER_SAMPLE_SPACING_M / self.map.info.resolution)
        )

        return [self.frontier_center(frontier)] + frontier[::step]

    # ==========================================================
    # GRID → WORLD
    # ==========================================================

    def world_to_grid(self, wx, wy):

        resolution = self.map.info.resolution

        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        gx = (wx - origin_x) / resolution
        gy = (wy - origin_y) / resolution

        return gx, gy

    def grid_to_world(self, gx, gy):

        resolution = self.map.info.resolution

        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution

        return wx, wy

    # ==========================================================
    # PICK FRONTIER
    # ==========================================================

    def near_obstacle(self, gx, gy):
        """True if an occupied map cell is within wall_clearance of (gx, gy)."""

        radius = int(math.ceil(self.wall_clearance / self.map.info.resolution))

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                x, y = gx + dx, gy + dy
                if (
                    self.valid_cell(x, y)
                    and self.cell_value(x, y) >= OCCUPIED_THRESHOLD
                ):
                    return True

        return False

    def is_goal_blacklisted(self, x, y):
        now = time.monotonic()

        # Drop expired entries so this list doesn't grow unbounded over
        # a long exploration run.
        self.failed_goals = [
            (fx, fy, t) for fx, fy, t in self.failed_goals
            if now - t < self.failed_goal_cooldown_sec
        ]

        for fx, fy, _ in self.failed_goals:
            if math.hypot(x - fx, y - fy) < self.failed_goal_radius:
                return True

        return False

    def choose_frontier(self, frontiers, robot_x, robot_y):

        candidates = []

        # What Nav2 sees around the robot, for the straight-line and
        # blocked-direction checks below. Without it (no costmap yet), every
        # direction counts as open.
        view = self.local_view()
        blocked = self.recently_blocked() and self.blocked_heading is not None
        zones = self.people_zones() if self.avoid_people else []

        # Skip frontiers a recent visit didn't shrink: the scan at the goal
        # couldn't see into them, so going back would just repeat that.
        self.prune_dead_frontiers()
        live = [
            frontier for frontier in frontiers
            if not overlaps_dead(
                self.frontier_keys(frontier),
                [keys for keys, _ in self.dead_frontiers],
                DEAD_FRONTIER_OVERLAP,
            )
        ]

        targets = [
            (frontier, point)
            for frontier in live
            for point in self.frontier_targets(frontier)
        ]

        for frontier, (gx, gy) in targets:

            # Convert frontier center to world coordinates.
            frontier_x, frontier_y = self.grid_to_world(gx, gy)

            distance = math.sqrt(
                (frontier_x - robot_x) ** 2
                +
                (frontier_y - robot_y) ** 2
            )

            # Don't choose something extremely close to the robot.
            # Checked before the neighbor search below so we don't pay
            # for an 17x17-cell scan on a candidate we'll discard anyway.
            if distance < 0.5:
                continue

            # Stop short of the unknown boundary. A goal on the frontier
            # cell itself is rejected by Nav2. goal_distance is how far
            # back toward the robot that goal should sit.
            pull = min(float(self.goal_distance), max(0.0, distance - 0.5))
            if distance > 1e-6:
                target_x = frontier_x + (robot_x - frontier_x) * (pull / distance)
                target_y = frontier_y + (robot_y - frontier_y) * (pull / distance)
            else:
                target_x, target_y = frontier_x, frontier_y

            target_gx, target_gy = self.world_to_grid(target_x, target_y)

            search_radius = 8

            center_x = int(math.floor(target_gx))
            center_y = int(math.floor(target_gy))

            free_cells = []

            for dy in range(-search_radius, search_radius + 1):

                for dx in range(-search_radius, search_radius + 1):

                    x = center_x + dx
                    y = center_y + dy

                    if not self.valid_cell(x, y):
                        continue

                    # Goal must be known free space.
                    if self.cell_value(x, y) != 0:
                        continue

                    wx, wy = self.grid_to_world(x, y)

                    d = math.hypot(wx - target_x, wy - target_y)

                    free_cells.append((d, x, y, wx, wy))

            # Closest free cell that isn't against a wall. Checking clearance
            # only until the first hit keeps this cheap.
            free_cells.sort()
            best_goal = next(
                (
                    (wx, wy) for _, x, y, wx, wy in free_cells
                    if not self.near_obstacle(x, y)
                ),
                None
            )

            if best_goal is None:
                continue

            goal_x, goal_y = best_goal

            # Skip spots Nav2 recently rejected or failed to reach, so a
            # single bad frontier (e.g. too close to an obstacle) doesn't
            # get re-selected and retried forever.
            if self.is_goal_blacklisted(goal_x, goal_y):
                continue

            goal_range = math.hypot(goal_x - robot_x, goal_y - robot_y)
            bearing = math.atan2(goal_y - robot_y, goal_x - robot_x)

            # Just turned away from an obstacle: take the longest walk on
            # offer, to get clear of that spot. Otherwise the score is the
            # information gain, filled in below for the best tier only.
            score = -goal_range if blocked else None

            is_far = goal_range >= self.min_goal_distance

            line_clear = True
            if view is not None:
                grid, local_x, local_y, _, map_to_local_yaw = view
                line_clear = straight_line_clear(
                    grid, local_x, local_y,
                    bearing + map_to_local_yaw,
                    min(goal_range, LINE_CHECK_M)
                )

            toward_blocked = blocked and (
                angle_difference(bearing, self.blocked_heading)
                < BLOCKED_CONE_RAD
            )

            # 2: the goal is in someone's space; 1: the straight way there
            # crosses a person, their next steps or a group; 0: neither.
            social = 0
            if zone_at(goal_x, goal_y, zones) is not None:
                social = 2
            elif line_crosses(robot_x, robot_y, goal_x, goal_y, zones) is not None:
                social = 1

            candidates.append(
                (
                    score,
                    goal_x,
                    goal_y,
                    len(frontier),
                    is_far,
                    line_clear,
                    toward_blocked,
                    goal_range,
                    frontier,
                    social
                )
            )

        if not candidates:
            return None

        # In order: stay away from a just-blocked direction, then out of
        # people's space, then walk far enough, then have a clear straight
        # line in Nav2's costmap and past people, then the score. Each only
        # breaks ties of the ones before it, so exploring never stalls when
        # no candidate is ideal.
        def tier(c):
            return (c[6], c[9] == 2, not c[4], not c[5], c[9] == 1)

        best_tier = min(tier(c) for c in candidates)
        pool = [c for c in candidates if tier(c) == best_tier]

        if not blocked:
            # Prefer the goal that would reveal the most unknown space for
            # the walk; ranking by distance and frontier size made the robot
            # nibble the closest edge. Ray-casting is the costly part, so
            # it's done for this tier only. Lower score is better.
            pool = [
                (
                    -visit_utility(
                        self.revealed_from(c[1], c[2]),
                        c[7],
                        self.info_gain_cells_per_meter,
                    ),
                ) + c[1:]
                for c in pool
            ]
            if self.vlm_client is not None:
                pool = self.rank_with_vlm(pool, robot_x, robot_y)
                if pool is None:
                    return WAITING_FOR_VLM
                if self.vlm_mode == 'active':
                    pool = self.follow_suggestion(pool, robot_x, robot_y)

        best = min(pool, key=lambda c: c[0])

        # (score, x, y, frontier size, frontier cells)
        return best[:4] + (best[8],)

    # ==========================================================
    # VLM FRONTIER RANKING (vlm_ranking.py)
    # ==========================================================

    def setup_vlm(self):
        """Camera subscription and server client, unless vlm_mode is off."""

        self.vlm_client = None
        self.vlm_mode = vlm_mode_name(self.get_parameter('vlm_mode').value)
        if self.vlm_mode == 'off':
            return
        if self.vlm_mode not in ('shadow', 'active'):
            self.get_logger().warn(
                f"Unknown vlm_mode '{self.vlm_mode}' (off, shadow or active); "
                'not using the VLM.'
            )
            return

        url = self.get_parameter('vlm_url').value
        self.vlm_weight = self.get_parameter('vlm_weight').value
        self.vlm_timeout = self.get_parameter('vlm_timeout').value
        self.vlm_candidates = max(1, self.get_parameter('vlm_candidates').value)
        self.vlm_include_longest = bool(self.get_parameter('vlm_include_longest').value)
        self.vlm_client = VlmClient(
            url,
            self.get_parameter('vlm_model').value,
            self.vlm_timeout,
            encode_jpeg,
            api_key=os.environ.get('VLM_API_KEY'),
        )
        self.vlm_ranking = None
        self.vlm_wait_started = None
        self.vlm_retry_after = 0.0
        self.views = ViewMemory(bins=VIEW_BINS)
        self.last_view_time = 0.0
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.vlm_suggest_weight = self.get_parameter('vlm_suggest_weight').value
        self.vlm_suggestion = None
        if self.vlm_mode == 'active':
            self.suggestion_sub = self.create_subscription(
                String, '/vlm/suggestion', self.suggestion_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
        self.get_logger().info(f'VLM ranking in {self.vlm_mode} mode, asking {url}.')

    def suggestion_callback(self, msg):
        self.vlm_suggestion = msg.data

    def follow_suggestion(self, pool, robot_x, robot_y):
        """The pool with goals the way the VLM suggests scoring better, if it
        has a recent suggestion the lidar agrees with. Only goals the map
        already offered can win, and Nav2 still drives to them."""

        heading = usable_heading(self.vlm_suggestion, time.time())
        if heading is None:
            return pool

        ranked = [
            (c[0] - suggestion_bonus(
                math.atan2(c[2] - robot_y, c[1] - robot_x), heading, self.vlm_suggest_weight),
             ) + c[1:]
            for c in pool
        ]
        before = min(pool, key=lambda c: c[0])
        after = min(ranked, key=lambda c: c[0])
        if before[1:3] != after[1:3]:
            self.get_logger().info(
                f'VLM suggestion changes the goal from ({before[1]:.2f}, {before[2]:.2f}) '
                f'to ({after[1]:.2f}, {after[2]:.2f}).'
            )
        return ranked

    def camera_callback(self, msg):
        """Remember the frame by the heading it was taken at. A few per
        second is plenty: the robot turns a full circle in about 10 s."""

        now = time.monotonic()
        if now - self.last_view_time < VIEW_PERIOD_SEC:
            return
        pose = self.get_robot_pose(self.map_frame)
        if pose is None:
            return
        self.last_view_time = now
        self.views.remember(*pose, msg, now)

    def rank_with_vlm(self, pool, robot_x, robot_y):
        """The pool with the VLM's opinion added to the best few scores, the
        pool unchanged if the VLM can't help now, or None to wait one more
        loop (up to vlm_timeout per goal choice) for an answer on its way.
        Shadow mode waits the same way but then keeps the map's pick.
        """

        now = time.monotonic()
        result = self.vlm_client.take_result()
        if result is not None:
            if result.scores is None:
                self.get_logger().warn(
                    f'VLM gave no ranking ({result.reason}); using the map '
                    f'score alone for {VLM_RETRY_SEC:.0f} s.'
                )
                self.vlm_retry_after = now + VLM_RETRY_SEC
            else:
                self.vlm_ranking = result

        top = vlm_shortlist(pool, self.vlm_candidates, self.vlm_include_longest)
        fresh = (
            self.vlm_ranking is not None
            and now - self.vlm_ranking.finished_at < VLM_RESULT_MAX_AGE_SEC
        )
        scores = [
            match_score(self.vlm_ranking, c[1], c[2], VLM_MATCH_RADIUS_M) if fresh else None
            for c in top
        ]

        if None in scores:
            if now < self.vlm_retry_after:
                return pool
            if not self.vlm_client.busy() and not self.ask_vlm(top, robot_x, robot_y):
                return pool
            # Bounded per goal choice, not per request: goals that shift
            # with each map update must not keep the robot waiting.
            if self.vlm_wait_started is None:
                self.vlm_wait_started = now
            # Shadow mode waits too: otherwise the goal is chosen before the
            # answer comes, and by the next choice (minutes later) the answer
            # is stale, so shadow never compared anything. Waiting the same way
            # also makes shadow a fair control for active: only steering differs.
            if now - self.vlm_wait_started < self.vlm_timeout:
                self.get_logger().info(
                    'Waiting for the VLM ranking...', throttle_duration_sec=5.0
                )
                return None
            self.vlm_wait_started = None
            return pool

        self.vlm_wait_started = None

        ranked = [
            (c[0] - vlm_bonus(s, self.vlm_weight),) + c[1:]
            for c, s in zip(top, scores)
        ]
        ours = top[0]
        theirs = min(ranked, key=lambda c: c[0])
        verdict = 'agrees' if theirs[1:3] == ours[1:3] else 'differs'
        self.get_logger().info(
            f'VLM {verdict}: map score picks ({ours[1]:.2f}, {ours[2]:.2f}), '
            f'with VLM ({theirs[1]:.2f}, {theirs[2]:.2f}). Scores '
            f'{[round(s, 2) for s in scores]}: {self.vlm_ranking.reason}'
        )
        self.log_vlm_decision(robot_x, robot_y, top, scores, ours, theirs)
        # Spent: the next goal is asked about from wherever the robot is then.
        self.vlm_ranking = None
        return pool if self.vlm_mode == 'shadow' else ranked

    def log_vlm_decision(self, robot_x, robot_y, top, scores, ours, theirs):
        """
        Note this ranking in the navigation log, with the map as it is now.

        In shadow mode the VLM's pick is never driven to, so this record is the only trace of
        it; a scorer can later count what each pick would have revealed. A logging failure must
        not stop exploring.
        """
        if self.nav_log is None or self.map is None:
            return
        try:
            info = self.map.info
            candidates = [
                {'x': round(c[1], 3), 'y': round(c[2], 3), 'map_score': round(c[0], 1),
                 'vlm_score': round(s, 3), 'distance_m': round(c[7], 2)}
                for c, s in zip(top, scores)
            ]
            self.nav_log.decision(
                (robot_x, robot_y), candidates,
                next(i for i, c in enumerate(top) if c[1:3] == ours[1:3]),
                next(i for i, c in enumerate(top) if c[1:3] == theirs[1:3]),
                self.vlm_mode,
                (info.resolution, info.origin.position.x, info.origin.position.y,
                 info.width, info.height, self.map.data),
            )
        except Exception as e:
            self.get_logger().warn(f'Navigation log error: {e}')

    def ask_vlm(self, top, robot_x, robot_y):
        """Send the camera views toward the best goals; False if there are
        none to send (e.g. the robot hasn't turned around here yet)."""

        now = time.monotonic()
        frames = [
            self.views.view_toward(
                robot_x, robot_y,
                math.atan2(c[2] - robot_y, c[1] - robot_x),
                now, VIEW_MAX_OFFSET_M, VIEW_MAX_ANGLE_RAD, VIEW_MAX_AGE_SEC,
            )
            for c in top
        ]
        if all(frame is None for frame in frames):
            self.get_logger().info(
                'No camera view toward any goal yet; using the map score.',
                throttle_duration_sec=10.0
            )
            return False
        # The score is -(revealed - cells_per_meter * walk), so the cells can be
        # had back without casting the rays again.
        # Frontier length from its cell count, and which goal is on the
        # longest one (only said when the shortlist looks for it).
        resolution = self.map.info.resolution
        longest_cells = max(c[3] for c in top)
        facts = [
            frontier_fact(
                c[7],
                max(0, round(self.info_gain_cells_per_meter * c[7] - c[0])),
                c[3] * resolution,
                self.vlm_include_longest and c[3] == longest_cells,
            )
            for c in top
        ]
        self.vlm_client.submit([(c[1], c[2]) for c in top], frames, facts)
        return True

    def revealed_from(self, x, y):
        """Unknown cells the lidar would see from world point (x, y)."""

        resolution = self.map.info.resolution
        gx, gy = self.world_to_grid(x, y)
        return count_revealed_unknown(
            self.map.data,
            self.map.info.width,
            self.map.info.height,
            int(math.floor(gx)),
            int(math.floor(gy)),
            int(self.info_gain_range / resolution),
            int(self.info_gain_rays),
        )

    def frontier_keys(self, frontier):
        """A frontier's cells as world keys, which stay valid when SLAM
        grows the map and the grid origin moves.

        Keyed on each cell's floor corner. The center sits half a cell
        higher, and rounding that half merges neighboring cells.
        """

        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        return frozenset(
            world_key(
                origin_x + gx * resolution,
                origin_y + gy * resolution,
                resolution,
            )
            for gx, gy in frontier
        )

    def prune_dead_frontiers(self):

        now = time.monotonic()
        self.dead_frontiers = [
            (keys, t) for keys, t in self.dead_frontiers
            if now - t < self.failed_goal_cooldown_sec
        ]

    def verify_frontier_shrank(self, frontiers):
        """After the scan at a goal, check the target frontier got smaller.

        The goal sits goal_distance back from the unknown edge and the robot
        spins there; if the frontier is still as big, it's an opening the
        scan can't see into, and the robot would otherwise keep returning to
        it until it declared itself stuck. Give up on it for a while.
        """

        target_keys, size_before = self.frontier_to_verify
        self.frontier_to_verify = None
        self.verify_not_before = None

        size_after = remaining_size(
            target_keys, [self.frontier_keys(f) for f in frontiers]
        )

        if size_after >= size_before:
            self.dead_frontiers.append((target_keys, time.monotonic()))
            self.get_logger().info(
                f'Frontier unchanged after the visit ({size_after} cells); '
                f'skipping it for {self.failed_goal_cooldown_sec:.0f} s.'
            )
        else:
            self.get_logger().info(
                f'Frontier shrank from {size_before} to {size_after} cells.'
            )

    # ==========================================================
    # SEND NAVIGATION GOAL
    # ==========================================================

    def send_goal(self, x, y, is_escape=False):
        """Send a Nav2 goal; False if Nav2 isn't ready yet."""

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'Waiting for Nav2...'
            )
            return False

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Keep robot orientation unchanged.
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending {"escape" if is_escape else "exploration"} goal: '
            f'x={x:.2f}, y={y:.2f}'
        )

        self.goal_active = True
        self.goal_is_escape = is_escape
        self.goal_sequence += 1
        self.active_goal_sequence = self.goal_sequence
        sequence = self.active_goal_sequence
        self.goal_outcome_emitted = False
        # Set by the caller for frontier goals once this returns True.
        self.goal_frontier = None
        self.current_goal = (x, y)
        self.goal_sent_time = time.monotonic()
        self.emit_event(
            'goal_sent', 'escape goal' if is_escape else 'exploration goal', x, y)
        if is_escape:
            self.emit_event('recovery_started', 'navigation to open space', x, y)

        self.goal_pub.publish(goal.pose)
        future = self.nav_client.send_goal_async(
            goal
        )
        self._track_submission(future, 'navigate_to_pose')

        future.add_done_callback(
            lambda result: self.goal_response_callback(result, sequence)
        )

        return True

    # ==========================================================
    # NAV2 RESPONSE
    # ==========================================================

    def _is_current_navigation_goal(self, sequence):
        """Late action replies must not mutate a newer goal or a stopped run."""
        return (
            self.goal_active and not self.shutting_down
            and (sequence is None or sequence == self.active_goal_sequence)
        )

    def goal_response_callback(self, future, sequence=None):

        try:

            goal_handle = future.result()

        except Exception as e:

            if not self._is_current_navigation_goal(sequence):
                return

            self.get_logger().error(
                f'Navigation goal failed: {e}'
            )

            self.record_goal_failure(f'goal response error: {e}')
            self.goal_active = False
            self.goal_sent_time = None
            return

        if not self._is_current_navigation_goal(sequence):
            # A submission can be accepted after timeout or shutdown. Never
            # attach it to the new goal; request cancellation of its own handle.
            if goal_handle.accepted:
                try:
                    self._adopt_orphan(goal_handle, self.nav_client)
                except Exception as error:
                    self._stop_for(
                        f'Could not cancel inactive goal {sequence}: {error}'
                    )
            return

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Nav2 rejected the exploration goal.'
            )

            self.record_goal_failure('Nav2 rejected goal')
            self.goal_active = False
            self.goal_sent_time = None
            return

        self.current_goal_handle = goal_handle

        self.get_logger().info(
            'Exploration goal accepted.'
        )
        x, y = self.current_goal if self.current_goal is not None else (math.nan, math.nan)
        self.emit_event('goal_accepted', 'Nav2 accepted goal', x, y)

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda result: self.navigation_result_callback(result, sequence)
        )

    def blacklist_current_goal(self):
        if self.current_goal is not None:
            x, y = self.current_goal
            self.failed_goals.append((x, y, time.monotonic()))

    def record_goal_failure(self, detail='navigation failed'):
        """A goal was rejected, failed or stalled: avoid that spot for a
        while, and treat several failures in a row as being stuck."""

        self.emit_goal_outcome('goal_failed', detail)
        self.blacklist_current_goal()
        # Never reached, so there's no visit to judge the frontier by.
        self.goal_frontier = None

        if self.goal_is_escape:
            # Don't escape from a failed escape (request_escape's cooldown
            # also guards this); just go back to exploring.
            self.get_logger().warn(
                'Could not reach the open space; carrying on exploring.'
            )
            return

        self.consecutive_failures += 1
        if self.consecutive_failures >= self.stuck_failure_count:
            self.consecutive_failures = 0
            self.request_escape(
                f'{self.stuck_failure_count} goals failed in a row'
            )

    # ==========================================================
    # NAVIGATION RESULT
    # ==========================================================

    def navigation_result_callback(self, future, sequence=None):

        if not self._is_current_navigation_goal(sequence):
            self.get_logger().info(
                f'Ignoring result for inactive navigation goal {sequence}.'
            )
            return

        try:

            result = future.result()

            status = result.status

            if status == 4:

                self.get_logger().info(
                    'Reached open space.' if self.goal_is_escape
                    else 'Reached exploration goal.'
                )
                self.consecutive_failures = 0
                self.emit_goal_outcome('goal_reached', 'Nav2 reached goal')
                if self.goal_is_escape:
                    self.emit_event('recovery_finished', 'reached open space')
                self.log_goal_ended('reached')

                if self.scan_after_goal or self.goal_is_escape:
                    self.scan_pending = True

                # Check the target frontier shrank, after the scan if there
                # is one (see verify_frontier_shrank).
                if self.goal_frontier is not None:
                    self.frontier_to_verify = self.goal_frontier
                    self.verify_not_before = None
                self.goal_frontier = None

            elif self.goal_cancelled_for_scan:

                # Interrupted for a requested scan: not the goal's fault.
                self.get_logger().info('Goal paused for the requested scan.')
                self.emit_goal_outcome('goal_cancelled', 'paused for requested scan')
                self.log_goal_ended('paused')

            else:

                self.get_logger().warn(
                    f'Navigation ended with status {status}'
                )
                self.log_goal_ended('failed')

                self.record_goal_failure(f'Nav2 result status {status}')

        except Exception as e:

            self.get_logger().error(
                f'Navigation result error: {e}'
            )

            self.record_goal_failure(f'navigation result error: {e}')

        self.current_goal_handle = None
        self.goal_active = False
        self.goal_sent_time = None
        self.goal_cancelled_for_scan = False

    # ==========================================================
    # SCAN ON REQUEST
    # ==========================================================

    def handle_scan_request(self, request, response):
        """Turn 360 degrees as soon as possible: the current goal is paused
        (not counted as failed), and failed spots are forgiven, since the
        scan gives SLAM fresh data and they may be reachable now."""

        if self.shutting_down:
            response.success = False
            response.message = 'The explorer is shutting down.'
            return response

        if self.maneuver_active and self.maneuver_name == 'Scan':
            response.success = False
            response.message = 'Already scanning.'
            return response

        # Scan clears a latched stop, but not one whose cause may still be
        # live: a cancelled maneuver Nav2 hasn't confirmed ended, or sensing
        # that hasn't updated since it went quiet.
        if self.orphans:
            response.success = False
            response.message = (
                'Waiting for Nav2 to confirm a late turn/back-up has ended.'
            )
            return response
        if self.maneuver_stopping:
            response.success = False
            response.message = (
                f'Waiting for Nav2 to confirm the cancelled '
                f'{self.maneuver_name} has ended.'
            )
            return response
        if self.fault_refresh is not None:
            ready, why = self._refresh_check(*self.fault_refresh)
            if not ready:
                response.success = False
                response.message = f'Not safe to move yet: {why}.'
                return response

        if self.goal_active and self.current_goal_handle is not None:
            self.goal_cancelled_for_scan = True
            self.current_goal_handle.cancel_goal_async()

        self.recovery_fault = None
        self.fault_refresh = None
        self.fault_checked_stamp = None
        self.refresh_after = None
        self.failed_goals = []
        self.dead_frontiers = []
        self.blocked_time = None
        self.scan_pending = True

        self.get_logger().info('Scan requested.')
        response.success = True
        response.message = (
            'Scan queued: it starts once the current move stops.'
            if self.goal_active or self.maneuver_active
            else 'Scan starting.'
        )
        return response

    # ==========================================================
    # MANEUVERS: 360 DEGREE SCAN, BACKING AWAY
    # ==========================================================

    def start_scan(self):

        goal = Spin.Goal()
        goal.target_yaw = float(self.scan_yaw)
        goal.time_allowance = Duration(sec=int(self.scan_time_allowance))

        if self.start_maneuver(
            self.spin_client, goal, 'Scan', self.scan_time_allowance
        ):
            self.scan_pending = False
            self.get_logger().info('Turning in place to scan the surroundings.')

    def observe(self, name, msg):
        if name == 'scan':
            self.latest_scan = msg
        stamp = msg.header.stamp
        self.observed[name] = (time.monotonic(), stamp.sec + stamp.nanosec * 1e-9)

    def data_fresh(self, names, after=None):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        return all(name in self.observed
                   and now - self.observed[name][0] < 3.0
                   and 0 <= ros_now - self.observed[name][1] < 3.0
                   and (after is None or self.observed[name][1] > after)
                   for name in names)

    def _hold_stop(self):
        """Publish a zero twist so Nav2's lower-priority command cannot move
        the robot while recovery is deciding, waiting, or latched stopped."""

        self.recovery_pub.publish(Twist())

    def _stop_for(self, reason, refresh=None):
        """Latch a stop. `refresh` is (stop ROS time, need a new /map) when
        the cause was sensing that stopped updating; the latch then also
        waits for that sensing, not just for a clear way ahead."""

        self.recovery_fault = reason
        self.fault_refresh = refresh
        self.fault_checked_stamp = self._ros_stamp('scan')
        # A latched stop always ends a recovery translation: tick_recovery
        # publishes on the same mux input and would overwrite the zero.
        self.recovery_move = None
        self._hold_stop()
        if ((refresh is not None or 'stale' in reason or 'missing' in reason)
                and hasattr(self, 'emit_event')):
            self.emit_event('sensor_stale', reason)
        self.get_logger().error(
            f'Recovery stopped: {reason} '
            'A later fresh scan can clear this; Scan clears it now.'
        )

    def _consider_clearing_fault(self):
        """A latched stop stays put until a new fresh scan changes the picture.

        If the way ahead is clear, exploration may continue. If it is still
        blocked but a rear or side corridor is open, take that move. The same
        scan is not judged twice.
        """

        self._hold_stop()

        # A cancelled turn/back-up may still be moving the robot until Nav2
        # confirms it ended (see maneuver_stopping). This runs here, not in
        # the exploration loop: the loop returns early while a stop is
        # latched, so it would never get to release the maneuver.
        if self.maneuver_active or self.orphans:
            self._check_unresolved()
            if self.maneuver_active or self.orphans:
                return

        # Stopped because sensing went quiet: a clear strip ahead proves
        # nothing about the cloud or map that stopped updating. Checked on
        # every attempt (kept until the latch actually releases, below), so
        # sensing that recovers and then dies again blocks the release.
        if self.fault_refresh is not None:
            ready, why = self._refresh_check(*self.fault_refresh)
            if not ready:
                self.get_logger().info(
                    f'Still stopped: {why}.', throttle_duration_sec=10.0
                )
                return

        scan = self.latest_scan
        age = self._scan_age()
        stamp = self._ros_stamp('scan')
        if scan is None or age is None or age > SCAN_MAX_AGE_SEC or stamp is None:
            return
        if self.fault_checked_stamp is not None and stamp <= self.fault_checked_stamp:
            return
        if scan.header.frame_id != self.robot_frame:
            self.fault_checked_stamp = stamp
            return
        view = self.local_view()
        if view is None:
            return
        self.fault_checked_stamp = stamp
        gap = free_distance_ahead(
            view[0], view[1], view[2], view[3], self.obstacle_stop_distance
        )
        distances = recovery_distances(self.backup_distance)
        move = None
        if gap is not None:
            move, _reason, leaving = assess_recovery_steps(
                *view[:4], distances, scan, age, SCAN_MAX_AGE_SEC
            )
        decision = latch_decision(gap is not None, move)
        if decision == 'hold':
            if self._try_recovery_turn(scan):
                return
            self.get_logger().info(
                'Still stopped: no safe direction on the latest scan.',
                throttle_duration_sec=10.0,
            )
            return
        reason = self.recovery_fault
        self.recovery_fault = None
        self.fault_refresh = None
        if decision == 'resume':
            self.get_logger().info(
                f'Recovery latch cleared; forward is clear (was: {reason}).'
            )
            return
        grid, x, y, yaw, _ = view
        self.recovery_move = (
            x, y, yaw, move, grid.header.frame_id, time.monotonic(), leaving
        )
        kind = 'leaving' if leaving else 'recovery'
        self.emit_event(
            'recovery_started',
            f'latched {kind} translation x={move[0]:.2f}, y={move[1]:.2f}',
        )
        self.get_logger().info(
            f'Fresh scan opened a way out: x={move[0]:.2f}, y={move[1]:.2f} m'
            + (', moving away from a wall close beside the body'
               if leaving else '')
            + f' (was: {reason}).'
        )

    def _scan_age(self):
        """Age of the latest scan by both its stamp and its arrival, or None
        (see recovery.scan_age)."""

        if 'scan' not in self.observed:
            return None
        received, stamp = self.observed['scan']
        return scan_age(
            received,
            time.monotonic(),
            stamp,
            self.get_clock().now().nanoseconds * 1e-9,
        )

    def _note_tf_stamps(self):
        """Record the latest odom and map transforms. A stamp from before the
        robot stopped does not describe the pose it stopped at."""

        odom_frame = 'odom'
        if self.local_costmap is not None:
            odom_frame = self.local_costmap.header.frame_id
        for frame, name in ((odom_frame, 'odom_tf'), (self.map_frame, 'map_tf')):
            try:
                stamped = self.tf_buffer.lookup_transform(
                    frame, self.robot_frame, rclpy.time.Time()
                )
            except TransformException:
                continue
            stamp = stamped.header.stamp
            self.tf_stamps[name] = stamp.sec + stamp.nanosec * 1e-9

    def _refresh_check(self, stop_ros, need_map):
        """(ready, reason): has every required source updated since
        `stop_ros`, and is it still updating now? See
        recovery.refresh_status and recovery.stale_sources."""

        self._note_tf_stamps()
        stamps = {
            'scan': self._ros_stamp('scan'),
            'cloud': self._ros_stamp('cloud'),
            'costmap': self._ros_stamp('costmap'),
            'odom_tf': self.tf_stamps.get('odom_tf'),
            'map_tf': self.tf_stamps.get('map_tf'),
            'slam_map': self._ros_stamp('map'),
        }
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        ready, reason = refresh_status(
            now_ros, stop_ros, REFRESH_TIMEOUT_SEC, stamps, need_map
        )
        if not ready:
            return ready, reason

        topics = {'scan': 'scan', 'cloud': 'cloud', 'costmap': 'costmap'}
        if need_map:
            topics['slam_map'] = 'map'
        sources = {
            name: (self.observed.get(topic, (None, None))[0], stamps[name])
            for name, topic in topics.items()
        }
        sources['odom_tf'] = (None, stamps['odom_tf'])
        sources['map_tf'] = (None, stamps['map_tf'])
        stale = stale_sources(
            now_ros, time.monotonic(), sources, SENSING_MAX_AGE_SEC
        )
        if stale:
            return False, (
                f'{", ".join(stale)} updated after the stop but not recently'
            )
        return True, ''

    def _ros_stamp(self, name):
        if name not in self.observed:
            return None
        return self.observed[name][1]

    def start_backup(self):
        """Short rear or side translation on the mapper's own mux input."""

        self.backup_pending = False
        self.last_backup_time = time.monotonic()
        self._hold_stop()
        view = self.local_view()
        scan = self.latest_scan
        if view is None or scan is None:
            self._stop_for('stale or missing scan, costmap, or transform')
            return
        if scan.header.frame_id != self.robot_frame:
            self._stop_for(
                f'scan frame is {scan.header.frame_id}, not {self.robot_frame}'
            )
            return
        distances = recovery_distances(self.backup_distance)
        move, reason, leaving = assess_recovery_steps(
            *view[:4], distances, scan, self._scan_age(), SCAN_MAX_AGE_SEC
        )
        if move is None:
            if self._try_recovery_turn(scan):
                return
            self._stop_for(reason)
            return
        grid, x, y, yaw, _ = view
        self.recovery_move = (
            x, y, yaw, move, grid.header.frame_id, time.monotonic(), leaving
        )
        kind = 'leaving' if leaving else 'recovery'
        self.emit_event(
            'recovery_started',
            f'{kind} translation x={move[0]:.2f}, y={move[1]:.2f}',
        )
        self.get_logger().info(
            f'Recovery translation: x={move[0]:.2f}, y={move[1]:.2f} m'
            + (', moving away from a wall close beside the body.'
               if leaving else '.')
        )
        self.log_event(f'recovery move x={move[0]:.2f} y={move[1]:.2f}')

    def _try_recovery_turn(self, scan):
        """No translation is safe: turn in place toward the most open heading
        on the scan with Nav2's Spin, which checks the footprint against the
        local costmap while turning. True when the turn was sent.

        Clears any latched stop first: the zero twist it publishes on the
        mapper's mux input (priority 6) would override Spin on Nav2's
        (priority 5). Not tried on a stale scan, and not again within
        RECOVERY_TURN_RETRY_SEC, so a turn that doesn't help can't make Oak
        swing back and forth.
        """

        now = time.monotonic()
        if (
            self.last_recovery_turn_time is not None
            and now - self.last_recovery_turn_time < RECOVERY_TURN_RETRY_SEC
        ):
            return False
        age = self._scan_age()
        if (
            scan is None or age is None or age > SCAN_MAX_AGE_SEC
            or scan.header.frame_id != self.robot_frame
        ):
            return False
        min_range = (
            ROBOT_FRONT_M + self.obstacle_stop_distance + RECOVERY_TURN_ROOM_M
        )
        angle = choose_turn(scan, min_range, self.failed_turn_sign)
        if angle is None and self.failed_turn_sign:
            angle = choose_turn(scan, min_range)
        if angle is None:
            return False

        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance = Duration(sec=int(RECOVERY_TURN_TIME_SEC))
        if not self.start_maneuver(
            self.spin_client, goal, RECOVERY_TURN_NAME, RECOVERY_TURN_TIME_SEC
        ):
            return False

        self.last_recovery_turn_time = now
        self.recovery_turn_angle = angle
        reason = self.recovery_fault
        self.recovery_fault = None
        self.fault_refresh = None
        self.recovery_move = None
        self.emit_event(
            'recovery_started', f'recovery turn {math.degrees(angle):.0f} deg'
        )
        self.get_logger().info(
            f'No safe translation; turning {math.degrees(angle):.0f} deg '
            'in place toward open space'
            + (f' (was: {reason}).' if reason else '.')
        )
        return True

    def _recovery_progress(self, pose, x, y, yaw, move, distance):
        """Distance along the commanded translation, and the cross-track error."""

        if pose is None:
            return 0.0, 0.0
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        body_x = (pose[0] - x) * cos_yaw + (pose[1] - y) * sin_yaw
        body_y = -(pose[0] - x) * sin_yaw + (pose[1] - y) * cos_yaw
        along = (body_x * move[0] + body_y * move[1]) / distance
        across = abs(body_x * move[1] - body_y * move[0]) / distance
        return along, across

    def _recovery_block_reason(self, pose, view, started, progress, distance,
                               across, move, start_pose, leaving=False):
        """Why the translation must stop, or None while it may continue.
        A leaving move keeps ignoring the walls it leaves behind, and must
        stay within the tighter drift limits that makes safe."""

        if pose is None or view is None:
            return 'odometry transform or costmap went stale during the move'
        if time.monotonic() - started > RECOVERY_TIME_LIMIT_SEC:
            return f'time limit of {RECOVERY_TIME_LIMIT_SEC:.0f} s reached'
        if across > drift_limits(leaving)[1]:
            return 'pose drifted off the commanded translation'
        if progress >= distance:
            return None
        age = self._scan_age()
        scan = self.latest_scan
        if scan is None or age is None or age > SCAN_MAX_AGE_SEC:
            return 'laser scan went stale during the move'
        if scan.header.frame_id != self.robot_frame:
            return f'scan frame is {scan.header.frame_id}, not {self.robot_frame}'
        # The costmap check stays anchored at the start pose, so its drift
        # envelope is the one the abort limits above enforce. The scan is in
        # the robot's current frame, so it checks the rest of the move from
        # here, with the full drift allowance again (stricter).
        # Keep the old 10 cm minimum look-ahead for normal moves, but never
        # check farther than a deliberately shorter fallback move can travel.
        look_ahead = max(min(0.10, distance), distance - progress)
        scale = look_ahead / distance
        step = (move[0] * scale, move[1] * scale)
        if not scan_covers_move(scan, *step, leaving=leaving) or not swept_clear(
            view[0], *start_pose, *move, start=max(0.0, progress),
            leaving=leaving,
        ):
            return 'clearance lost: obstacle or unknown coverage in the move'
        return None

    def _finish_recovery(self, moved):
        """Stop and wait. The next goal is chosen only after fresh sensing."""

        self._hold_stop()
        self.recovery_move = None
        self.emit_event('recovery_finished', f'translated {moved:.3f} m')
        need_map = moved >= SLAM_MIN_TRAVEL_M
        self.refresh_after = (
            self.get_clock().now().nanoseconds * 1e-9,
            time.monotonic(),
            need_map,
        )
        self.get_logger().info(
            'Recovery move finished; waiting for a fresh scan, live cloud, '
            'costmap, and transforms'
            + (' and a new SLAM map.' if need_map else
               ' (move was under SLAM\'s minimum travel, so /map is not required).')
        )

    def tick_recovery(self):
        x, y, yaw, move, frame, started, leaving = self.recovery_move
        max_yaw, max_across = drift_limits(leaving)
        pose = self.get_robot_pose(frame)
        view = self.local_view()
        distance = math.hypot(*move)
        progress, across = self._recovery_progress(pose, x, y, yaw, move, distance)
        yaw_drift = (
            pose is not None and angle_difference(pose[2], yaw) > max_yaw
        )
        if yaw_drift:
            across = max(across, max_across + 0.01)
        if pose is not None and progress >= distance and across <= max_across:
            self._finish_recovery(progress)
            return
        reason = self._recovery_block_reason(
            pose, view, started, progress, distance, across, move, (x, y, yaw),
            leaving,
        )
        if reason is not None:
            self.recovery_move = None
            self._stop_for(reason)
            return
        command = Twist()
        command.linear.x = RECOVERY_SPEED_MPS * move[0] / distance
        command.linear.y = RECOVERY_SPEED_MPS * move[1] / distance
        self.recovery_pub.publish(command)

    def start_maneuver(self, client, goal, name, time_allowance):
        """Send a Nav2 behavior goal; False if its server isn't up yet."""

        if not client.server_is_ready():
            self.get_logger().info(
                f'Waiting for Nav2 before: {name}...',
                throttle_duration_sec=5.0
            )
            return False

        self.maneuver_generation += 1
        generation = self.maneuver_generation
        self.maneuver_name = name
        self.maneuver_client = client
        self.maneuver_server_missing_since = None
        self.maneuver_active = True
        self.maneuver_stopping = False
        self.maneuver_sent_time = time.monotonic()
        self.maneuver_time_allowance = time_allowance

        future = client.send_goal_async(goal)
        self._track_submission(
            future, 'spin' if client is self.spin_client else 'backup'
        )
        future.add_done_callback(
            lambda f, g=generation, c=client:
            self.maneuver_response_callback(f, g, c)
        )
        return True

    def _track_submission(self, future, action_name):
        """Remember a goal Nav2 hasn't answered yet, so stop() can wait for
        it: until accepted it has no handle to cancel."""

        self.pending_submissions.append((future, action_name))
        future.add_done_callback(
            lambda f: self.pending_submissions.remove((f, action_name))
            if (f, action_name) in self.pending_submissions else None
        )

    def maneuver_response_callback(self, future, generation, client):

        try:
            goal_handle = future.result()
        except Exception as e:
            if generation == self.maneuver_generation:
                self._stop_for(f'{self.maneuver_name} failed to start: {e}')
                self.finish_maneuver()
            return

        if generation != self.maneuver_generation:
            # Accepted after it was abandoned or released: it must not run
            # alongside whatever comes next, so cancel it and keep motion
            # blocked until Nav2 confirms it ended.
            if goal_handle.accepted:
                self._adopt_orphan(goal_handle, client)
            return

        if not goal_handle.accepted:
            self._stop_for(f'Nav2 rejected the {self.maneuver_name}')
            self.finish_maneuver()
            return

        self.maneuver_handle = goal_handle
        if self.maneuver_stopping:
            # Timed out before Nav2 accepted it: cancel now, and wait below
            # for the result that confirms it ended.
            goal_handle.cancel_goal_async()
        goal_handle.get_result_async().add_done_callback(
            lambda f, g=generation: self.maneuver_result_callback(f, g)
        )

    def maneuver_result_callback(self, future, generation):

        if generation != self.maneuver_generation:
            return  # an older maneuver; the current one isn't affected

        try:
            status = future.result().status
            error = None
        except Exception as e:
            status, error = None, e

        if status not in TERMINAL_ACTION_STATUSES:
            # An error fetching the result, or a non-final status, proves
            # nothing about whether the maneuver is still moving the robot.
            # Keep it owned, cancel it, and wait for confirmation (a final
            # status, or its server gone: see _check_stopping_maneuver).
            what = f'result error: {error}' if error else f'status {status}'
            if not self.maneuver_stopping:
                self.maneuver_stopping = True
                if self.maneuver_handle is not None:
                    self.maneuver_handle.cancel_goal_async()
                self._stop_for(
                    f'{self.maneuver_name} {what}; not confirmed ended'
                )
            else:
                self.get_logger().warn(
                    f'Abandoned {self.maneuver_name}: {what}; still waiting.'
                )
            return

        if self.maneuver_stopping:
            # The stop it caused stays latched (see _consider_clearing_fault).
            self.get_logger().info(
                f'Abandoned {self.maneuver_name} has ended in Nav2 '
                f'(status {status}).'
            )
            self.finish_maneuver()
            return

        if self.maneuver_name == 'Scan' and status != 4:
            self._stop_for(f'scan maneuver ended with status {status}')
        if self.maneuver_name == RECOVERY_TURN_NAME:
            if status == 4:
                self.failed_turn_sign = 0
                self.emit_event(
                    'recovery_finished',
                    f'turned {math.degrees(self.recovery_turn_angle):.0f} deg',
                )
                # Like a translation: the next goal waits for fresh sensing.
                # A turn alone doesn't make slam_toolbox publish a new /map.
                self.refresh_after = (
                    self.get_clock().now().nanoseconds * 1e-9,
                    time.monotonic(),
                    False,
                )
            else:
                # Nav2 saw a collision ahead (or gave up): next time, try
                # the other way first.
                self.failed_turn_sign = (
                    1 if self.recovery_turn_angle > 0 else -1
                )
                self._stop_for(f'recovery turn ended with status {status}')
        if status == 4:
            self.get_logger().info(f'{self.maneuver_name} complete.')
        elif not self.recovery_fault:
            self.get_logger().warn(
                f'{self.maneuver_name} ended early with status {status}.'
            )
        self.finish_maneuver()

    def _action_ended(self, handle, client, missing_since):
        """(ended, missing_since, why) for a Nav2 action that was cancelled
        but whose result may never arrive. Ended means Nav2's goal status
        (rclpy updates the handle from the status topic) is final, or the
        action server has been gone for SERVER_GONE_SEC without a break, so
        it can't be moving the robot."""

        status = getattr(handle, 'status', None) if handle else None
        if status in TERMINAL_ACTION_STATUSES:
            return True, missing_since, f'status {status} on the status topic'
        now = time.monotonic()
        if client is not None and not client.server_is_ready():
            if missing_since is None:
                missing_since = now
            if now - missing_since >= SERVER_GONE_SEC:
                return True, missing_since, 'its Nav2 behavior server is gone'
        else:
            missing_since = None
        return False, missing_since, ''

    def _check_stopping_maneuver(self):
        """Release a cancelled maneuver once _action_ended confirms it.
        Without this, a result that never arrives (e.g. Nav2's behavior
        server crashed) would keep exploration stopped for good. The stop it
        caused stays latched either way."""

        ended, self.maneuver_server_missing_since, why = self._action_ended(
            self.maneuver_handle,
            self.maneuver_client,
            self.maneuver_server_missing_since,
        )
        if ended:
            self.get_logger().info(
                f'Abandoned {self.maneuver_name} ended ({why}).'
            )
            self.finish_maneuver()
            return
        self.get_logger().info(
            f'Waiting for Nav2 to confirm the cancelled {self.maneuver_name} '
            'has ended. If Nav2 is stuck, restart it.',
            throttle_duration_sec=10.0,
        )

    def _adopt_orphan(self, handle, client):
        """Nav2 accepted an action the explorer had already given up on.
        Cancel it and block motion until it's confirmed ended."""

        handle.cancel_goal_async()
        orphan = {'handle': handle, 'client': client, 'missing_since': None}
        self.orphans.append(orphan)
        handle.get_result_async().add_done_callback(
            lambda f, o=orphan: self._orphan_result(f, o)
        )
        reason = 'Nav2 accepted an abandoned action late'
        if self.recovery_fault:
            # Keep the existing stop's reason and requirements.
            self.recovery_move = None
            self._hold_stop()
            self.get_logger().warn(f'{reason}; cancelling it.')
        else:
            self._stop_for(reason)

    def _orphan_result(self, future, orphan):
        try:
            status = future.result().status
        except Exception:
            return  # unconfirmed: _check_unresolved keeps checking
        if status in TERMINAL_ACTION_STATUSES and orphan in self.orphans:
            self.orphans.remove(orphan)
            self.get_logger().info(
                f'Late action ended in Nav2 (status {status}).'
            )

    def _check_unresolved(self):
        """Release whatever Nav2 actions are now confirmed ended."""

        if self.maneuver_stopping:
            self._check_stopping_maneuver()
        for orphan in list(self.orphans):
            ended, orphan['missing_since'], why = self._action_ended(
                orphan['handle'], orphan['client'], orphan['missing_since']
            )
            if ended:
                self.orphans.remove(orphan)
                self.get_logger().info(f'Late action ended ({why}).')

    def finish_maneuver(self):
        # Retire this maneuver's number: any reply that still arrives for it
        # is stale (a late acceptance becomes an orphan and is cancelled).
        self.maneuver_generation += 1

        if self.maneuver_name == 'Scan' and not self.recovery_fault:
            # A full turn is enough heading change for slam_toolbox to insert
            # scans, so the next goal waits for a new /map as well.
            self.refresh_after = (
                self.get_clock().now().nanoseconds * 1e-9,
                time.monotonic(),
                True,
            )

        self.maneuver_handle = None
        self.maneuver_client = None
        self.maneuver_server_missing_since = None
        self.maneuver_active = False
        self.maneuver_stopping = False
        self.maneuver_sent_time = None

    # ==========================================================
    # OBSTACLE AHEAD
    # ==========================================================

    def watch_ahead(self):
        """Back away whenever an obstacle is right in front of the nose.

        While driving to a goal, the goal is given up first once the obstacle
        has been there blocked_timeout seconds (0 = at once). While idle
        (between goals, before a scan), the robot backs up straight away.
        During a turn or back-up nothing is done: Nav2's behaviors check for
        collisions themselves.
        """

        try:
            # A latched stop, or a Nav2 action not yet confirmed ended, takes
            # precedence over a recovery translation.
            if self.recovery_fault or self.orphans or self.maneuver_stopping:
                self.recovery_move = None
                self._consider_clearing_fault()
                return
            if self.recovery_move is not None:
                self.tick_recovery()
                return
            if self.refresh_after is not None:
                # Keep publishing zero: twist_mux would otherwise fall through
                # to Nav2 half a second after the last command.
                self._hold_stop()
                return
            if self.maneuver_active:
                self.blocked_since = None
                return
            if self.yield_for_people():
                return

            view = self.local_view()
            if view is None:
                return
            grid, x, y, yaw, _ = view
            forward = self._forward_speed(grid.header.frame_id, x, y, yaw)

            gap = free_distance_ahead(
                grid, x, y, yaw, self.obstacle_stop_distance
            )
            if gap is None:
                self.blocked_since = None
                return

            if not self.goal_active:
                self.back_away_while_idle(gap)
                return

            if self.current_goal_handle is None:
                return  # goal not accepted yet; nothing to cancel

            distance = min(max(float(self.backup_distance), 0.05), 0.5)
            if (
                forward is not None and forward < FORWARD_MOTION_MPS
                and translation_open(grid, x, y, yaw, distance)
            ):
                # Turning in place (or standing still): the nose swinging
                # past a wall isn't a drive into it, and Nav2's controller
                # checks the footprint for that turn itself. Abandoning the
                # goal here left Oak crosswise in narrow passages, with no
                # safe way back or sideways (found in go2_sim). But only
                # while a recovery translation is still open: Nav2 checks
                # the bare footprint, no margin, so it would turn Oak right
                # up against the wall, where no turn, back-up or side step
                # is allowed any more (found in go2_sim).
                self.blocked_since = None
                return

            now = time.monotonic()
            if self.blocked_since is None:
                self.blocked_since = now
                return
            if now - self.blocked_since < self.blocked_timeout:
                return

            map_pose = self.get_robot_pose(self.map_frame)
            self.blocked_heading = map_pose[2] if map_pose else None
            self.blocked_time = now
            self.blocked_since = None

            self.get_logger().warn(
                f'Obstacle {gap:.2f} m ahead for {self.blocked_timeout:.1f} s; '
                'abandoning this goal and heading somewhere open.'
            )
            # The result callback then blacklists the goal and frees the loop.
            # Dropping the handle stops the next ticks (before Nav2 confirms)
            # from cancelling and asking to escape again.
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            self.backup_pending = True
            self.request_escape('obstacle ahead')

        except Exception as e:
            # Same reason as exploration_loop: never let a callback error
            # crash the node while a goal is in flight.
            if self.recovery_move is not None:
                self.recovery_move = None
                self._stop_for(f'obstacle watch error: {e}')
            else:
                self.get_logger().error(f'Obstacle watch error: {e}')

    def yield_for_people(self):
        """While driving to a goal, hold still when a person, their next
        steps or a group is just ahead, so they can pass. True while holding.

        The wait ends after YIELD_MAX_SEC, under Nav2's 10 s progress check,
        and is not repeated until the way ahead has been clear once: someone
        standing still in a hallway must not stop exploring. Nav2's own
        obstacle avoidance still applies after that."""

        if not (self.yield_to_people and self.goal_active):
            self.yield_since, self.yield_spent = None, False
            return False
        pose = self.get_robot_pose(self.map_frame)
        if pose is None:
            return False
        x, y, yaw = pose
        ahead = line_crosses(
            x, y, x + YIELD_LOOKAHEAD_M * math.cos(yaw), y + YIELD_LOOKAHEAD_M * math.sin(yaw),
            self.people_zones(),
        )
        if ahead is None:
            self.yield_since, self.yield_spent = None, False
            return False
        if self.yield_spent:
            return False
        now = time.monotonic()
        if self.yield_since is None:
            self.yield_since = now
            self.get_logger().info(f'A {ahead.kind} is just ahead; waiting for them to pass.')
        if now - self.yield_since >= YIELD_MAX_SEC:
            self.yield_spent = True
            self.get_logger().info('Still in the way; carrying on around them.')
            return False
        self._hold_stop()
        return True

    def _forward_speed(self, frame, x, y, yaw):
        """Speed toward the nose since the previous watch_ahead tick, in m/s,
        or None when that can't be told (first tick, frame change, or a gap
        of ticks). None keeps the old behavior: an obstacle ahead counts."""

        now = time.monotonic()
        last = self.last_watch_pose
        self.last_watch_pose = (frame, x, y, yaw, now)
        if last is None or last[0] != frame:
            return None
        dt = now - last[4]
        if dt <= 0.0 or dt > 3.0 * WATCH_PERIOD_SEC:
            return None
        return (
            (x - last[1]) * math.cos(last[3])
            + (y - last[2]) * math.sin(last[3])
        ) / dt

    def back_away_while_idle(self, gap):
        """Something is right in front and the robot isn't driving anywhere:
        back up now, unless a back-up was just tried (the space behind may be
        blocked too, and Nav2 would refuse every attempt)."""

        if self.backup_pending or (
            self.last_backup_time is not None
            and time.monotonic() - self.last_backup_time < BACKUP_RETRY_SEC
        ):
            return

        self.get_logger().warn(f'Obstacle {gap:.2f} m ahead; backing away.')
        self.start_backup()

    def recently_blocked(self):

        return (
            self.blocked_time is not None
            and time.monotonic() - self.blocked_time < BLOCKED_MEMORY_SEC
        )

    # ==========================================================
    # GETTING UNSTUCK: NEAREST OPEN SPACE
    # ==========================================================

    def request_escape(self, reason):
        """Queue a drive to the nearest open space (run by the loop once the
        robot is free to move), unless one was tried just now."""

        if (
            self.last_escape_time is not None
            and time.monotonic() - self.last_escape_time < ESCAPE_COOLDOWN_SEC
        ):
            self.get_logger().info(
                f'Stuck ({reason}), but just tried open space; not again yet.'
            )
            return

        self.get_logger().warn(f'Stuck ({reason}): heading for open space.')
        self.log_event(f'stuck: {reason}')
        self.escape_pending = True

    def start_escape(self):
        """Send a goal to the nearest roomy spot on the SLAM map. A scan
        follows on arrival (scan_after_goal), then exploring resumes."""

        self.escape_pending = False
        self.last_escape_time = time.monotonic()

        position = self.get_robot_position()
        if self.map is None or position is None:
            self.get_logger().warn('Escape skipped: no map or position yet.')
            return

        resolution = self.map.info.resolution
        start = tuple(int(math.floor(v)) for v in self.world_to_grid(*position))
        found = find_open_space(
            self.map.data,
            self.map.info.width,
            self.map.info.height,
            start,
            clearance=int(math.ceil(self.open_space_clearance / resolution)),
            max_radius=int(OPEN_SPACE_SEARCH_RADIUS_M / resolution),
            min_move=int(OPEN_SPACE_MIN_MOVE_M / resolution),
        )

        if found is None:
            self.get_logger().warn(
                'No roomier spot within '
                f'{OPEN_SPACE_SEARCH_RADIUS_M:.0f} m; carrying on exploring.'
            )
            return

        gx, gy, room = found
        x, y = self.grid_to_world(gx, gy)
        self.get_logger().info(
            f'Open space at x={x:.2f}, y={y:.2f} '
            f'({room * resolution:.2f} m from the nearest wall).'
        )
        self.send_goal(x, y, is_escape=True)

    # ==========================================================
    # EXPLORATION LOOP
    # ==========================================================

    def exploration_loop(self):
        try:
            self._exploration_loop_impl()
        except Exception as e:
            # A callback exception here would otherwise propagate out of
            # rclpy.spin(), crashing the node before stop() gets a chance
            # to cancel any in-flight Nav2 goal — the robot would then
            # keep walking toward the last frontier with nothing watching
            # it. Log and skip this cycle instead; the next timer tick
            # tries again.
            self.get_logger().error(f'Exploration loop error: {e}')

    def _exploration_loop_impl(self):

        if self.recovery_move is not None or self.recovery_fault:
            return
        if self.refresh_after is not None:
            stop_ros, started, need_map = self.refresh_after
            ready, reason = self._refresh_check(stop_ros, need_map)
            if ready:
                self.refresh_after = None
                self.get_logger().info(
                    'Fresh scan, live cloud, costmap, and transforms received. '
                    'The accumulated 3D map was not reset; Nav2 clears '
                    'disappeared obstacles by raytracing /scan.'
                )
            else:
                if time.monotonic() - started > REFRESH_TIMEOUT_SEC:
                    self.refresh_after = None
                    # Keep the same requirements on the latch: it may only
                    # release once this sensing is fresh after stop_ros.
                    self._stop_for(reason, refresh=(stop_ros, need_map))
                return

        # Maneuvers only need odometry, not the map, so the startup scan runs
        # before SLAM has a map: turning in place is what gives it one.
        if self.maneuver_active:
            if self.maneuver_stopping:
                self._check_stopping_maneuver()
            elif (
                self.maneuver_sent_time is not None
                and time.monotonic() - self.maneuver_sent_time
                > self.maneuver_time_allowance + 10.0
            ):
                self.get_logger().warn(
                    f'No {self.maneuver_name} result; stopping exploration.'
                )
                # Cancel it in Nav2, and stay "active" until Nav2 confirms it
                # ended: freeing the state now would let the next move start
                # while this one may still be turning or backing up. (If it
                # hasn't been accepted yet, the response callback cancels it.)
                self.maneuver_stopping = True
                if self.maneuver_handle is not None:
                    self.maneuver_handle.cancel_goal_async()
                self._stop_for(f'no {self.maneuver_name} result')
            return

        if not self.goal_active:
            # Back away first, then head for open space, then look around.
            if self.backup_pending:
                self.start_backup()
                return
            if self.scan_pending:
                self.start_scan()
                return
            if self.escape_pending:
                self.start_escape()
                if self.goal_active:
                    return
            if self.scan_pending:
                self.start_scan()
                return

        # Don't do anything until SLAM has produced a map.
        if self.map is None:
            self.get_logger().info(
                'Waiting for map...',
                throttle_duration_sec=5.0
            )
            return

        # If a goal was accepted but Nav2 never reported a result (e.g. the
        # action server dropped the connection post-acceptance), neither
        # done-callback will ever fire. Force a reset so exploration doesn't
        # stall forever.
        if (
            self.goal_active
            and self.goal_sent_time is not None
            and time.monotonic() - self.goal_sent_time > self.goal_timeout_sec
        ):
            self.get_logger().warn(
                'No Nav2 result after '
                f'{self.goal_timeout_sec:.0f}s, treating goal as stalled.'
            )
            # Cancel it in Nav2 too: just forgetting it would leave Nav2
            # driving there while we start a turn, back-up or new goal.
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self.record_goal_failure()
            self.current_goal_handle = None
            self.goal_active = False
            self.goal_sent_time = None

        # Don't issue multiple goals simultaneously.
        if self.goal_active:
            return

        robot_position = self.get_robot_position()

        if robot_position is None:

            # Normal for a few seconds at startup: SLAM only publishes the
            # map frame once it has matched enough scans.
            self.get_logger().info(
                f'Waiting for robot position '
                f'({self.map_frame} -> {self.robot_frame} from SLAM)...',
                throttle_duration_sec=5.0
            )

            return

        robot_x, robot_y = robot_position

        frontiers = self.find_frontiers()

        # A goal was reached: once its scan is done and SLAM has had time to
        # publish what it saw, check the target frontier actually shrank.
        if self.frontier_to_verify is not None and not self.scan_pending:
            now = time.monotonic()
            if self.verify_not_before is None:
                self.verify_not_before = now + MAP_SETTLE_SEC
                return
            if now < self.verify_not_before:
                return
            self.verify_frontier_shrank(frontiers)

        if not frontiers:

            if not self.exploration_complete_logged:
                self.get_logger().info(
                    'No frontiers found. Exploration complete.'
                )
                self.exploration_complete_logged = True
                if not self.exploration_terminal_emitted:
                    self.emit_event(
                        'exploration_completed',
                        'No frontiers found. Exploration complete.',
                    )
                    self.exploration_terminal_emitted = True

            return

        self.exploration_complete_logged = False

        self.get_logger().info(
            f'Found {len(frontiers)} frontier regions.'
        )

        selected = self.choose_frontier(
            frontiers,
            robot_x,
            robot_y
        )

        if selected is WAITING_FOR_VLM:
            return

        if selected is None:
            self.get_logger().info(
                'No usable goal: every frontier is too close to a wall or '
                'failed recently. Waiting for the map to change or for '
                f'failed spots to expire ({self.failed_goal_cooldown_sec:.0f} s).',
                throttle_duration_sec=10.0
            )
            self.no_goal_cycles += 1
            if self.no_goal_cycles >= STUCK_NO_GOAL_CYCLES:
                self.no_goal_cycles = 0
                self.request_escape('no usable goal')
            return

        self.no_goal_cycles = 0

        score, x, y, size, frontier = selected

        self.get_logger().info(
            f'Selected frontier: '
            f'x={x:.2f}, '
            f'y={y:.2f}, '
            f'size={size}'
        )

        if self.send_goal(x, y):
            # Sized in keys, the unit verify_frontier_shrank measures in.
            keys = self.frontier_keys(frontier)
            self.goal_frontier = (keys, len(keys))
            self.log_goal_sent(robot_x, robot_y, x, y, size, score)

    def log_goal_sent(self, robot_x, robot_y, x, y, size, score):
        """Note the new goal in the navigation log, with the camera view toward
        it if the VLM has been keeping views. A logging failure must not stop
        exploring, with a goal already on its way."""

        if self.nav_log is None:
            return
        try:
            jpeg = None
            if self.vlm_client is not None:
                frame = self.views.view_toward(
                    robot_x, robot_y, math.atan2(y - robot_y, x - robot_x),
                    time.monotonic(), VIEW_MAX_OFFSET_M, VIEW_MAX_ANGLE_RAD, VIEW_MAX_AGE_SEC,
                )
                if frame is not None:
                    jpeg = encode_jpeg(frame)
            self.nav_log.goal_sent(
                (robot_x, robot_y), (x, y), known_cell_count(self.map.data), size, score, jpeg
            )
        except Exception as e:
            self.get_logger().warn(f'Navigation log error: {e}')

    def log_event(self, what):
        if self.nav_log is not None:
            self.nav_log.event(what)

    def log_goal_ended(self, result):
        if self.nav_log is not None and not self.goal_is_escape:
            self.nav_log.goal_ended(result)

    def stop(self):
        # Nothing new may start moving the robot from here on.
        self.shutting_down = True
        self.timer.cancel()
        self.watch_timer.cancel()
        self.recovery_move = None
        self.recovery_pub.publish(Twist())
        if not self.exploration_terminal_emitted:
            self.emit_goal_outcome('goal_cancelled', 'explorer stopped')
            self.emit_event('exploration_stopped', 'explorer stopped')
            self.exploration_terminal_emitted = True

        # The goal Oak is walking to goes first: the waits below can take
        # several seconds when Nav2 is slow, and the RViz panel force-kills
        # the explorer if it takes too long. rclpy.spin() has already
        # returned, so spin briefly to actually send the cancel.
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling active navigation goal.')
            future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.current_goal_handle = None

        # A goal Nav2 hasn't answered yet has no handle to cancel. Wait (and
        # spin, so the answer is processed) until it's accepted, which sets
        # the handles cancelled below, or rejected.
        deadline = time.monotonic() + SHUTDOWN_ACCEPT_WAIT_SEC
        for future, action_name in list(self.pending_submissions):
            remaining = deadline - time.monotonic()
            if remaining > 0:
                rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
            if not future.done():
                self._cancel_all_goals(action_name)

        # A goal accepted during the wait above.
        if self.current_goal_handle is not None:
            future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        for orphan in self.orphans:
            future = orphan['handle'].cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if self.maneuver_handle is not None:
            self.get_logger().info(f'Cancelling the {self.maneuver_name}.')
            future = self.maneuver_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        # Write out the last goal, which is otherwise only written when the
        # next one is sent.
        if self.nav_log is not None and self.map is not None:
            self.nav_log.finish(known_cell_count(self.map.data))

    def _cancel_all_goals(self, action_name):
        """Last resort on shutdown: Nav2 never answered one of our goals, so
        it may still accept and run it with nobody tracking it. Cancel every
        goal on that action (a zero goal ID and stamp means all), even other
        nodes' goals: stopping the robot is the safe outcome.

        Known limit: a goal Nav2 receives only after this cancel is not
        covered. That needs Nav2 to be failing already (no answer in
        SHUTDOWN_ACCEPT_WAIT_SEC); the keyboard or joystick still override it.
        """

        self.get_logger().error(
            f'Nav2 never answered a {action_name} goal; cancelling all '
            f'{action_name} goals so the robot stops.'
        )
        client = self.create_client(
            CancelGoal, f'/{action_name}/_action/cancel_goal'
        )
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        else:
            self.get_logger().error(
                f'{action_name} cancel service unavailable. Stop the robot by '
                f'hand: ros2 service call /{action_name}/_action/cancel_goal '
                'action_msgs/srv/CancelGoal "{}"'
            )
        self.destroy_client(client)


# How long stop() waits for Nav2 to answer goals it hasn't accepted yet.
SHUTDOWN_ACCEPT_WAIT_SEC = 2.0

PR_SET_PDEATHSIG = 1  # from <linux/prctl.h>

# Final action_msgs/GoalStatus values: SUCCEEDED, CANCELED, ABORTED. Only
# these prove an abandoned maneuver has ended.
TERMINAL_ACTION_STATUSES = (4, 5, 6)

# How long a stopping maneuver's server must be missing without a break
# before it counts as gone rather than a discovery blip.
SERVER_GONE_SEC = 5.0

# Spacing of the extra goal candidates taken along a long frontier.
FRONTIER_SAMPLE_SPACING_M = 1.0

# A frontier counts as one given up on (a visit didn't shrink it) when at
# least this share of its cells belong to it.
DEAD_FRONTIER_OVERLAP = 0.5

# Wait after the scan at a goal before judging whether the target frontier
# shrank: SLAM publishes /map every 1 s (map_update_interval).
MAP_SETTLE_SEC = 1.5

# How often watch_ahead looks at the local costmap while driving.
WATCH_PERIOD_SEC = 0.25

# After a goal is abandoned for an obstacle ahead: how long to steer goal
# choice away from that heading, and how wide a cone around it to avoid.
BLOCKED_MEMORY_SEC = 30.0
BLOCKED_CONE_RAD = math.radians(45.0)

# How far toward a goal to check the straight line in the local costmap;
# beyond this the costmap (20 x 20 m) has little to say anyway.
LINE_CHECK_M = 3.0

# Backing away from an obstacle: slow, and with a short time limit.
BACKUP_SPEED_MPS = 0.15
BACKUP_TIME_ALLOWANCE_SEC = 10.0

# Least time between automatic back-ups when idle, so a robot boxed in front
# and behind doesn't retry (and get refused by Nav2) every quarter second.
BACKUP_RETRY_SEC = 5.0

# Turning in place when no translation is safe (_try_recovery_turn): the
# maneuver's name, Nav2's time limit for it, the least time between two
# such turns, and how much room past the stop distance the new heading must
# have on the scan, so the turn doesn't end facing another close wall.
RECOVERY_TURN_NAME = 'Recovery turn'
RECOVERY_TURN_TIME_SEC = 15.0
RECOVERY_TURN_RETRY_SEC = 10.0
RECOVERY_TURN_ROOM_M = 0.2

# Slower than this toward the nose counts as not driving forward: an
# obstacle ahead then doesn't abandon the goal (see watch_ahead).
FORWARD_MOTION_MPS = 0.03

# Getting unstuck: how far from the robot to look for open space, the least
# it must move to count, and the pause between escapes so a failed escape
# can't immediately trigger another one.
OPEN_SPACE_SEARCH_RADIUS_M = 4.0
OPEN_SPACE_MIN_MOVE_M = 0.5
ESCAPE_COOLDOWN_SEC = 30.0

# "No usable goal" this many loops in a row counts as stuck.
STUCK_NO_GOAL_CYCLES = 2

# object_detection publishes /people in this frame.
PEOPLE_FRAME = 'odom'
# /people older than this is ignored: the people have moved on.
PEOPLE_MAX_AGE_SEC = 2.0
# How far ahead a person makes the robot wait, and for how long at most.
# The wait must stay under Nav2's progress checker (10 s,
# movement_time_allowance in nav2_params.yaml) or Nav2 aborts the goal.
YIELD_LOOKAHEAD_M = 1.5
YIELD_MAX_SEC = 6.0

# choose_frontier's answer while active VLM mode waits for a ranking.
WAITING_FOR_VLM = object()

# Camera views kept for the VLM: one per 45 degrees of heading, a few per
# second at most. A view counts toward a goal if it was taken within
# VIEW_MAX_OFFSET_M of where the robot is now, in the last VIEW_MAX_AGE_SEC,
# and faces within VIEW_MAX_ANGLE_RAD of the goal (inside a ~90 degree
# camera view).
VIEW_BINS = 8
VIEW_PERIOD_SEC = 0.3
VIEW_MAX_OFFSET_M = 1.0
VIEW_MAX_AGE_SEC = 120.0
VIEW_MAX_ANGLE_RAD = math.radians(35.0)

# A VLM ranking applies to goals within VLM_MATCH_RADIUS_M of the ones it
# was asked about, for VLM_RESULT_MAX_AGE_SEC. After a failed request, the
# map score is used alone for VLM_RETRY_SEC before asking again.
VLM_MATCH_RADIUS_M = 0.5
VLM_RESULT_MAX_AGE_SEC = 30.0
VLM_RETRY_SEC = 30.0

# Images sent to the VLM are shrunk to this width: plenty for "doorway or
# wall", and it keeps each request small over the SSH tunnel.
VLM_IMAGE_WIDTH = 448
VLM_JPEG_QUALITY = 80


def encode_jpeg(msg):
    """A camera Image message as JPEG bytes, shrunk to VLM_IMAGE_WIDTH.

    Imported here so the explorer runs without OpenCV when vlm_mode is off.
    Runs on the VLM client's thread, not the explorer's.
    """
    import cv2
    from cv_bridge import CvBridge

    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
    height, width = image.shape[:2]
    if width > VLM_IMAGE_WIDTH:
        image = cv2.resize(image, (VLM_IMAGE_WIDTH, int(height * VLM_IMAGE_WIDTH / width)))
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, VLM_JPEG_QUALITY])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return jpeg.tobytes()


def stop_when_parent_dies():
    """Get SIGINT when the launching process exits, so the explorer shuts
    down cleanly (cancelling its Nav2 goal) instead of living on as an orphan.

    The RViz AutoMapper panel starts the explorer and stops it on Stop, but if
    RViz crashes or is killed, nothing stops it, and each orphan keeps sending
    Nav2 goals that fight the others.
    """
    try:
        libc = ctypes.CDLL('libc.so.6', use_errno=True)
        libc.prctl(PR_SET_PDEATHSIG, signal.SIGINT)
    except OSError:
        pass  # Not Linux: run without the safeguard.


def main(args=None):

    stop_when_parent_dies()

    # Keep the ROS context alive while Ctrl+C runs stop(). The default rclpy
    # signal handler shuts the context down first, which prevents the final
    # zero velocity, active Nav2 goal cancellation, and exploration_stopped
    # event from being published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = FrontierExplorer()
    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        raise KeyboardInterrupt

    previous_sigint_handler = signal.signal(signal.SIGINT, request_shutdown)
    # SIGTERM (e.g. docker stop) is treated the same way.
    previous_sigterm_handler = signal.signal(signal.SIGTERM, request_shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # SIGINT can interrupt rclpy while it is converting a subscription
        # message, in which case the extension raises RuntimeError instead of
        # delivering KeyboardInterrupt. Only suppress that shutdown race;
        # unrelated runtime failures must still be visible.
        if not shutdown_requested:
            raise
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint_handler)
        signal.signal(signal.SIGTERM, previous_sigterm_handler)


if __name__ == '__main__':
    main()
