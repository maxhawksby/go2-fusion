#!/usr/bin/env python3
"""Frontier explorer: autonomous map building for the Go2.

Python / ROS 2 Humble port of frontier_explorer.cpp from
github.com/Manas-arumalla/autonomous-go2-inspection (C++, Jazzy).

Reads the SLAM occupancy grid (/map), finds frontiers (free cells next to
unknown ones), groups them into clusters, picks the best one by information
gain (big frontier, penalised by distance, explore_lite style) and drives
there through Nav2's NavigateToPose action.

Recovery belongs to Nav2: this node never publishes a velocity. If the robot
gets near a goal but Nav2 doesn't finish, it moves on to the next frontier;
if it stops making progress, the goal is blacklisted and another frontier is
chosen, leaving backing up and spinning to Nav2's own collision-checked
behaviors. Blacklist entries expire, so no area is given up on for good.

State machine: IDLE -> EXPLORING -> NAVIGATING -> (EXPLORING | DONE).
SCANNING is a 360 degree turn requested through ~/scan (the RViz AutoMapper
panel's Scan button); it returns to the state it interrupted.

Uses only /map, TF (map -> base_link) and Nav2, so it runs unchanged on the
real robot and in the Gazebo Fortress simulator (go2_sim).
"""

import ctypes
import math
import signal
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from go2_interfaces.msg import AutoMapperEvent
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker, MarkerArray

IDLE = 'IDLE'
EXPLORING = 'EXPLORING'
NAVIGATING = 'NAVIGATING'
SCANNING = 'SCANNING'
DONE = 'DONE'

UNKNOWN = -1


# ==========================================================
# GRID HELPERS (no ROS, testable with a hand-made grid)
# ==========================================================

def box_any(mask, radius):
    """True where `mask` has a True cell within `radius` cells (square
    window; cells past the grid edge count as False)."""

    height, width = mask.shape
    integral = np.zeros((height + 1, width + 1), dtype=np.int32)
    integral[1:, 1:] = mask.astype(np.int32).cumsum(0).cumsum(1)
    rows = np.arange(height)
    cols = np.arange(width)
    y0 = np.clip(rows - radius, 0, height)
    y1 = np.clip(rows + radius + 1, 0, height)
    x0 = np.clip(cols - radius, 0, width)
    x1 = np.clip(cols + radius + 1, 0, width)
    total = (
        integral[np.ix_(y1, x1)] - integral[np.ix_(y0, x1)]
        - integral[np.ix_(y1, x0)] + integral[np.ix_(y0, x0)]
    )
    return total > 0


class GridView:
    """One /map snapshot with the masks the explorer needs.

    free:  0 <= value <= free_threshold
    clear: no wall (value >= occupied_threshold) within goal_clearance, so
           Nav2 can fit the robot's footprint there. Unknown cells don't
           count as walls: a frontier borders unknown space by definition.
    """

    def __init__(self, msg, free_threshold, occupied_threshold, goal_clearance):
        info = msg.info
        self.width = info.width
        self.height = info.height
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.grid = np.asarray(msg.data, dtype=np.int16)[
            :self.width * self.height].reshape(self.height, self.width)
        self.free = (self.grid >= 0) & (self.grid <= free_threshold)
        self.unknown = self.grid == UNKNOWN
        clearance_cells = max(1, int(round(goal_clearance / self.resolution)))
        self.clear = ~box_any(self.grid >= occupied_threshold, clearance_cells)

    def to_world(self, cx, cy):
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def navigable(self, wx, wy):
        """Free and clear of walls: a goal Nav2 can actually reach."""

        cx = int(math.floor((wx - self.origin_x) / self.resolution))
        cy = int(math.floor((wy - self.origin_y) / self.resolution))
        if not (0 <= cx < self.width and 0 <= cy < self.height):
            return False
        return bool(self.free[cy, cx] and self.clear[cy, cx])

    def frontier_mask(self):
        """Free cells with an unknown 8-neighbor (grid border excluded)."""

        mask = np.zeros_like(self.free)
        if self.width < 3 or self.height < 3:
            return mask
        h, w = self.height, self.width
        near_unknown = np.zeros((h - 2, w - 2), dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                near_unknown |= self.unknown[1 + dy:h - 1 + dy, 1 + dx:w - 1 + dx]
        mask[1:-1, 1:-1] = self.free[1:-1, 1:-1] & near_unknown
        return mask


def find_frontiers(view, robot_x, robot_y, min_frontier_size,
                   min_goal_distance, gain_scale, potential_scale,
                   is_blacklisted=lambda x, y: False):
    """Frontier clusters as dicts (x, y, size, dist, score), plus the number
    of frontier cells.

    Each cluster (8-connected) is aimed at its cell NEAREST the robot that is
    at least min_goal_distance away and clear of walls, not its centroid: a
    frontier ring around the robot would have its centroid on the robot.
    score = gain_scale * size - potential_scale * distance (bigger and closer
    is better).
    """

    mask = view.frontier_mask()
    flat = mask.ravel()
    cells = np.flatnonzero(flat)
    if cells.size == 0:
        return [], 0

    width = view.width
    cx = cells % width
    cy = cells // width
    wx = view.origin_x + (cx + 0.5) * view.resolution
    wy = view.origin_y + (cy + 0.5) * view.resolution
    dist = np.hypot(wx - robot_x, wy - robot_y)
    usable = (dist >= min_goal_distance) & view.clear.ravel()[cells]

    position = {int(c): i for i, c in enumerate(cells)}
    visited = np.zeros(cells.size, dtype=bool)
    height = view.height
    out = []

    for start in range(cells.size):
        if visited[start]:
            continue
        visited[start] = True
        queue = deque([start])
        size = 0
        best = -1
        while queue:
            i = queue.popleft()
            size += 1
            if usable[i] and (best < 0 or dist[i] < dist[best]):
                best = i
            x, y = int(cx[i]), int(cy[i])
            for dy in (-1, 0, 1):
                ny = y + dy
                if ny < 0 or ny >= height:
                    continue
                for dx in (-1, 0, 1):
                    nx = x + dx
                    if nx < 0 or nx >= width:
                        continue
                    j = position.get(ny * width + nx)
                    if j is not None and not visited[j]:
                        visited[j] = True
                        queue.append(j)

        if size < min_frontier_size or best < 0:
            continue
        goal_x, goal_y = float(wx[best]), float(wy[best])
        if is_blacklisted(goal_x, goal_y):
            continue
        d = float(dist[best])
        out.append({
            'x': goal_x, 'y': goal_y, 'size': size, 'dist': d,
            'score': gain_scale * size - potential_scale * d,
        })

    return out, int(cells.size)


def clamp_goal(view, goal_x, goal_y, robot_x, robot_y, max_goal_distance,
               min_goal_distance, is_blacklisted=lambda x, y: False):
    """Short-goal stepping: a frontier farther than max_goal_distance is
    replaced by the farthest navigable, non-blacklisted point on the line to
    it within that distance. The next tick finds frontiers again and steps
    again. If no step is navigable, the frontier itself is kept (Nav2 tries
    it; a real failure blacklists it), so exploring never freezes."""

    d = math.hypot(goal_x - robot_x, goal_y - robot_y)
    if d <= max_goal_distance:
        return goal_x, goal_y
    if view.resolution <= 0.0 or max_goal_distance < min_goal_distance:
        return goal_x, goal_y
    ux, uy = (goal_x - robot_x) / d, (goal_y - robot_y) / d
    s = max_goal_distance
    while s >= min_goal_distance:
        tx, ty = robot_x + ux * s, robot_y + uy * s
        if view.navigable(tx, ty) and not is_blacklisted(tx, ty):
            return tx, ty
        s -= view.resolution
    return goal_x, goal_y


# ==========================================================
# NODE
# ==========================================================

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        p = self.declare_parameter
        self.min_frontier_size = p('min_frontier_size', 8).value
        # More than Nav2's goal tolerance (0.25 m), so every goal needs real travel.
        self.min_goal_distance = p('min_goal_distance', 0.8).value
        self.free_threshold = p('free_threshold', 25).value
        self.occupied_threshold = p('occupied_threshold', 65).value
        self.blacklist_radius = p('blacklist_radius', 0.5).value
        # Goals at least this far from any wall. About the global costmap's
        # inflation_radius (0.55 m in go2_robot_sdk/config/nav2_params.yaml)
        # and more than the footprint's reach (0.45 m to the rear), else the
        # goal sits in the inflation and Nav2 can't place the body there.
        self.goal_clearance = p('goal_clearance', 0.55).value
        self.goal_timeout = p('goal_timeout', 60.0).value
        self.planning_period = p('planning_period', 2.0).value
        # Near a frontier the lidar already sees past it, so "close enough"
        # for arrival_patience seconds counts as arrived (stops circling).
        self.arrival_radius = p('arrival_radius', 0.55).value
        self.arrival_patience = p('arrival_patience', 3.0).value
        # Moving progress_dist meters counts as progress; none for
        # progress_timeout seconds means stuck. Long enough for Nav2's own
        # recovery (clear costmap, spin, back up) to run first.
        self.progress_dist = p('progress_dist', 0.25).value
        self.progress_timeout = p('progress_timeout', 18.0).value
        self.gain_scale = p('gain_scale', 1.0).value
        self.potential_scale = p('potential_scale', 3.0).value
        self.blacklist_ttl = p('blacklist_ttl', 30.0).value
        self.max_goal_distance = p('max_goal_distance', 3.0).value
        # Don't quit at the first empty cycle: if frontier cells remain but
        # all were blacklisted, clear the blacklist and retry (at most
        # max_clear_retries times between map growth), and require
        # done_confirm empty cycles in a row before declaring DONE.
        self.done_confirm = p('done_confirm', 3).value
        self.max_clear_retries = p('max_clear_retries', 3).value
        autostart = p('autostart', True).value
        self.robot_frame = p('robot_base_frame', 'base_link').value
        self.map_frame = p('map_frame', 'map').value
        self.scan_yaw = p('scan_yaw', 2.0 * math.pi).value
        self.scan_time_allowance = p('scan_time_allowance', 30.0).value

        # State
        self.state = IDLE
        self.map = None
        self.blacklist = []  # (x, y, ROS time in ns)
        self.last_frontier_cells = 0
        self.cur_free_cells = 0
        self.free_high_water = 0
        self.empty_cycles = 0
        self.clears_done = 0
        self.cur_goal = (0.0, 0.0)
        self.goal_sent_time = None
        self.near_goal = False
        self.near_since = None
        self.last_progress_pos = (0.0, 0.0)
        self.last_progress_time = None
        self.goal_handle = None
        # Every goal gets a number; a Nav2 reply for an older one (e.g. the
        # result of a goal that was cancelled to advance) is ignored.
        self.goal_sequence = 0
        self.active_goal_sequence = 0
        self.goal_outcome_emitted = True
        self.scan_handle = None
        self.scan_generation = 0
        self.resume_state = EXPLORING

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /map is latched (transient local): a volatile subscription would
        # miss the grid published before this node started.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.markers_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 1)
        # Measurement-only events for go2_scoreboard and go2_sim's trial
        # runner. They never change what the explorer does.
        event_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.event_pub = self.create_publisher(
            AutoMapperEvent, '/auto_mapper/events', event_qos)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        self.start_srv = self.create_service(Empty, '/explore/start', self.handle_start)
        self.stop_srv = self.create_service(Empty, '/explore/stop', self.handle_stop)
        # /frontier_explorer/scan: RViz AutoMapper panel's Scan button.
        self.scan_srv = self.create_service(Trigger, '~/scan', self.handle_scan)

        # Wall-clock timer like the C++ create_wall_timer, so planning keeps
        # its pace even if simulated time runs slow.
        self.timer = self.create_timer(
            float(self.planning_period), self.tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.state = EXPLORING if autostart else IDLE
        self.get_logger().info(f'frontier_explorer ready (autostart={autostart})')
        if autostart:
            self.emit_event('exploration_started', detail='frontier explorer started')

    # ==========================================================
    # EVENTS
    # ==========================================================

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
            # explorer's behavior.
            self.get_logger().warn(
                f'Could not publish automapper event {event_type}: {error}',
                throttle_duration_sec=10.0,
            )

    def emit_goal_outcome(self, event_type, detail=''):
        """At most one terminal outcome per goal."""
        if self.goal_outcome_emitted or self.active_goal_sequence == 0:
            return
        self.emit_event(event_type, detail, *self.cur_goal)
        self.goal_outcome_emitted = True

    # ==========================================================
    # INPUTS
    # ==========================================================

    def map_callback(self, msg):
        info = msg.info
        if (info.width == 0 or info.height == 0 or info.resolution <= 0.0
                or len(msg.data) < info.width * info.height):
            self.get_logger().warn(
                f'Ignoring occupancy grid: {info.width}x{info.height}, '
                f'resolution={info.resolution}, {len(msg.data)} cells',
                throttle_duration_sec=5.0)
            return
        self.map = msg

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'no {self.map_frame}->{self.robot_frame} TF yet: {e}',
                throttle_duration_sec=5.0)
            return None
        return t.transform.translation.x, t.transform.translation.y

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def seconds_since(self, stamp_ns):
        return (self.now_ns() - stamp_ns) * 1e-9

    # ==========================================================
    # BLACKLIST (entries expire after blacklist_ttl)
    # ==========================================================

    def add_blacklist(self, x, y):
        self.blacklist.append((x, y, self.now_ns()))

    def is_blacklisted(self, x, y):
        for bx, by, t in self.blacklist:
            if (self.seconds_since(t) < self.blacklist_ttl
                    and math.hypot(x - bx, y - by) < self.blacklist_radius):
                return True
        return False

    def prune_blacklist(self):
        self.blacklist = [
            b for b in self.blacklist if self.seconds_since(b[2]) < self.blacklist_ttl]

    # ==========================================================
    # PLANNING LOOP
    # ==========================================================

    def tick(self):
        try:
            self._tick()
        except Exception as e:
            # An exception escaping rclpy.spin() would kill the node with a
            # Nav2 goal still running and nothing watching it.
            self.get_logger().error(f'Exploration loop error: {e}')

    def _tick(self):
        self.prune_blacklist()
        if self.state == NAVIGATING:
            self.watchdog()
            return
        if self.state != EXPLORING:
            return
        if self.map is None:
            self.get_logger().warn('waiting for /map', throttle_duration_sec=5.0)
            return
        pose = self.robot_pose()
        if pose is None:
            return
        rx, ry = pose

        view = GridView(self.map, self.free_threshold, self.occupied_threshold,
                        self.goal_clearance)
        frontiers, frontier_cells = find_frontiers(
            view, rx, ry, self.min_frontier_size, self.min_goal_distance,
            self.gain_scale, self.potential_scale, self.is_blacklisted)
        self.last_frontier_cells = frontier_cells
        self.cur_free_cells = int(view.free.sum())
        self.get_logger().info(
            f'findFrontiers: free={self.cur_free_cells} '
            f'unknown={int(view.unknown.sum())} frontierCells={frontier_cells} '
            f'clusters={len(frontiers)}')
        self.publish_markers(frontiers)

        # Real progress is the map growing. Refresh the clear-retry budget
        # only then, not on arrival: arrival also fires while circling a
        # frontier that can't be consumed, which would livelock short of DONE.
        if self.cur_free_cells > self.free_high_water:
            self.free_high_water = self.cur_free_cells
            self.clears_done = 0

        if not frontiers:
            if self.cur_free_cells < 50:
                self.get_logger().warn(
                    'map not populated yet -- waiting for SLAM',
                    throttle_duration_sec=5.0)
                return
            if (self.last_frontier_cells > 0 and self.blacklist
                    and self.clears_done < self.max_clear_retries):
                self.get_logger().info(
                    f'no navigable frontier ({self.last_frontier_cells} cells, all '
                    f'blacklisted/filtered) -- clearing blacklist + retrying '
                    f'({self.clears_done + 1}/{self.max_clear_retries})')
                self.blacklist = []
                self.clears_done += 1
                self.empty_cycles = 0
                return
            self.empty_cycles += 1
            if self.empty_cycles < self.done_confirm:
                return
            self.get_logger().info('No reachable frontiers left. Exploration complete.')
            self.state = DONE
            self.emit_event('exploration_completed', 'no reachable frontiers left')
            return

        self.empty_cycles = 0
        best = max(frontiers, key=lambda f: f['score'])
        gx, gy = clamp_goal(view, best['x'], best['y'], rx, ry,
                            self.max_goal_distance, self.min_goal_distance,
                            self.is_blacklisted)
        self.send_goal(gx, gy, rx, ry)

    def publish_markers(self, frontiers):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f['x']
            m.pose.position.y = f['y']
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.g = 1.0
            m.color.a = 0.8
            markers.markers.append(m)
        self.markers_pub.publish(markers)

    # ==========================================================
    # NAV2 GOALS
    # ==========================================================

    def send_goal(self, wx, wy, rx, ry):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'waiting for Nav2 action server', throttle_duration_sec=5.0)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        yaw = math.atan2(wy - ry, wx - rx)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        now = self.now_ns()
        self.cur_goal = (wx, wy)
        self.goal_sent_time = now
        self.state = NAVIGATING
        self.near_goal = False
        self.last_progress_pos = (rx, ry)
        self.last_progress_time = now
        self.goal_handle = None
        self.goal_sequence += 1
        self.active_goal_sequence = self.goal_sequence
        self.goal_outcome_emitted = False
        sequence = self.goal_sequence

        self.get_logger().info(
            f'-> frontier ({wx:.2f}, {wy:.2f})  dist={math.hypot(wx - rx, wy - ry):.2f}')
        self.emit_event('goal_sent', 'exploration goal', wx, wy)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, s=sequence: self.goal_response(f, s))

    def goal_response(self, future, sequence):
        try:
            handle = future.result()
        except Exception as e:
            handle = None
            self.get_logger().error(f'Navigation goal failed: {e}')

        if sequence != self.active_goal_sequence or self.state != NAVIGATING:
            # Given up on (advance, stop, scan) before Nav2 answered: make
            # sure it doesn't drive on its own.
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return

        if handle is None or not handle.accepted:
            self.get_logger().warn('goal rejected')
            self.emit_goal_outcome('goal_failed', 'Nav2 rejected goal')
            self.add_blacklist(*self.cur_goal)
            self.state = EXPLORING
            return

        self.goal_handle = handle
        self.emit_event('goal_accepted', 'Nav2 accepted goal', *self.cur_goal)
        handle.get_result_async().add_done_callback(
            lambda f, s=sequence: self.goal_result(f, s))

    def goal_result(self, future, sequence):
        if sequence != self.active_goal_sequence:
            return  # an older goal; advance() already dealt with it
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        # Blacklist every attempted goal: on failure (Nav2 already ran its
        # recovery before aborting) and on success (otherwise it would be
        # picked again forever). Entries expire.
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.emit_goal_outcome('goal_reached', 'Nav2 reached goal')
        else:
            self.get_logger().warn(
                f'goal failed/aborted (status {status}, after Nav2 recovery) -- blacklisting')
            self.emit_goal_outcome('goal_failed', f'Nav2 result status {status}')
        self.add_blacklist(*self.cur_goal)
        self.goal_handle = None
        if self.state == NAVIGATING:
            self.state = EXPLORING

    def advance(self, why, outcome='goal_failed'):
        x, y = self.cur_goal
        self.get_logger().info(
            f'{why} -- blacklisting ({x:.2f},{y:.2f}) + replanning to a different frontier')
        self.emit_goal_outcome(outcome, why)
        self.abandon_goal()
        self.add_blacklist(x, y)
        self.near_goal = False
        self.state = EXPLORING

    def abandon_goal(self):
        """Cancel the goal in flight and ignore any reply about it."""
        self.goal_sequence += 1
        self.active_goal_sequence = self.goal_sequence
        self.goal_outcome_emitted = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def watchdog(self):
        """Each tick while NAVIGATING: anti-circling and anti-stuck."""
        pose = self.robot_pose()
        if pose is not None:
            rx, ry = pose
            gx, gy = self.cur_goal
            # Arrival: lingering near the frontier without Nav2 finishing is
            # circling; the lidar already sees past it, so move on.
            if math.hypot(gx - rx, gy - ry) < self.arrival_radius:
                if not self.near_goal:
                    self.near_goal = True
                    self.near_since = self.now_ns()
                elif self.seconds_since(self.near_since) > self.arrival_patience:
                    self.advance('reached frontier (near, advancing)', 'goal_reached')
                    return
            else:
                self.near_goal = False
            # Progress: wedged for progress_timeout (Nav2's own recovery had
            # its chance) -> try another frontier. Never drive backwards here.
            lx, ly = self.last_progress_pos
            if math.hypot(rx - lx, ry - ly) > self.progress_dist:
                self.last_progress_pos = (rx, ry)
                self.last_progress_time = self.now_ns()
            elif self.seconds_since(self.last_progress_time) > self.progress_timeout:
                self.advance('no progress (stuck)')
                return
        if self.seconds_since(self.goal_sent_time) > self.goal_timeout:
            self.advance('goal timed out')

    # ==========================================================
    # SERVICES
    # ==========================================================

    def handle_start(self, request, response):
        if self.state in (IDLE, DONE):
            self.empty_cycles = 0
            self.state = EXPLORING
            self.emit_event('exploration_started', 'started by /explore/start')
        self.get_logger().info('exploration started')
        return response

    def handle_stop(self, request, response):
        self.stop_motion('stopped by /explore/stop')
        self.state = IDLE
        self.get_logger().info('exploration stopped')
        return response

    def stop_motion(self, why):
        if self.state == NAVIGATING:
            self.emit_goal_outcome('goal_cancelled', why)
            self.abandon_goal()
        if self.state == SCANNING:
            self.cancel_scan()
        if self.state != IDLE:
            self.emit_event('exploration_stopped', why)

    def handle_scan(self, request, response):
        """Turn 360 degrees now (Nav2's collision-checked Spin). The goal in
        flight is paused, not blacklisted, and the blacklist is forgiven: the
        scan gives SLAM fresh data and those spots may be reachable now."""

        if self.state == SCANNING:
            response.success = False
            response.message = 'Already scanning.'
            return response
        if not self.spin_client.server_is_ready():
            response.success = False
            response.message = 'Nav2 spin behavior is not available.'
            return response

        if self.state == NAVIGATING:
            self.emit_goal_outcome('goal_cancelled', 'paused for requested scan')
            self.abandon_goal()
        self.resume_state = IDLE if self.state == IDLE else EXPLORING
        self.blacklist = []
        self.empty_cycles = 0
        self.state = SCANNING

        goal = Spin.Goal()
        goal.target_yaw = float(self.scan_yaw)
        goal.time_allowance = DurationMsg(sec=int(self.scan_time_allowance))
        self.scan_generation += 1
        generation = self.scan_generation
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(lambda f, g=generation: self.scan_response(f, g))

        self.get_logger().info('Scan requested: turning in place.')
        response.success = True
        response.message = 'Scan starting.'
        return response

    def scan_response(self, future, generation):
        try:
            handle = future.result()
        except Exception as e:
            handle = None
            self.get_logger().error(f'Scan failed to start: {e}')
        if generation != self.scan_generation or self.state != SCANNING:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('Nav2 rejected the scan.')
            self.state = self.resume_state
            return
        self.scan_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, g=generation: self.scan_result(f, g))

    def scan_result(self, future, generation):
        if generation != self.scan_generation:
            return
        try:
            status = future.result().status
        except Exception:
            status = GoalStatus.STATUS_UNKNOWN
        self.get_logger().info(f'Scan ended (status {status}).')
        self.scan_handle = None
        if self.state == SCANNING:
            self.state = self.resume_state

    def cancel_scan(self):
        self.scan_generation += 1
        if self.scan_handle is not None:
            self.scan_handle.cancel_goal_async()
            self.scan_handle = None

    # ==========================================================
    # SHUTDOWN
    # ==========================================================

    def stop(self):
        """Cancel whatever Nav2 is doing for us. rclpy.spin() has returned
        by now, so spin briefly here or the cancel is never sent."""
        handles = [h for h in (self.goal_handle, self.scan_handle) if h is not None]
        # Cancelled below, where the reply is waited for.
        self.goal_handle = None
        self.scan_handle = None
        self.stop_motion('explorer stopped')
        self.state = IDLE
        for handle in handles:
            self.get_logger().info('Cancelling active Nav2 goal.')
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


PR_SET_PDEATHSIG = 1  # from <linux/prctl.h>


def stop_when_parent_dies():
    """Get SIGINT when the launching process exits, so the explorer shuts
    down cleanly (cancelling its Nav2 goal) instead of living on as an orphan.

    The RViz AutoMapper panel starts the explorer and stops it on Stop; if
    RViz crashes, nothing else would stop it, and each orphan keeps sending
    Nav2 goals that fight the others.
    """
    try:
        libc = ctypes.CDLL('libc.so.6', use_errno=True)
        libc.prctl(PR_SET_PDEATHSIG, signal.SIGINT)
    except OSError:
        pass  # Not Linux: run without the safeguard.


def main(args=None):
    stop_when_parent_dies()

    # Keep the ROS context alive while Ctrl+C runs stop(): the default rclpy
    # handler shuts it down first, and the goal cancel would never be sent.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = FrontierExplorer()
    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        raise KeyboardInterrupt

    previous_sigint_handler = signal.signal(signal.SIGINT, request_shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # SIGINT can interrupt rclpy mid-conversion, which raises
        # RuntimeError instead of KeyboardInterrupt. Only hide that race.
        if not shutdown_requested:
            raise
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint_handler)


if __name__ == '__main__':
    main()
