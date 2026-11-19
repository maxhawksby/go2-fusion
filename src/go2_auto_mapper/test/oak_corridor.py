"""The real explorer in a fake world: Oak in a 1.0 m corridor that opens
into a half-mapped room, Nav2 replaced by a stand-in that answers every Spin
with a chosen status. The lidar cloud can die part-way and the Spin server
can stay down, and Oak can ignore walking commands slower than its deadband.
No robot, no Docker, no ROS install.

    python src/go2_auto_mapper/test/oak_corridor.py \
        [seconds] [spin_status] [start_yaw_deg] [scan_turn_deg]
"""

import math
import sys
from concurrent.futures import Future

from ros_stubs import Auto, load_explorer

ex = load_explorer()

RES = 0.05
ROBOT_FRONT, ROBOT_REAR, HALF_WIDTH = 0.36, 0.45, 0.20


class World:
    """Walls as boxes (x0, x1, y0, y1). Corridor y in [-0.5, 0.5] for
    x in [-1.5, 1.5], open room x in [1.5, 5.5], y in [-2, 2]."""

    walls = [
        (-1.6, 1.5, 0.5, 0.6), (-1.6, 1.5, -0.6, -0.5),  # corridor sides
        (-1.7, -1.5, -0.6, 0.6),                          # corridor dead end
        (1.5, 5.6, 2.0, 2.1), (1.5, 5.6, -2.1, -2.0),    # room sides
        (5.5, 5.6, -2.1, 2.1),                            # room far wall
        (1.5, 1.6, 0.6, 2.0), (1.5, 1.6, -2.0, -0.6),    # room near wall
    ]
    known_until_x = 3.5  # SLAM has seen the room only up to here

    def occupied(self, x, y):
        return any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0, x1, y0, y1 in self.walls)

    def inside(self, x, y):
        return ((-1.5 <= x <= 1.5 and -0.5 <= y <= 0.5)
                or (1.5 <= x <= 5.5 and -2.0 <= y <= 2.0))

    def grid(self, slam):
        x0, y0, w, h = -2.0, -2.5, int(8.0 / RES), int(5.0 / RES)
        data = []
        for gy in range(h):
            for gx in range(w):
                x, y = x0 + (gx + 0.5) * RES, y0 + (gy + 0.5) * RES
                if self.occupied(x, y):
                    cell = 100
                elif self.inside(x, y):
                    cell = -1 if slam and x > self.known_until_x else 0
                else:
                    cell = -1 if slam else 0
                data.append(cell)
        info = Auto(resolution=RES, width=w, height=h,
                    origin=Auto(position=Auto(x=x0, y=y0)))
        return info, data

    def ray(self, x, y, heading, max_range=10.0):
        step = 0.01
        d = 0.0
        while d < max_range:
            if self.occupied(x + d * math.cos(heading), y + d * math.sin(heading)):
                return d
            d += step
        return float('inf')

    def footprint_hits(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        for i in range(9):
            for j in range(5):
                bx = -ROBOT_REAR + (ROBOT_FRONT + ROBOT_REAR) * i / 8
                by = -HALF_WIDTH + 2 * HALF_WIDTH * j / 4
                if self.occupied(x + bx * c - by * s, y + bx * s + by * c):
                    return True
        return False


class Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def now(self):
        ns = int(self.t * 1e9)
        return Auto(nanoseconds=ns, to_msg=lambda: Auto(
            sec=ns // 10**9, nanosec=ns % 10**9))

    def stamp(self):
        ns = int(self.t * 1e9)
        return Auto(sec=ns // 10**9, nanosec=ns % 10**9)


class Sim:
    def __init__(self, spin_status=6, start=(0.0, 0.0, math.radians(60)),
                 scan_turns_before_abort=0.0, cloud_dies_at=None,
                 spin_ready=True, min_walk_speed=0.0):
        self.scan_cache = {}
        self.min_walk_speed = min_walk_speed
        self.cloud_dies_at = cloud_dies_at
        self.spin_ready = spin_ready
        self.scan_turns_before_abort = scan_turns_before_abort
        self.clock = Clock()
        self.start_t = self.clock.t
        self.world = World()
        self.pose = list(start)
        self.spin_status = spin_status
        self.events = []      # (time, callback) for Nav2 results
        self.nav_goals = []
        self.spins = []
        self.collided = False
        self.last_cmd = (None, None)

        ex.time = self.clock
        ex.ActionClient = lambda _node, action, name: self.server(name)
        ex.FrontierExplorer.get_clock = lambda node: self.clock
        self.node = ex.FrontierExplorer()
        self.node.tf_buffer = Auto(lookup_transform=self.lookup)
        self.searches = 0
        real_find = self.node.find_frontiers

        def counted_find():
            self.searches += 1
            return real_find()
        self.node.find_frontiers = counted_find
        slam_info, slam_data = self.world.grid(slam=True)
        cost_info, cost_data = self.world.grid(slam=False)
        self.slam = (slam_info, slam_data)
        self.cost = (cost_info, cost_data)

    # Nav2 stand-in ----------------------------------------------------
    def server(self, name):
        sim = self

        def send_goal_async(goal):
            result = Future()
            handle = Auto(accepted=True,
                          cancel_goal_async=lambda: Future(),
                          get_result_async=lambda: result)
            if name == '/spin':
                sim.spins.append(goal.target_yaw)
                sim.events.append((sim.clock.t + 1.0,
                                   lambda: sim.finish_spin(goal, result)))
            elif name == '/navigate_to_pose':
                sim.nav_goals.append((sim.clock.t, goal.pose.pose.position.x,
                                      goal.pose.pose.position.y))
            accepted = Future()
            accepted.set_result(handle)
            return accepted

        ready = (lambda: sim.spin_ready) if name == '/spin' else (lambda: True)
        return Auto(server_is_ready=ready, send_goal_async=send_goal_async,
                    wait_for_server=lambda **_: True)

    def finish_spin(self, goal, result):
        target = goal.target_yaw
        if self.spin_status == 4:
            turned = target
        elif abs(target) > 6.0:  # the look-around Scan, refused part-way
            turned = math.copysign(self.scan_turns_before_abort, target)
        else:  # a recovery turn Nav2 refuses before it moves
            turned = 0.0
        self.pose[2] += turned
        if self.world.footprint_hits(*self.pose):
            self.collided = True
        result.set_result(Auto(status=self.spin_status))

    # Sensors -----------------------------------------------------------
    def lookup(self, frame, child, _time):
        x, y, yaw = self.pose
        return Auto(header=Auto(stamp=self.clock.stamp()), transform=Auto(
            translation=Auto(x=x, y=y, z=0.0),
            rotation=Auto(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))

    def publish_sensors(self, with_maps):
        n = self.node
        x, y, yaw = self.pose
        angle_min, inc, count = -math.pi, math.pi / 180, 361
        key = (round(x, 3), round(y, 3), round(yaw, 3))
        if key not in self.scan_cache:
            self.scan_cache[key] = [
                min(self.world.ray(x, y, yaw + angle_min + i * inc), 10.0)
                for i in range(count)]
        ranges = list(self.scan_cache[key])
        scan = Auto(header=Auto(stamp=self.clock.stamp(), frame_id='base_link'),
                    angle_min=angle_min, angle_increment=inc, range_min=0.05,
                    range_max=10.0, ranges=ranges)
        n.observe('scan', scan)
        if (self.cloud_dies_at is None
                or self.clock.t - self.start_t < self.cloud_dies_at):
            n.observe('cloud', Auto(header=Auto(stamp=self.clock.stamp())))
        if with_maps:
            info, data = self.cost
            n.local_costmap_callback(Auto(header=Auto(stamp=self.clock.stamp(),
                                                      frame_id='odom'),
                                          info=info, data=data))
            info, data = self.slam
            n.map_callback(Auto(header=Auto(stamp=self.clock.stamp(),
                                            frame_id='map'),
                                info=info, data=data))

    # Motion: integrate the mapper's own twist (twist_mux input 6) --------
    def drive(self, dt):
        sent = self.node.recovery_pub.sent
        if sent:
            self.last_cmd = (self.clock.t, sent[-1])
            sent.clear()
        when, cmd = self.last_cmd
        if cmd is None or self.clock.t - when > 0.5:
            return
        x, y, yaw = self.pose
        vx, vy = cmd.linear.x, cmd.linear.y
        if math.hypot(vx, vy) < self.min_walk_speed:
            vx = vy = 0.0
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw += cmd.angular.z * dt
        self.pose = [x, y, yaw]
        if self.world.footprint_hits(x, y, yaw):
            self.collided = True

    def run(self, seconds):
        dt = 0.05
        steps = int(seconds / dt)
        zero_holds = 0
        for k in range(steps):
            self.clock.t += dt
            if k % 2 == 0:
                self.publish_sensors(with_maps=(k % 20 == 0))
            for event in [e for e in self.events if e[0] <= self.clock.t]:
                self.events.remove(event)
                event[1]()
            for timer in self.node._timers:
                timer[2] -= dt
                if timer[2] <= 1e-9:
                    timer[2] += timer[0]
                    timer[1]()
            sent = self.node.recovery_pub.sent
            zero_holds += sum(1 for c in sent
                              if c.linear.x == 0 and c.linear.y == 0
                              and c.angular.z == 0)
            self.drive(dt)
        return {
            'seconds': seconds,
            'spin_status': self.spin_status,
            'navigation_goals': len(self.nav_goals),
            'frontier_searches': self.searches,
            'spins_sent': len(self.spins),
            'zero_speed_holds': zero_holds,
            'still_stopped': bool(self.node.recovery_fault),
            'collided': self.collided,
            'pose': tuple(round(v, 2) for v in self.pose),
        }


if __name__ == '__main__':
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    status = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    start_yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    scan_turn = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    sim = Sim(spin_status=status, start=(0.0, 0.0, math.radians(start_yaw)),
              scan_turns_before_abort=math.radians(scan_turn))
    print(sim.run(seconds))
    seen = set()
    for level, msg in sim.node.log:
        key = msg[:60]
        if key not in seen:
            seen.add(key)
            print(f'[{level}] {msg}')
