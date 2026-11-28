import math
from concurrent.futures import Future

import pytest

import oak_corridor as oc
from ros_stubs import Auto
from go2_auto_mapper.frontier_scoring import find_frontier_clusters

SUCCEEDED, ABORTED, CANCELED = 4, 6, 5
FINISH_WITHIN_SEC = 300.0

POCKETS = {
    'by-far-wall': (5.2, 5.5, -0.15, 0.15),
    'in-corner': (5.2, 5.5, 1.7, 2.0),
    'corridor-end': (-1.5, -1.2, -0.15, 0.15),
}

DOOR_X0, DOOR_X1 = 3.0, 3.9
CLOSET_TOP = 2.4


class MappedRoom(oc.World):
    known_until_x = 99.0

    def __init__(self, unknown=()):
        self.unknown = list(unknown)

    def grid(self, slam):
        info, data = super().grid(slam)
        if slam:
            x0, y0 = info.origin.position.x, info.origin.position.y
            for gy in range(info.height):
                for gx in range(info.width):
                    x = x0 + (gx + 0.5) * oc.RES
                    y = y0 + (gy + 0.5) * oc.RES
                    if any(a <= x <= b and c <= y <= d
                           for a, b, c, d in self.unknown):
                        data[gy * info.width + gx] = -1
        return info, data

    def ray(self, x, y, heading, max_range=10.0):
        dx, dy = math.cos(heading), math.sin(heading)
        best = float('inf')
        for x0, x1, y0, y1 in self.walls:
            near, far = 0.0, max_range
            for p, d, lo, hi in ((x, dx, x0, x1), (y, dy, y0, y1)):
                if abs(d) < 1e-12:
                    if not lo <= p <= hi:
                        near, far = 1.0, 0.0
                    continue
                t0, t1 = sorted(((lo - p) / d, (hi - p) / d))
                near, far = max(near, t0), min(far, t1)
            if near <= far:
                best = min(best, near)
        return best


class DoorwayRoom(MappedRoom):
    walls = [w for w in oc.World.walls if w != (1.5, 5.6, 2.0, 2.1)] + [
        (1.5, DOOR_X0, 2.0, 2.1), (DOOR_X1, 5.6, 2.0, 2.1),
        (DOOR_X0 - 0.1, DOOR_X0, 2.0, CLOSET_TOP + 0.1),
        (DOOR_X1, DOOR_X1 + 0.1, 2.0, CLOSET_TOP + 0.1),
        (DOOR_X0 - 0.1, DOOR_X1 + 0.1, CLOSET_TOP, CLOSET_TOP + 0.1),
    ]

    def __init__(self, seen):
        super().__init__([] if seen else [(DOOR_X0, DOOR_X1, 2.0, CLOSET_TOP)])

    def inside(self, x, y):
        return (super().inside(x, y)
                or (DOOR_X0 <= x <= DOOR_X1 and 2.0 <= y <= CLOSET_TOP))


FAR_X = 9.5       # far wall of the long room
HALF_Y = 3.0      # the long room is 6 m wide
EDGE_X = 4.0      # SLAM has seen the room only up to here
CRUMB, SPECK = 3, 3   # along the edge: 3 free cells, then 3 wall specks


class CrumbledEdgeRoom(MappedRoom):
    """#137: a 6 m wide room runs on to FAR_X, but SLAM has only seen it up
    to EDGE_X, and the sparse lidar left wall specks along that edge. The
    edge to 33 m2 of unseen floor then comes out as 3-cell crumbs, each under
    min_frontier_size, so no frontier survives the noise filter."""

    walls = [
        (-1.6, 1.5, 0.5, 0.6), (-1.6, 1.5, -0.6, -0.5),    # corridor sides
        (-1.7, -1.5, -0.6, 0.6),                            # corridor dead end
        (1.5, FAR_X + 0.1, HALF_Y, HALF_Y + 0.1),           # room sides
        (1.5, FAR_X + 0.1, -HALF_Y - 0.1, -HALF_Y),
        (FAR_X, FAR_X + 0.1, -HALF_Y - 0.1, HALF_Y + 0.1),  # room far wall
        (1.5, 1.6, 0.6, HALF_Y), (1.5, 1.6, -HALF_Y, -0.6),  # room near wall
    ]

    def inside(self, x, y):
        return ((-1.5 <= x <= 1.5 and -0.5 <= y <= 0.5)
                or (1.5 <= x <= FAR_X and -HALF_Y <= y <= HALF_Y))

    def grid(self, slam):
        x0, y0 = -2.0, -HALF_Y - 0.5
        w, h = int(12.0 / oc.RES), int((2 * HALF_Y + 1.0) / oc.RES)
        data = []
        for gy in range(h):
            for gx in range(w):
                x, y = x0 + (gx + 0.5) * oc.RES, y0 + (gy + 0.5) * oc.RES
                if self.occupied(x, y):
                    cell = 100
                elif not self.inside(x, y):
                    cell = -1 if slam else 0
                elif slam and x > EDGE_X:
                    cell = -1
                elif slam and x > EDGE_X - oc.RES and gy % (CRUMB + SPECK) >= CRUMB:
                    cell = 100   # a speck on the last column SLAM has seen
                else:
                    cell = 0
                data.append(cell)
        info = Auto(resolution=oc.RES, width=w, height=h,
                    origin=Auto(position=Auto(x=x0, y=y0)))
        return info, data


class Run(oc.Sim):
    def __init__(self, world):
        real_world = oc.World
        oc.World = lambda: world
        try:
            super().__init__(spin_status=SUCCEEDED, start=(3.0, 0.0, 0.0))
        finally:
            oc.World = real_world
        self.goal = None
        self.finished_at = None

    def server(self, name):
        if name != '/navigate_to_pose':
            return super().server(name)
        sim = self

        def send_goal_async(goal):
            result = Future()
            position = goal.pose.pose.position
            sim.nav_goals.append((sim.clock.t, position.x, position.y))
            if sim.goal:
                sim.end_goal(ABORTED)

            def cancel():
                if sim.goal and sim.goal[1] is result:
                    sim.end_goal(CANCELED)
                done = Future()
                done.set_result(Auto())
                return done
            sim.goal = ((position.x, position.y), result)
            accepted = Future()
            accepted.set_result(Auto(accepted=True, cancel_goal_async=cancel,
                                     get_result_async=lambda: result))
            return accepted

        return Auto(server_is_ready=lambda: True,
                    send_goal_async=send_goal_async,
                    wait_for_server=lambda **_: True)

    def end_goal(self, status):
        _, result = self.goal
        self.goal = None
        if not result.done():
            result.set_result(Auto(status=status))

    def drive(self, dt):
        super().drive(dt)
        if self.goal:
            (gx, gy), _ = self.goal
            x, y, yaw = self.pose
            if math.hypot(gx - x, gy - y) < 0.25:
                self.end_goal(SUCCEEDED)
            else:
                want = math.atan2(gy - y, gx - x)
                err = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
                if abs(err) > 0.2:
                    step = [x, y, yaw + math.copysign(min(abs(err), 0.8 * dt), err)]
                else:
                    step = [x + 0.3 * dt * math.cos(yaw),
                            y + 0.3 * dt * math.sin(yaw), yaw + err]
                if self.world.footprint_hits(*step):
                    self.end_goal(ABORTED)
                else:
                    self.pose = step
        if self.finished_at is None and self.finished():
            self.finished_at = self.clock.t

    def finished(self):
        return any(event.event_type == 'exploration_completed'
                   for event in self.node.event_pub.sent)

    def count_log(self, text):
        return sum(text in message for _, message in self.node.log)


def test_control_finishes_when_nothing_is_left():
    run = Run(MappedRoom())
    run.run(30.0)
    assert run.finished_at is not None


@pytest.mark.parametrize('pocket', sorted(POCKETS))
def test_finishes_when_only_an_unreachable_pocket_is_left(pocket):
    run = Run(MappedRoom([POCKETS[pocket]]))
    run.run(FINISH_WITHIN_SEC)
    assert run.finished_at is not None, (
        f'no exploration_completed in {FINISH_WITHIN_SEC:.0f} s: '
        f'{len(run.nav_goals)} goals, '
        f'{run.count_log("No usable goal")} "No usable goal" logs, '
        f'{run.count_log("Frontier unchanged")} visits that did not shrink it')
    assert not [g for g in run.nav_goals if g[0] > run.finished_at], (
        'kept sending goals after reporting finished')
    assert not run.collided


def test_crumbled_edge_world_has_no_frontier_past_the_noise_filter():
    # The world really is #137's case: crumbs only, all dropped as noise.
    info, data = CrumbledEdgeRoom().grid(slam=True)
    assert find_frontier_clusters(data, info.width, info.height, 5) == []
    crumbs = find_frontier_clusters(data, info.width, info.height, 1)
    assert crumbs and all(len(c) <= CRUMB for c in crumbs)


def test_edge_broken_into_crumbs_is_not_called_finished():
    # #137: with 22 m2 of floor behind the crumbs, don't say the room is done.
    run = Run(CrumbledEdgeRoom())
    run.run(120.0)
    assert not run.finished(), (
        'reported exploration complete with most of the room unseen '
        f'({run.count_log("Not finishing")} "Not finishing" logs)')
    assert run.count_log('Not finishing') > 0
    assert not run.collided


def test_doorway_unseen_on_two_visits_is_not_given_up():
    run = Run(DoorwayRoom(seen=False))
    while run.count_log('Frontier unchanged') < 2 and run.clock.t < 1600.0:
        run.run(5.0)
    assert run.count_log('Frontier unchanged') >= 2, 'doorway never visited twice'

    second_failure, goals_before = run.clock.t, len(run.nav_goals)
    door = ((DOOR_X0 + DOOR_X1) / 2, 2.0)

    def went_back():
        return any(math.hypot(x - door[0], y - door[1]) < 1.0
                   for _, x, y in run.nav_goals[goals_before:])
    while not went_back() and run.clock.t < second_failure + FINISH_WITHIN_SEC:
        run.run(5.0)
        assert not run.finished(), 'reported finished with a doorway still unseen'
    assert went_back(), 'never went back to the doorway'

    run.slam = DoorwayRoom(seen=True).grid(slam=True)
    run.run(60.0)
    assert run.finished()
