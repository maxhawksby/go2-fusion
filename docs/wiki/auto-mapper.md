---
title: Auto mapper
parent: maps-nav
code: [src/go2_auto_mapper/**]
---

# Auto mapper

The auto mapper lets Oak map a room by itself: it keeps finding the edges of what it has already mapped and walks there to see more, until nothing is left to explore. **It moves the robot on its own, so only run it with a person watching Oak in a cleared area.**

Part of: [Maps and navigation](maps-nav.md)

## How it works

**Frontiers.** On the [SLAM](slam.md) map, every 5 cm square is free, wall, or unknown. A frontier is a free square right next to an unknown one: the border between "explored" and "not yet seen". Imagine exploring a dark building with a flashlight: the frontier is every doorway and corner you have glimpsed but not walked through. Go stand at a frontier, look around, and the map grows. Repeat until there are no frontiers left.

**One exploration loop** (node `explorer`, every 2 seconds):

1. **Look around first.** At start and after each goal, Oak turns a full circle using Nav2's Spin behavior, so the lidar sees everything around it and SLAM can place it on the map.
2. **Find frontiers.** It scans `/map` for frontier squares and groups neighbours into regions; regions under 5 squares are treated as noise. Since 2026-10-17 this is one fast numpy pass: about 4.4 ms instead of 0.5 s on a 40 x 40 m map.
3. **Pick a goal.** For each region it picks spots 0.8 m back into known space and at least 0.5 m from walls. It skips directions it was just blocked in, spots closer than 1.5 m, and spots without a clear straight line. Of the rest, it picks the one that would reveal the most: it casts 48 imaginary rays from each spot out to 4 m and counts unknown squares they would uncover, minus a cost per meter of walking.
4. **Go.** It sends the goal to [Nav2](nav2.md) on `/navigate_to_pose`, exactly like clicking "Nav2 Goal" in RViz, so Nav2's obstacle avoidance still applies.
5. **Learn from it.** If a visit did not shrink the frontier (the opening can't be seen into), that frontier is skipped for 2 minutes. Failed goals are also skipped for 2 minutes.
6. **Get unstuck.** If something is within 0.3 m of the nose, it cancels the goal and checks short moves backward, left and right against the live `/scan` and Nav2's local costmap, then slides 0.05 to 0.5 m at 0.1 m/s along the safest one. These moves go out on `/cmd_vel_mapper`, a [twist_mux](twist-mux.md) input at priority 6 (above Nav2, below keyboard and gamepad). If nothing is safe, it stays stopped and logs why.
7. **Head for open space.** After 3 failed goals in a row (or similar signs of being stuck), it drives to the nearest spot with 0.8 m of room on every side, scans, and carries on.
8. **Finish.** With no frontiers left it logs "Exploration complete" and waits, in case a door opens.

**Buttons.** The RViz AutoMapper panel has **Start/Stop**, **Scan** (pause the goal, forget recent failures, turn 360 degrees; the same as calling `/frontier_explorer/scan`) and **Reset maps** (restart SLAM with an empty 2D map and clear the 3D point cloud).

### The finer details

- **Before SLAM is ready.** Until SLAM publishes the `map` frame, the explorer logs "Waiting for robot position" and does nothing else.
- **Goal spots.** Each region offers its middle plus one spot about every meter along long regions. The "clear straight line" check uses Nav2's local costmap (`/local_costmap/costmap`). The reveal score subtracts `info_gain_cells_per_meter` (100) unknown squares per meter of travel; the logic lives in `frontier_scoring.py`.
- **Unstuck moves, exactly.** It checks the whole padded body (0.36 m forward, 0.45 m back, 0.40 m wide, plus a 5 cm margin) everywhere it could be during the move: anywhere along the way, up to 0.05 m off the line and turned up to 0.10 rad (about 6 degrees), which are the limits that abort the move. They were 0.10 m and 0.15 rad; at those, the bigger envelope touched walls 0.17 m away and Oak never made a recovery move in the simulator. Only walls themselves (lethal costmap cells) block it, not their inflation. The one exception is the obstacle that stopped Oak: while backing straight away from it, anything past the nose is ignored. Unknown cells, a scan older than 0.75 s, and missing or infinite rays toward the space the move enters all count as unsafe. It picks the safe corridor with the most extra clearance, moves for at most 8 seconds, and rechecks 4 times a second. `/scan` must be in the `base_link` frame. If no direction is safe, the robot drifts, or clearance disappears, it stays stopped (a "latch"). A later fresh scan clears the latch when the way ahead is open, or starts another short move when a rear or side corridor has opened; **Scan** clears it at once.
- **After a move** it stays stopped until a newer `/scan`, live `/point_cloud2`, local costmap and map/odom transforms arrive. A new SLAM `/map` is needed only when the move was at least 0.2 m (slam_toolbox's minimum travel). Obstacles that moved away are cleared by Nav2's costmap voxel layer tracing `/scan` (`clearing: true` in `nav2_params.yaml`); unknown local costmap cells stay unknown, so recovery never treats them as free. The scan's height filter also limits which obstacles recovery can see.
- **Stuck** means 3 failed goals in a row (`stuck_failure_count`), an obstacle-ahead abort, or "No usable goal" twice running. The open-space search goes through free space up to 4 m away (so never through walls). A failed escape does not trigger another one, escapes are at least 30 s apart, and escape goals always trigger a scan, even with `scan_after_goal` off. A failed scan also leaves exploration stopped.
- **Stopping.** Ctrl+C cancels the in-flight Nav2 goal and waits briefly for the cancel to reach Nav2, so Oak does not keep walking toward a goal nobody tracks. Publishing an empty Twist on `/cmd_vel_joy` works as an emergency stop because twist_mux gives it the highest priority (10), overriding Nav2.

### Settings

| Parameter | Default | Meaning |
| --- | --- | --- |
| `map_topic` | `/map` | Occupancy grid topic to read |
| `robot_frame` | `base_link` | TF frame for the robot's position |
| `map_frame` | `map` | TF frame the map is published in |
| `min_frontier_size` | `5` | Minimum cluster size (in cells) to count as a real frontier |
| `goal_distance` | `0.8` | How far inside known space, toward the robot, to place the goal |
| `min_goal_distance` | `1.5` | Skip goals closer than this (meters) while any farther frontier is left |
| `wall_clearance` | `0.5` | Skip goal spots closer than this (meters) to a wall in the SLAM map |
| `info_gain_range` | `4.0` | How far (meters) the lidar is assumed to see when estimating what a goal would reveal |
| `info_gain_rays` | `48` | Rays cast per candidate for that estimate |
| `info_gain_cells_per_meter` | `100.0` | Unknown cells a meter of travel must be worth (400 cells = 1 square meter at 5 cm) |
| `obstacle_stop_distance` | `0.3` | Gap (meters) in front of the nose that counts as blocked |
| `blocked_timeout` | `0.0` | Seconds an obstacle must stay ahead before a goal is given up (0 = at once) |
| `backup_distance` | `0.3` | How far (meters) the unstuck move goes, clamped to 0.05-0.5 m |
| `open_space_clearance` | `0.8` | Room (meters) on every side that counts as open space when stuck |
| `stuck_failure_count` | `3` | Failed goals in a row that count as stuck |
| `scan_on_start` | `true` | Turn a full circle before exploring |
| `scan_after_goal` | `true` | Turn a full circle after each goal it reaches |
| `scan_yaw` | `6.283` (2 pi) | How far to turn per scan, in radians |
| `scan_time_allowance` | `30.0` | Seconds Nav2 has to finish the turn before giving up |

## Connects to

- [SLAM](slam.md) - reads `/map` and the `map` frame.
- [Nav2](nav2.md) - sends goals to `/navigate_to_pose`, uses `/spin` and `/backup`, reads `/local_costmap/costmap`.
- [Lidar](lidar.md) - reads `/scan` and `/point_cloud2` to check it is safe to move.
- [twist_mux](twist-mux.md) - recovery moves on `/cmd_vel_mapper` (priority 6).
- [RViz panels](rviz-panels.md) - the AutoMapper panel starts and stops it.
- [Simulator](simulator.md) - try it with no robot, or score it over 10 runs.

## How it breaks

From the package README and Luis's commits (Oct 14-19):

- **Stopping for no reason.** Drifted floor points in the 3D cloud were marked as obstacles, and a 0.5 m lidar cut-off hid walls beside the robot. Fixed by giving Nav2 only the filtered 2D scan and lowering the minimum range to 0.1 m.
- **"No usable goal".** Press Scan: it re-looks and retries recently failed spots.
- **Nibbling the nearest edge.** Ranking goals by distance made it creep along the closest frontier; replaced by "most revealed per meter" on 2026-10-16.
- **Two motions at once.** Codex reviews on 2026-10-17 found cases where a late Nav2 reply or a recovery move could override a stop; each maneuver is now tracked until Nav2 confirms it ended, and a latched stop always wins.
- **Latched stopped next to the first close wall.** The unstuck check used to look at Oak's whole padded body, including the nose. Once something was within about 0.25 m of the nose, its inflated cost already covered the body, so backing away and both side moves all looked blocked, and Oak waited forever ("no safe direction: rear and both sides lack clearance"). It also refused moves through Nav2's inscribed band (cells where the robot's *centre* would touch) on top of the padded body, which doubled the body's width, so backing out along a wall 0.2 m away looked blocked too. And it never accepted backing up at all: straight behind Oak is the scan's last-plus-one ray, which it read as "no measurement". Found in the [Simulator](simulator.md). Now the whole body is checked through the allowed drift against walls themselves (lethal cells), only the obstacle Oak backs away from is ignored, and the scan wraps around. A first version checked only the half of the body facing the move, which a review caught: a wall beside the front half could be hit if Oak turned while backing up. Not yet confirmed on Oak.
- **Backing up is new on the real Oak.** On master these moves were always refused, so Oak has never actually backed up or slid sideways on its own. With these fixes it will. The first tries on the real floor need a spotter ready to stop it (an empty Twist on `/cmd_vel_joy`, or the gamepad).
- **Waits for fresh data.** After every scan or move it waits up to 20 s for newer `/scan`, `/point_cloud2`, `/map`, costmap and TF. If they don't come, exploration stays stopped until Scan is pressed.
- **The 3D cloud never forgets** moved objects; neither SLAM nor the 3D map is cleared during recovery.
- **Big maps** make each step slower, because the whole map is rescanned every cycle.

## Who's on it and what's next

**Luis** owns the auto mapper. In the week of 2026-10-18 he made exploration reliable (fixes above, Scan and Reset maps buttons, faster frontiers, Nav2 and SLAM tuning) and merged it to master. Next: full exploration tests on Oak.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_auto_mapper (`explorer.py` runs the loop; `frontier_scoring.py`, `recovery.py`, `obstacle_sensing.py` and `open_space.py` hold the logic that has unit tests).

With the robot stack running (SLAM and Nav2 on) and a person watching Oak:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 run go2_auto_mapper explorer'
```

Change a setting at start, for example `ros2 run go2_auto_mapper explorer --ros-args -p min_frontier_size:=10`. Scan on demand: `ros2 service call /frontier_explorer/scan std_srvs/srv/Trigger`.

Ctrl+C stops it and cancels the Nav2 goal. If it runs in the background, stop it with `pkill -f 'go2_auto_mapper/lib/go2_auto_mapper/explorer'`. Emergency stop from another terminal:

```shell
docker compose exec unitree_ros bash -ic 'ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'
docker compose exec unitree_ros bash -ic 'ros2 topic pub /cmd_vel_joy geometry_msgs/msg/Twist "{}" --once'
```

Tests, no robot needed:

```shell
docker compose run --rm unitree_ros bash -c 'python3 -m pytest /ros2_ws/src/go2_auto_mapper/test'
```

<!-- sources: src/go2_auto_mapper/README.md, go2_auto_mapper/explorer.py (subscriptions, action clients, /cmd_vel_mapper), obstacle_sensing.py, frontier_scoring.py, src/go2_robot_sdk/config/twist_mux.yaml, src/go2_rviz_plugins/src/auto_mapper_panel.cpp, docs/USEFUL_COMMANDS.md, AGENTS.md, docs/FUSION.md, git log Oct 14-19 (commits 7ebad74, 6bf385f, 1973d50, 7b3a8e7, 9b9e25b, 264939b, a51627d), contract team updates (Luis) -->
