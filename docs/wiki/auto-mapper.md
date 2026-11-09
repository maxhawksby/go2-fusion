---
title: Auto mapper
parent: maps-nav
code: [src/go2_auto_mapper/**]
verified: 5df52bad1708
---

# Auto mapper

The auto mapper lets Oak map a room by itself: it keeps finding the edges of what it has already mapped and walks there to see more, until nothing is left to explore. **It moves the robot on its own, so only run it with a person watching Oak in a cleared area.**

Part of: [Maps and navigation](maps-nav.md)

## How it works

**Frontiers.** On the [SLAM](slam.md) map, every 5 cm square is free, wall, or unknown. A frontier is a free square right next to an unknown one: the border between "explored" and "not yet seen". Imagine exploring a dark building with a flashlight: the frontier is every doorway and corner you have glimpsed but not walked through. Go stand at a frontier, look around, and the map grows. Repeat until there are no frontiers left.

Since November 2026 the explorer is a Python port of the frontier explorer in Manas Arumalla's [autonomous-go2-inspection](https://github.com/Manas-arumalla/autonomous-go2-inspection) (written there in C++ for ROS 2 Jazzy and Gazebo Harmonic). It only reads `/map` and TF and only talks to Nav2, so it runs the same on Oak and in our Gazebo Fortress [Simulator](simulator.md).

**One planning loop** (node `frontier_explorer`, every 2 seconds):

1. **Find frontiers.** It marks every frontier square on `/map` and groups touching ones into regions. Regions under 8 squares are noise.
2. **Aim at each region.** Each region's goal is its square *nearest* Oak that is at least 0.8 m away and at least 0.55 m from any wall (about Nav2's inflation and more than the body's 0.45 m reach), so Nav2 can fit Oak there. Unknown squares don't count as walls.
3. **Pick the best region.** Score = region size minus 3 x distance in meters: big openings beat small ones, close beats far.
4. **Take short steps.** A goal more than 3 m away is replaced by the farthest safe spot on the straight line toward it within 3 m. The next loop looks again and steps again, so Nav2 rarely gets a long goal it can't finish.
5. **Go.** It sends the goal to [Nav2](nav2.md) on `/navigate_to_pose`, facing the frontier, exactly like clicking "Nav2 Goal" in RViz.
6. **Watch the walk.** While Nav2 drives, the explorer checks every loop: within 0.55 m of the goal for 3 s counts as arrived (the lidar already sees past it); less than 0.25 m of movement in 18 s means stuck; 60 s total means timed out. Either way the goal is cancelled and another frontier is picked.
7. **Blacklist.** Every goal it tried (reached or failed) is skipped for 30 s within 0.5 m, so it doesn't pick the same spot again. Entries expire, so no area is given up on for good.
8. **Finish.** With no frontier left: if frontier squares exist but all were blacklisted, it clears the blacklist and retries (up to 3 times until the map grows again). It logs "Exploration complete" (the simulator trial runner watches for this text) only after 3 empty loops in a row, and then stops.

**Recovery belongs to Nav2.** The explorer never sends a speed itself. Backing up and spinning when stuck are Nav2's own behaviors, which check the costmap before moving; a blind reverse could hit what is behind Oak.

The frontier goals are drawn as small green dots on `/explore/frontiers` (add a MarkerArray display in RViz). At each step it also publishes `/auto_mapper/events` for the [Scoreboard](scoreboard.md): exploration started/stopped/completed and goal sent/accepted/reached/failed/cancelled. These only report; they don't change what the explorer does.

**Buttons and services.** The RViz AutoMapper panel's **Start/Stop** runs and kills the node; **Scan** calls `/frontier_explorer/scan`: it pauses the current goal (not counted as failed), forgets the blacklist and turns 360 degrees with Nav2's Spin, then carries on. `/explore/stop` and `/explore/start` (`std_srvs/srv/Empty`) pause and resume exploring without killing the node.

### The finer details

- **Before SLAM is ready.** Until `/map` exists, has at least 50 free squares and SLAM publishes the `map` frame, it logs "waiting" and does nothing, so an empty first map can't end exploring at once.
- **Late Nav2 replies.** Each goal is numbered. A reply about a goal that was already given up is ignored, and if Nav2 accepts it late it is cancelled, so two goals never run at once. (The C++ original could blacklist the new goal by mistake here.)
- **Stopping.** Ctrl+C cancels the Nav2 goal or Spin in flight and waits up to 2 s for Nav2 to hear it. If RViz (which started it) dies, the explorer gets Ctrl+C too. Publishing an empty Twist on `/cmd_vel_joy` is an emergency stop because [twist_mux](twist-mux.md) gives it the highest priority (10).
- **Simulated time.** The loop runs on the wall clock, like the original; the timeouts above use ROS time, so they follow `/clock` in the simulator.

### Settings

| Parameter | Default | Meaning |
| --- | --- | --- |
| `min_frontier_size` | `8` | Smallest region (squares) that counts as a frontier |
| `min_goal_distance` | `0.8` | Goals are at least this far (m) from Oak |
| `goal_clearance` | `0.55` | Goals are at least this far (m) from any wall |
| `free_threshold` / `occupied_threshold` | `25` / `65` | Map values at or below / at or above these count as free / wall |
| `gain_scale` / `potential_scale` | `1.0` / `3.0` | Score = gain_scale x size - potential_scale x distance |
| `max_goal_distance` | `3.0` | Longest single step (m) toward a far frontier |
| `arrival_radius` / `arrival_patience` | `0.55` / `3.0` | Within this many m for this many s counts as arrived |
| `progress_dist` / `progress_timeout` | `0.25` / `18.0` | Less movement (m) than this for this long (s) counts as stuck |
| `goal_timeout` | `60.0` | Seconds before any goal is given up |
| `blacklist_radius` / `blacklist_ttl` | `0.5` / `30.0` | Tried goals block this radius (m) for this long (s) |
| `done_confirm` / `max_clear_retries` | `3` / `3` | Empty loops before COMPLETE / blacklist clears before giving up |
| `planning_period` | `2.0` | Seconds between loops |
| `autostart` | `true` | Start exploring at once (else wait for `/explore/start`) |
| `robot_base_frame` / `map_frame` | `base_link` / `map` | TF frames |
| `scan_yaw` / `scan_time_allowance` | `6.283` / `30.0` | Scan button: turn (rad) and seconds Nav2 gets for it |

## Connects to

- [SLAM](slam.md) - reads `/map` and the `map` frame.
- [Nav2](nav2.md) - sends goals to `/navigate_to_pose`, and `/spin` for the Scan button. Nav2 does all obstacle avoidance and recovery.
- [twist_mux](twist-mux.md) - no longer used: the `/cmd_vel_mapper` input (priority 6) is left over from the old recovery moves.
- [RViz panels](rviz-panels.md) - the AutoMapper panel starts and stops it.
- [Simulator](simulator.md) - try it with no robot, or score it over 10 runs.
- [Scoreboard](scoreboard.md) - counts goals and recoveries from structured events.

## How it breaks

For the current explorer (not yet tried on Oak or in the simulator, TODO(jeth): run a simulator trial and a watched run on Oak):

- **Goals inside the inflation.** If `goal_clearance` is smaller than Nav2's global `inflation_radius`, goals land where Nav2 can't place Oak; they get rejected and blacklisted, and exploring wastes time. Keep the two about equal.
- **Gives up on a narrow doorway.** A frontier whose every square is within 0.55 m of a wall offers no goal at all, so a doorway narrower than about 1.1 m is never entered.
- **Wedged.** It never moves Oak itself; if Nav2's own recovery can't free Oak, the explorer only cancels after 18 s and tries elsewhere. Use the gamepad or keyboard.
- **COMPLETE too early or never.** It needs 3 empty loops to finish; if SLAM keeps adding tiny edges, it keeps going.

Older history, from the previous explorer that this port replaced (package README and Luis's commits, Oct 14-19):

- **Stopping for no reason.** Drifted floor points in the 3D cloud were marked as obstacles, and a 0.5 m lidar cut-off hid walls beside the robot. Fixed by giving Nav2 only the filtered 2D scan and lowering the minimum range to 0.1 m.
- **"No usable goal".** Press Scan: it re-looks and retries recently failed spots.
- **Nibbling the nearest edge.** Ranking goals by distance made it creep along the closest frontier; replaced by "most revealed per meter" on 2026-10-16.
- **Two motions at once.** Codex reviews on 2026-10-17 found cases where a late Nav2 reply or a recovery move could override a stop; each maneuver is now tracked until Nav2 confirms it ended, and a latched stop always wins.
- **Latched stopped next to the first close wall.** The unstuck check used to look at Oak's whole padded body, including the nose. Once something was within about 0.25 m of the nose, its inflated cost already covered the body, so backing away and both side moves all looked blocked, and Oak waited forever ("no safe direction: rear and both sides lack clearance"). It also refused moves through Nav2's inscribed band (cells where the robot's *centre* would touch) on top of the padded body, which doubled the body's width, so backing out along a wall 0.2 m away looked blocked too. And it never accepted backing up at all: straight behind Oak is the scan's last-plus-one ray, which it read as "no measurement". Found in the [Simulator](simulator.md). Now the whole body is checked through the allowed drift against walls themselves (lethal cells), only the obstacle Oak backs away from is ignored, and the scan wraps around. A first version checked only the half of the body facing the move, which a review caught: a wall beside the front half could be hit if Oak turned while backing up. These fixes reached master in pull request #41 (2026-11-05). In the simulator Oak can still latch stopped beside a 1.2 m doorway (see [Simulator](simulator.md)). Not yet confirmed on Oak.
- **Backing up is new on the real Oak.** Before pull request #41 these moves were always refused, so Oak has never actually backed up or slid sideways on its own. Now it will. The first tries on the real floor need a spotter ready to stop it (an empty Twist on `/cmd_vel_joy`, or the gamepad).
- **Ctrl+C did not stop cleanly.** ROS shut itself down before the auto mapper's stop step ran, so the final zero speed and the Nav2 goal cancel never went out. Fixed in pull request #41: ROS now stays up until that step is done.
- **Waits for fresh data.** After every scan or move it waits up to 12 s for newer `/scan`, `/point_cloud2`, `/map`, costmap and TF. If they don't come, exploration stays stopped until Scan is pressed.
- **The 3D cloud forgets moved objects only where Oak looks** (see [Lidar](lidar.md)); neither SLAM nor the 3D map is cleared during recovery.
- **Big maps** make each step slower, because the whole map is rescanned every cycle.

## Who's on it and what's next

**Luis** owns the auto mapper. In the week of 2026-10-18 he made exploration reliable (fixes above, Scan and Reset maps buttons, faster frontiers, Nav2 and SLAM tuning) and merged it to master. Max fixed the unstuck move after the [Simulator](simulator.md) found it latching (pull request #41), and Noah fixed the Ctrl+C cleanup in the same pull request. In November 2026 the explorer was replaced by the port of the autonomous-go2-inspection explorer (branch `jeth/feat/automap-rework`), which hands all recovery to Nav2. Next: simulator trials, then a watched run on Oak.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_auto_mapper (`explorer.py` is the whole explorer; the previous explorer's helper files were removed with it).

With the robot stack running (SLAM and Nav2 on) and a person watching Oak:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 run go2_auto_mapper explorer'
```

Change a setting at start, for example `ros2 run go2_auto_mapper explorer --ros-args -p min_frontier_size:=10`. Pause / resume: `ros2 service call /explore/stop std_srvs/srv/Empty` (and `/explore/start`). Scan on demand: `ros2 service call /frontier_explorer/scan std_srvs/srv/Trigger`.

Ctrl+C stops it and cancels the Nav2 goal. If it runs in the background, stop it with `pkill -f 'go2_auto_mapper/lib/go2_auto_mapper/explorer'`. Emergency stop from another terminal:

```shell
docker compose exec unitree_ros bash -ic 'ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'
docker compose exec unitree_ros bash -ic 'ros2 topic pub /cmd_vel_joy geometry_msgs/msg/Twist "{}" --once'
```

Tests, no robot needed:

```shell
docker compose run --rm unitree_ros bash -c 'python3 -m pytest /ros2_ws/src/go2_auto_mapper/test'
```

The robot build check on GitHub also runs these tests on every pull request (style checks are skipped there for now).

<!-- sources: autonomous-go2-inspection go2_exploration/src/frontier_explorer.cpp, src/go2_robot_sdk/config/nav2_params.yaml, src/go2_auto_mapper/README.md, go2_auto_mapper/explorer.py (subscriptions, action clients, /cmd_vel_mapper), obstacle_sensing.py, frontier_scoring.py, src/go2_robot_sdk/config/twist_mux.yaml, src/go2_rviz_plugins/src/auto_mapper_panel.cpp, docs/USEFUL_COMMANDS.md, AGENTS.md, docs/FUSION.md, git log Oct 14-19 (commits 7ebad74, 6bf385f, 1973d50, 7b3a8e7, 9b9e25b, 264939b, a51627d), contract team updates (Luis) -->
