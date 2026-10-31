---
title: Simulator
parent: lab-setup
code: [src/go2_sim/**]
---

# Simulator

The simulator is a pretend Oak in a pretend flat, inside Gazebo (a robot simulator). It lets anyone test the [auto mapper](auto-mapper.md), [SLAM](slam.md) and [Nav2](nav2.md) on a laptop, with no robot and no lab, and run the auto mapper many times in a row to see how often it finishes.

Part of: [Lab setup](lab-setup.md)

> **Get it running.** Run every line from a fresh terminal at the repo root. First `git switch master && git pull`.
>
> This branch, `max/fix/explorer-recovery`, adds the auto mapper fixes on top of the simulator, so the auto mapper explores instead of stalling at about 5 % (pending review).
>
> 0. **Stop the robot stack first** (WSL: `cd docker && docker compose down`; PowerShell: `cd docker; docker compose -f docker-compose.windows.yml down`; Mac: Ctrl-C any running `robot.launch.py`). The sim and the robot stack share build folders.
> 1. **Rebuild the image once:** it adds Gazebo, about 1 GB to download. On a 16-core desktop the rebuild took about 11 minutes; a laptop is slower. The image ends up around 14 GB.
>    - WSL: `cd docker && docker compose build && docker compose run --rm unitree_ros true`
>    - Windows PowerShell: `cd docker; docker compose -f docker-compose.windows.yml build; docker compose -f docker-compose.windows.yml run --rm unitree_ros true`
>    - Mac: `bash docker/mac/go2.sh build && bash docker/mac/go2.sh up`
> 2. **Watch one run** (needs RViz windows working: [WSL and Windows](wsl.md), [Mac setup](mac-setup.md)). PowerShell has no windows, so watching is WSL or Mac only.
>    - WSL: `cd docker && docker compose run --rm unitree_ros ros2 launch go2_sim sim.launch.py rviz:=true explore:=true`
>    - Mac: `docker compose -f docker/mac/compose.yaml exec unitree_ros bash -ic 'ros2 launch go2_sim sim.launch.py rviz:=true explore:=true'`
>    - Add `gui:=true` for the Gazebo window too. Without an NVIDIA card it halves the simulator's speed (below).
> 3. **Run the 10 trials** (no windows; about 45-60 minutes):
>    - WSL: `cd docker && docker compose run --rm unitree_ros ros2 run go2_sim trials`
>    - Windows PowerShell: `cd docker; docker compose -f docker-compose.windows.yml run --rm unitree_ros ros2 run go2_sim trials`
>    - Mac: `docker compose -f docker/mac/compose.yaml exec unitree_ros bash -ic 'ros2 run go2_sim trials'`
>
> No NVIDIA card is needed. Tested on 2026-10-28 with no GPU at all, software rendering, 4 CPUs and 6 GB: headless runs kept real-time speed (factor 1.0, about 2 CPUs, at most 1.8 GB). RViz-only watching ran at 0.9. The Gazebo window plus RViz ran at 0.55 and used up to 2.5 GB. The PowerShell and Mac lines are untested: TODO(Adib): try them on a Mac.

## How it works

**No legs, on purpose.** Oak's own computer does the walking; our code only ever tells it "go this fast forward, sideways and turning". So the simulated Oak is one Go2-sized box (0.70 x 0.31 m) that Gazebo slides around at the commanded speed, sideways included. That is what our stack can tell apart, and it is much faster and steadier than simulating legs.

**Same wiring as the real robot.** The simulator starts the real stack, `robot.launch.py`, with one change: the [Go2 driver](go2-driver.md) is swapped for a stand-in, `sim_driver`. The stand-in speaks the driver's topics:

- it takes commands from `/cmd_vel_out` ([twist_mux](twist-mux.md)'s output) and passes them to Gazebo after a short delay;
- it publishes `/odom` and the `odom` -> `base_link` link of the [TF tree](tf.md), with made-up step-counting error added;
- it publishes `/point_cloud2` in the `odom` frame, like the real driver, from a simulated lidar under Oak's chin (where `go2.urdf` puts it); `pointcloud_to_laserscan` turns it into `/scan` as usual;
- it publishes a standing `/joint_states`, so RViz draws the legs.

So SLAM, Nav2, twist_mux and the auto mapper run unchanged. `/imu` and `/go2_states` are not simulated. The front camera is optional and off unless you ask for it (see the dials): rendering it costs CPU on a laptop with no GPU. The cloud reaches `lidar_to_pointcloud` through the same `/robot0/point_cloud2` -> `/point_cloud2` rename as on the robot (the open topic question in the [Test log](test-log.md) applies here too).

**Kept away from the real robot.** The simulator uses ROS domain 42 and talks only inside your computer (`ROS_LOCALHOST_ONLY=1`), so its drive commands can never reach a real Oak on the network. To look at its topics from another terminal, first run `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1`.

**The world.** `placeholder_rooms` is a made-up 10 x 8 m flat: four rooms, 1.2 m doorways, two routes round (so SLAM can close a loop), a few furniture boxes and a pillar. It is not the lab. Jeth is building a tape-measured lab world (issue #28); any world drops in with `world:=`.

## Watch one run (with windows)

You need RViz windows working first ([WSL and Windows](wsl.md) or [Mac setup](mac-setup.md)). Then:

```shell
cd docker
docker compose run --rm unitree_ros ros2 launch go2_sim sim.launch.py rviz:=true explore:=true
```

RViz shows Oak and the map growing. `explore:=true` starts the auto mapper right away. Leave it out to drive yourself (keyboard teleop in a second terminal, with the domain set as above). Start somewhere else with `x:=7.5 y:=7.0 yaw:=1.57`. Add `gui:=true` to also open the Gazebo window, which shows the flat itself; closing it stops everything. Without a GPU the Gazebo window is expensive (see the box above), so RViz alone is the better choice on most laptops.

## Run many trials (headless)

```shell
cd docker
docker compose run --rm unitree_ros ros2 run go2_sim trials
```

This runs the auto mapper 10 times with no windows. Each run starts a fresh simulator at a random start spot (the same spots every time for the same `--seed`), waits for Nav2, starts the auto mapper, and stops at "Exploration complete", after 15 minutes, or when Oak is stuck (moved less than 0.25 m in 3 minutes). It then prints one row per run plus totals:

- **Map complete**: how much of the flat's floor the SLAM map has seen, compared with a true map built from the world file.
- **Time**: from the auto mapper starting to "Exploration complete".
- **Collisions**: separate bumps of Oak's body against walls or furniture (a touch sensor on the body).
- **Distance**: how far Oak really walked.
- **Finished**: yes if the auto mapper said it was done. Otherwise the note says why it stopped and where Oak ended up, so you can find the trouble spot.

Results land in `src/go2_sim/results/<date-time>/` on your laptop (git ignores it): `results.csv`, `results.md` (the table), each run's final map, and logs. Options: `--runs 3`, `--timeout 600`, `--stall-timeout 300` (0 = never), `--seed 2`, `--world <name or .sdf>`. Dials go after `--`, for example `ros2 run go2_sim trials --runs 5 -- lidar_rate:=2.0 odom_noise_xy:=0.05`.

## Watch the trials live

The trials run headless, but you can look in on them. Start the trials in a named container, then attach windows from a second terminal:

```shell
cd docker
docker compose run --rm --name oak-trials unitree_ros ros2 run go2_sim trials
docker exec -it oak-trials bash -ic 'ros2 run go2_sim watch'     # second terminal
```

`watch` opens a Gazebo window and RViz on whichever run is going, and moves to the next run when one ends. Without a GPU use `--no-gazebo` (RViz only): the Gazebo window would slow the run it watches. `--no-rviz` opens Gazebo only. It only looks: closing it, or never opening it, does not change the runs. It needs RViz windows working ([WSL and Windows](wsl.md)).

## The dials

Launch arguments (for `sim.launch.py`, or after `--` for trials). The odometry and command ones live in `src/go2_sim/config/sim.yaml` with a few more.

| Dial | Default | What it does |
| --- | --- | --- |
| `world` | `placeholder_rooms` | Which world: a name in `src/go2_sim/worlds/` or a path to an `.sdf` file |
| `x`, `y`, `yaw` | 2.5, 2.0, 0 | Start spot in the world (m, m, radians) |
| `gui` | `false` | Open the Gazebo window; also opens RViz unless `rviz:=false` |
| `explore` | `false` | Start the auto mapper at once |
| `lidar_rate` | 5 | Lidar scans per second. Measured on Oak: 3 to 6.8 for the cloud, 4 to 5 for `/scan` ([Driver](driver.md), [Test log](test-log.md)). Try 2 for a bad day |
| `lidar_range` | 12 | Farthest the lidar sees (m) |
| `odom_noise_xy` | 0.01 | Random step-counting error: about 1 cm after 1 m, 5 cm after 25 m |
| `odom_noise_yaw` | 0.01 | Random turning error, the same idea for turns |
| `odom_drift_yaw` | 0 | Steady heading drift, radians per meter walked (a slow curve SLAM has to fix) |
| `cmd_latency` | 0.1 | Seconds between a drive command and Oak moving |
| `seed` | 0 | Odometry noise seed (0 = different every run) |
| `camera_rate` | 0, or 5 with `vlm_mode` on | Front camera pictures per second on `/camera/image_raw`, 640 x 360. 0 means no camera. Its 90 degree view is a guess. TODO(owner): measure the real camera's |
| `vlm_mode` | `off` | The auto mapper's [VLM](auto-mapper.md#using-the-vlm) goal ranking: `off`, `shadow` or `active`. Only used with `explore:=true` |
| `vlm_url`, `vlm_model` | empty | Where the VLM server is and which model to ask, passed to the auto mapper |

In `sim.yaml` only: `odom_scale_xy` and `odom_scale_yaw` (Oak says it went or turned a few % more than it did), `cmd_timeout` (Oak stops after 0.5 s without commands), speed and acceleration limits, the lidar mount position, and the self-filter box that drops lidar points on Oak's own head and legs (kept inside the 0.36 m nose, so something touching the nose is still seen). TODO(owner): measure Oak's real odometry error, command delay and stop timeout, and set these defaults from them.

## Adding a world

Put the `.sdf` file anywhere and pass `world:=/ros2_ws/src/<path>.sdf`. Two rules: build it for Gazebo Fortress, and make walls and furniture boxes or cylinders, because the true map for scoring is built from those. A world made of meshes (for example a 3D scan) needs a true map shipped next to it as `<world>.gt.yaml` plus `.pgm`, in the same format the map saver writes. `ros2 run go2_sim ground_truth <world>` writes the true map, so you can open the picture and check it.

## First trial batch (2026-10-28)

The first 10-run batch in the placeholder world, default dials, real-time factor 1.0: **1 of 10 finished, mean map completeness 78.3 % (47.7 to 99.5 %), 0 collisions, 135 m walked**, about 4.5 minutes per run. Eight of the nine unfinished runs ended stuck at or beside a 1.2 m doorway (real doors are narrower).

Getting there, the simulator caught bugs in [Nav2](nav2.md) (the costmap stall below) and in the [Auto mapper](auto-mapper.md)'s unstuck move (three ways it refused every escape). That batch ran with the fixes on this branch, `max/fix/explorer-recovery`, which Luis should review. TODO(Max): add its pull request number.

## What it found so far

With this branch alone (master's Nav2 settings and auto mapper), a 2-run check maps 5.1 % and 5.8 % and never moves: the auto mapper logs "timed out waiting for fresh costmap" after its first scan (first item below).

- **The auto mapper stalls waiting for a costmap.** After Oak stops or spins in place, it waits for a fresh `/local_costmap/costmap`, but Nav2 only re-sends the full grid when its rolling window moves, so it can wait forever ("timed out waiting for fresh costmap"). Fixed with `always_send_full_costmap` (see [Nav2](nav2.md)); still to confirm on Oak.
- **The auto mapper latches stopped next to close obstacles.** Its unstuck move says "no safe direction: rear and both sides lack clearance" and waits, even with open floor behind. The lidar there reads true distances (checked beam by beam against the true map), so this is the auto mapper's own check, not the simulator. Three bugs caused it, all fixed (see [Auto mapper](auto-mapper.md)); still to confirm on Oak.
- **Nav2 drives Oak to within 0.05-0.25 m of door jambs, the pillar and furniture**, then reports "No valid trajectories" or "Failed to make progress". TODO(Luis): does Oak get that close on the real floor?
- **`go2.urdf`'s fixed `odom` -> `base_link` joint** puts `base_link` on the floor until odometry starts; a lidar cloud sent before then shows the floor as walls. The stand-in driver now waits for odometry first. The real driver might hit the same thing at startup (see [TF and robot model](tf.md)).

## VLM test (2026-10-31)

One 8-minute run with the VLM on and one with it off, in the placeholder world with `seed:=1`, using Qwen2.5-VL 7B:

| | VLM `active` | VLM off |
|---|---|---|
| Known map squares at 2 minutes | 27,894 | 27,948 |
| Known map squares at 8 minutes | 28,322 | 28,148 |
| Goals sent / reached | 7 / 5 | 6 / 5 |
| Finished exploring | No | No |

The VLM made no measurable difference, but one run each is too few to say more, and this world's plain gray walls give a camera model little to go on. A world that looks like the real lab would be a fairer test. Both runs stopped mapping after about 2 minutes on `Still stopped: no safe direction on the latest scan`. TODO(Luis): find out why, starting with the lower `obstacle_stop_distance` and `wall_clearance`.

## How it breaks

- **"Gazebo's `ign` command is missing".** The image is older than the simulator: run `docker compose build`.
- **No lidar, startup timeout.** The headless lidar is drawn with OpenGL; without a GPU, Mesa's software renderer (llvmpipe) does it, which works (tested, see the box at the top). If the trial log says `startup timeout: no lidar on /point_cloud2`, look at `logs/run_01_launch.log` for OpenGL errors.
- **Runs slower than real time.** The auto mapper's waits are in wall-clock seconds, so a slow simulator makes it give up early and scores look worse than they are. The trial runner prints a real-time factor per run. Below about 0.9, close the Gazebo window (on a laptop without a GPU it alone halves the speed) or close other programs. Fewer lidar points is not a fix: below 720 per ring, `/scan` gets holes the auto mapper treats as unsafe.
- **It is kinder than the real Oak.** Perfectly flat floor, no people, no glass or mirrors, a cleaner lidar, and a body that slides instead of stepping. A good trial score is a first check, not proof; confirm on Oak and log it in the [Test log](test-log.md).
- **Collisions push, not stop.** Gazebo keeps pushing Oak at the commanded speed while it touches a wall, so one long scrape counts as one bump.
- **The RViz AutoMapper panel works** (it starts the auto mapper in the simulator's domain), but its Reset maps button restarts SLAM, which makes that run's score meaningless.

## Who's on it and what's next

Max added the simulator (branch `max/feat/go2-sim`). Next: Jeth's lab world (issue #28) in place of the placeholder, and real odometry and lidar numbers from Oak for the dials. TODO(owner): an owner for the simulator.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_sim (`launch/sim.launch.py` starts it all; `go2_sim/sim_driver.py` is the stand-in driver; `go2_sim/trials.py` the trial runner; `go2_sim/world_tools.py` builds the true map and scores runs; `worlds/` and `models/oak/` are the Gazebo files.)

Unit tests (no Gazebo needed):

```shell
docker compose run --rm unitree_ros bash -c 'python3 -m pytest /ros2_ws/src/go2_sim/test'
```

<!-- sources: src/go2_sim/** (this branch), src/go2_robot_sdk/launch/robot.launch.py (driver arg), go2_driver_node.py and ros2_publisher.py (topics, odom frame), urdf/go2.urdf (radar_joint), config/nav2_params.yaml (footprint), config/twist_mux.yaml, docs/wiki/driver.md and test-log.md (lidar rates), docs/wiki/auto-mapper.md ("Exploration complete") -->
