---
title: Simulator
parent: lab-setup
code: [src/go2_sim/**]
verified: 669d6eb2be69
---

# Simulator

The simulator is a pretend Oak in a pretend flat, inside Gazebo (a robot simulator). It lets anyone test the [auto mapper](auto-mapper.md), [SLAM](slam.md) and [Nav2](nav2.md) on a laptop, with no robot and no lab, and run the auto mapper many times in a row to see how often it finishes.

Part of: [Lab setup](lab-setup.md)

> **Get it running.** Run every line from a fresh terminal at the repo root. First `git switch master && git pull`.
>
> With the auto mapper recovery fixes, the auto mapper explores instead of stalling at about 5 % of the map, but it can still latch stopped at a doorway (see [First trial batch](#first-trial-batch-2026-10-28)).
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

## ITE-inspired hallways

`ite_hallways` is a hallway-only practice world based on Noah's description,
not a measured third-floor plan. It has a 24 x 24 m interior, a 3 m wide square
perimeter corridor and a 5 m wide cross-hall linking the left and right sides.
Two solid cores stand in for inaccessible rooms, elevators and stairs. Two
solid table boxes sit at the west end of the lobby; the east boundary stands
in for the closed elevator/stair entrance side. All dimensions are assumptions.
TODO(noah): confirm orientation and replace assumed lengths with measurements.

![Assumed hallway layout, closed cores and lobby tables](assets/ite-hallways.svg)

This tests turning at corners, choosing a branch, closing loops and returning
to hallways skipped while exploring the lobby. The explorer must retain
reachable frontiers beyond the current view; heading toward the biggest open
space alone does not guarantee it finishes. The generated answer key is used
only by the scorers, not by SLAM or goal selection. Boxes make both collision
geometry and scoring agree; individual rooms and stair drops are not simulated.

From this branch's repo root in PowerShell, run these yourself, in order:

```powershell
# Unit checks, then a saved ground-truth preview; neither launches exploration.
.\scripts\run_ite_hallways.ps1 -Mode Validate
.\scripts\run_ite_hallways.ps1 -Mode Preview
# One 60-second exploration run to check startup and saved artifacts.
.\scripts\run_ite_hallways.ps1 -Mode Smoke
# Only after the smoke run works: five matched starts, up to ten minutes each.
.\scripts\run_ite_hallways.ps1 -Mode Batch -Runs 5 -Seed 28 -Timeout 600
```

The script reuses the installed `go2scoreboard-unitree_ros:latest` image;
`-Image <existing-simulator-image>` selects another. It does not rebuild or
download an image. Compose project `go2ite` gets its own build/install volumes,
and a fixed container name prevents two script sessions running together. The
image entrypoint builds the mounted workspace, so allow startup/build time.
No new Docker dependencies are required for this world. Keep other simulator
sessions stopped while running this suite. The trial runner selects an isolated
ROS domain starting at 50 for each run; direct simulation launches default to 42.

Each invocation saves `session.json`, the world, a tracked source patch, console
log and exit code under `src/go2_sim/results/ite_hallways/`. Trial modes also
save the existing CSV/report, maps, logs and scoreboard artifacts beneath that
session's `trials/` folder. Bags are off by default; add `-RecordBag` when
replay evidence is needed. Keep the commit unchanged during a batch. A successful
process exit means the command ran; inspect `finished`, coverage, collisions and
stop reasons to judge mapping. Startup failures are not algorithm failures.

The headless preview writes a `.pgm` and `.yaml`; the drawing above can be
viewed without Docker. GUI users can instead launch the existing simulator
with `world:=ite_hallways x:=1.5 y:=1.5 yaw:=1.57 gui:=true explore:=false`
and start exploration deliberately after checking the layout. PowerShell GUI
setup is unchanged.

### First hallway smoke result and diagnostic follow-up

The first seed-28000 smoke test on November 10 ran for 60 seconds: 3.19%
completeness, about 0.32 m traveled, zero collisions, one goal accepted and
none reached. The scoreboard parity check passed. The initial spin predicted
a collision, and Nav2 repeatedly failed its progress check; DWB later rejected
unknown-region and oscillating trajectories. This establishes a stall, not its
root cause. The aggregator's separate `Time`/`frame_id` error is also present,
but this launch builds navigation's `/scan` from `/point_cloud2` directly.

Keep the same world, seed and algorithm for one diagnostic repeat:

```powershell
.\scripts\run_ite_hallways.ps1 -Mode Smoke -Seed 28 -Variant scan_diagnostic -RecordBag
```

This records the existing scoreboard topics, including `/scan`, local costmap,
TF, output velocity and ROS logs. It does not include the raw point cloud or
every Nav2 internal topic. No longer batch is needed until the stall is understood.
The offline helper reads bags without launching ROS nodes, playing messages or
issuing motion commands. In a container with the workspace sourced, run:

```shell
python3 /ros2_ws/src/go2_sim/go2_sim/navigation_diagnostic.py /ros2_ws/src/go2_sim/results/ite_hallways
```

Each recorded run gets `navigation_diagnostic.json` next to its bag. It reports
scan frames and directional returns, topic gaps, transforms present on both
static and dynamic TF, output velocity and relevant failure logs. These are
investigation clues, not a safety verdict. Aggregate scan statistics cover the
whole bag, and costmap unknown percentage covers the whole grid. The first
smoke test had bags disabled, so these measurements cannot be recovered from it.

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

Results land in `src/go2_sim/results/<date-time>/` on your laptop (git ignores it): `results.csv`, `results.md` (the table), each run's final map, and logs. Options: `--runs 3`, `--timeout 600`, `--stall-timeout 300` (0 = never), `--seed 2`, `--world <name or .sdf>`, `--first-index 3 --runs 1` (only the fourth start spot of that seed), `--domain 60` (a fixed ROS domain, so several runners can work side by side). Dials for the simulator go after `--`, for example `ros2 run go2_sim trials --runs 5 -- lidar_rate:=2.0 odom_noise_xy:=0.05`. An experiment can temporarily change an auto-mapper ROS parameter without editing its code, for example `--variant blocked_grace --explorer-param blocked_timeout:=2.0`. The variant and every override are saved with the scoreboard run so comparisons do not mix configurations.

The trial runner also starts the [Scoreboard](scoreboard.md) before each auto
mapper. Its independent artifacts are under the run's `scoreboard/` folder,
including a rosbag by default. The legacy simulator scorer remains in place and
both scorers begin at the auto mapper's structured `exploration_started` event.
The CSV reports whether reported odometry distance (within 10% or 0.5 m),
completeness (within one percentage point) and collision counts agree. It also
keeps Gazebo's hidden physical distance and reports the odometry error separately;
that ground truth is an answer key, not data available to Oak. Use
`--no-scoreboard-bag` for a small development run or `--no-scoreboard` to
isolate the legacy scorer.

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

Getting there, the simulator caught bugs in [Nav2](nav2.md) (the costmap stall below) and in the [Auto mapper](auto-mapper.md)'s unstuck move (three ways it refused every escape). That batch ran with the first version of those fixes, before the unstuck move learned to check Oak's whole body through its allowed drift. With the final version, one 8-minute run (seed 1, 2026-10-31) mapped 48.3 %, with 0 costmap timeouts and 0 collisions, but latched stopped next to the 1.2 m doorway after 44 s ("no safe direction: rear and both sides lack clearance") and never moved again. Master without the fixes maps 5.0 % in the same run. TODO(Max): re-run the 10-run batch with the final fixes.

Every simulator experiment since 2026-11-06 (hypothesis, change, runs, why each run quit, verdict) is logged in the [experiments journal](experiments.md).

## Score VLM decisions

Comparing whole runs with the VLM on and off needs very many runs: where Oak happens to get stuck swings a run by 50 points either way. Scoring each decision instead turns one run into dozens of paired comparisons. Run the auto mapper in `shadow` mode with a navigation log (`--explorer-param vlm_mode:=shadow --explorer-param nav_log_dir:=<run>/nav`), then:

```shell
ros2 run go2_sim score_decisions <run folder> [<run folder> ...] --csv decisions.csv
```

Each run folder holds `nav/` and the trial runner's output (its `results.csv` gives the start pose; its `.gt.yaml` is the true map). For every logged ranking it sweeps 360 lidar rays out to 4 m from each candidate on the true map, stopping at real walls, and counts the cells that were still unknown then, minus the explorer's walking cost (100 cells per meter). It prints, where the VLM picked differently, how much better or worse its pick really was, and how often each pick was the truly best candidate. It only works in the simulator, where the true map exists, and it values a goal by what a full turn there would show, not by what Oak sees on the way.

## VLM test (2026-10-31)

One 8-minute run with the VLM on and one with it off, in the placeholder world with `seed:=1`, using Qwen2.5-VL 7B:

| | VLM `active` | VLM off |
|---|---|---|
| Known map squares at 2 minutes | 27,894 | 27,948 |
| Known map squares at 8 minutes | 28,322 | 28,148 |
| Goals sent / reached | 7 / 5 | 6 / 5 |
| Finished exploring | No | No |

The VLM made no measurable difference, but one run each is too few to say more, and this world's plain gray walls give a camera model little to go on. A world that looks like the real lab would be a fairer test. Both runs stopped mapping after about 2 minutes on `Still stopped: no safe direction on the latest scan`, the latch the [Experiments](experiments.md) page is chasing.

## What it found so far

Before the recovery fixes (master's old Nav2 settings and auto mapper), a 2-run check maps 5.1 % and 5.8 % and never moves: the auto mapper logs "timed out waiting for fresh costmap" after its first scan (first item below).

The first [Scoreboard](scoreboard.md) integration proof on 2026-11-05
reproduced that stop with seed 27. It also proved that the new scorer and this
legacy scorer agree despite the failed exploration: both measured 0.0 m,
0 collisions and 5.7% completeness. Treat that as measurement-pipeline
evidence, not as a successful mapping run.

After pull request #41 merged the costmap and recovery fixes, the same seed was
rerun on 2026-11-05. It traveled 2.8 m, mapped 48.0% and recorded 0 collisions
before the runner declared it stuck after 60 seconds with less than 0.25 m of
new movement. The independent scoreboard measured 2.7235 m, 47.959% and 0
collisions. This confirms the old startup stall is fixed and the measurement
paths agree, but it does not claim that exploration finishes reliably.

On 2026-11-06, a three-run batch demonstrated the remaining variability: two
starts reached 92.0% and 85.2%, while one stalled early. That batch also exposed
a validation bug: the explorer could keep mapping between the legacy snapshot
and scoreboard finalization, and simulator reset jumps inflated legacy distance.
The trial runner now freezes exploration at one measurement boundary, applies
the scoreboard's 1 m jump rejection to truth odometry, and preserves unknown
cells in saved maps. A deterministic seed-27 validation then passed independent
parity at 48.6% completeness, 3.3 m versus 3.119 m, and zero collisions. See
the [experiments journal](experiments.md) for the evidence and limitations.

- **The auto mapper stalls waiting for a costmap.** After Oak stops or spins in place, it waits for a fresh `/local_costmap/costmap`, but Nav2 only re-sends the full grid when its rolling window moves, so it can wait forever ("timed out waiting for fresh costmap"). Fixed with `always_send_full_costmap` (see [Nav2](nav2.md)); still to confirm on Oak.
- **The auto mapper latches stopped next to close obstacles.** Its unstuck move says "no safe direction: rear and both sides lack clearance" and waits. Earlier recovery bugs were fixed (see [Auto mapper](auto-mapper.md)), but repeat runs show a remaining case where Oak is genuinely too close to fit the fixed 0.3 m escape. Shorter bounded recovery moves are the next experiment; no Oak result is claimed yet.
- **Nav2 drives Oak to within 0.05-0.25 m of door jambs, the pillar and furniture**, then reports "No valid trajectories" or "Failed to make progress". TODO(Luis): does Oak get that close on the real floor?
- **`go2.urdf`'s fixed `odom` -> `base_link` joint** puts `base_link` on the floor until odometry starts; a lidar cloud sent before then shows the floor as walls. The stand-in driver now waits for odometry first. The real driver might hit the same thing at startup (see [TF and robot model](tf.md)).

## How it breaks

- **"Gazebo's `ign` command is missing".** The image is older than the simulator: run `docker compose build`.
- **No lidar, startup timeout.** The headless lidar is drawn with OpenGL; without a GPU, Mesa's software renderer (llvmpipe) does it, which works (tested, see the box at the top). If the trial log says `startup timeout: no lidar on /point_cloud2`, look at `logs/run_01_launch.log` for OpenGL errors.
- **Runs slower than real time.** The auto mapper's waits are in wall-clock seconds, so a slow simulator makes it give up early and scores look worse than they are. The trial runner prints a real-time factor per run. Below about 0.9, close the Gazebo window (on a laptop without a GPU it alone halves the speed) or close other programs. Fewer lidar points is not a fix: below 720 per ring, `/scan` gets holes the auto mapper treats as unsafe.
- **It is kinder than the real Oak.** Perfectly flat floor, no people, no glass or mirrors, a cleaner lidar, and a body that slides instead of stepping. A good trial score is a first check, not proof; confirm on Oak and log it in the [Test log](test-log.md).
- **Collisions push, not stop.** Gazebo keeps pushing Oak at the commanded speed while it touches a wall, so one long scrape counts as one bump.
- **The RViz AutoMapper panel works** (it starts the auto mapper in the simulator's domain), but its Reset maps button restarts SLAM, which makes that run's score meaningless.

## Who's on it and what's next

Max added the simulator (pull request #35). Next: Jeth's lab world (issue #28) in place of the placeholder, and real odometry and lidar numbers from Oak for the dials. TODO(owner): an owner for the simulator.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_sim (`launch/sim.launch.py` starts it all; `go2_sim/sim_driver.py` is the stand-in driver; `go2_sim/trials.py` the trial runner; `go2_sim/world_tools.py` builds the true map and scores runs; `worlds/` and `models/oak/` are the Gazebo files.)

Unit tests (no Gazebo needed):

```shell
docker compose run --rm unitree_ros bash -c 'python3 -m pytest /ros2_ws/src/go2_sim/test'
```

The robot build check on GitHub also runs these tests on every pull request (style checks are skipped there for now).

<!-- sources: src/go2_sim/** (this branch), src/go2_robot_sdk/launch/robot.launch.py (driver arg), go2_driver_node.py and ros2_publisher.py (topics, odom frame), urdf/go2.urdf (radar_joint), config/nav2_params.yaml (footprint), config/twist_mux.yaml, docs/wiki/driver.md and test-log.md (lidar rates), docs/wiki/auto-mapper.md ("Exploration complete") -->
