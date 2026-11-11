# Research: differences between the simulator and the real Oak setup

Answers issue #80 (parent map #77). Read from the repo at `origin/master` `f821efb` on 2026-11-11.

Nothing was run. No Docker, no simulator, no robot. Every row is from reading code and wiki pages. Paths are relative to the repo root. `file:12` means line 12.

This file lives on the throwaway branch `max/research/sim-vs-oak`. It is never merged. The repo has no folder for research notes, so it sits in `docs/research/`.

## Short answer

35 differences found.

| Group | Count |
| --- | ---: |
| A. Likely to change how Oak behaves | 17 |
| B. Only changes what is measured | 9 |
| C. Harmless | 9 |

The three that matter most:

1. **What the lidar shows close to Oak (A3, A4, A8).** The simulator makes one clean, level sweep and deletes points on Oak's own body. The real cloud is the robot's own 3D voxel map, its height drifts, and nothing in our code removes body or floor points. On 2026-11-08 the real Oak stopped for "obstacles" 0.05 to 0.20 m past its nose in an empty hallway. The simulator cannot produce that.
2. **Odometry (A1, A2).** The simulator sends Oak's true speed in `/odom`. The real driver never fills in the speed, so it is always zero. The simulator's position error is a guess (1 %, no drift). Nobody has measured the real one.
3. **Body and motion (A9, A10, A11).** The simulated body is a 0.70 x 0.31 m box that slides. Nav2 and the auto mapper plan for a 0.81 x 0.40 m outline. The real Oak walks on legs, and its delay and stop time are unmeasured.

Two things the map (#77, note C) says differ but do not, on master:

- **Start-up file.** The simulator and every lab guide start the same file, `robot.launch.py` (C1).
- **Explorer settings.** All four ways of starting the explorer use the same defaults (C4). They differ only when a person passes an override or runs a local branch (A16).

## What is the same

These are shared, because the simulator includes the real launch file (`src/go2_sim/launch/sim.launch.py:150-155`):

- Nav2 settings, including speed limits 0.5 m/s and 1.0 rad/s, and the 0.81 x 0.40 m footprint (`src/go2_robot_sdk/config/nav2_params.yaml:157-170`, `:217`, `:265`, `:387-390`).
- SLAM settings (`src/go2_robot_sdk/config/mapper_params_online_async.yaml`).
- twist_mux priorities (`src/go2_robot_sdk/config/twist_mux.yaml`).
- How `/scan` is cut from the cloud: source `/point_cloud2`, 0.1 m below to 0.5 m above `base_link`, nothing closer than 0.1 m (`src/go2_robot_sdk/launch/robot.launch.py:171-196`).
- The Python `lidar_processor` nodes (`robot.launch.py:215-241`).
- `use_sim_time` is false for the whole stack in both (`sim.launch.py:148-149`, `robot.launch.py:101`, `nav2_params.yaml:3`).
- Lidar mount position: `src/go2_sim/config/sim.yaml:24-26` equals `src/go2_robot_sdk/urdf/go2.urdf:1264-1267`.
- Explorer default parameters (`src/go2_auto_mapper/go2_auto_mapper/explorer.py:86-163`).

## A. Likely to change how Oak behaves (17)

| # | What | Simulator | Real Oak | On purpose? |
| --- | --- | --- | --- | --- |
| A1 | Speed inside `/odom` | True speed from Gazebo is copied in. `src/go2_sim/go2_sim/sim_driver.py:171-172`, `:203-204` | Never set, so always zero. `src/go2_robot_sdk/go2_robot_sdk/infrastructure/ros2/ros2_publisher.py:79-102`. Nav2 reads this topic (`nav2_params.yaml:55`) | No comment or wiki line mentions it |
| A2 | Position error in odometry | True pose plus made-up noise: 0.01 m per sqrt(m), 0.01 rad per sqrt(rad), no scale error, no drift. `sim.yaml:8-12`, `sim_driver.py:158-170` | The robot's own pose estimate, topic `rt/utlidar/robot_pose`. `.../domain/constants/webrtc_topics.py:21`, `.../application/services/robot_data_service.py:32`. Real error never measured | Placeholder. `sim.yaml:3` and `docs/wiki/simulator.md:210` both say TODO, measure on Oak |
| A3 | Height and tilt of `base_link` | Fixed 0.32 m up, always level, only heading changes. `sim.yaml:14`, `sim_driver.py:190-193` | Robot's z plus 0.07 m, full roll and pitch. `ros2_publisher.py:68-75`. Height drifts, which smears the floor into the cloud. `robot.launch.py:181-185`, `nav2_params.yaml:228-231` | Half. `simulator.md:274` admits a "perfectly flat floor". No line says the missing drift is on purpose |
| A4 | What a lidar cloud is | One live sweep per message: 16 rings, 720 points each, 15 degrees down to 30 degrees up. `src/go2_sim/models/oak/model.sdf.template:72-108`, `sim_driver.py:208-247` | The robot's own 3D voxel map, decoded, weak points dropped, duplicates removed. `webrtc_topics.py:19`, `robot_data_service.py:27`, `.../infrastructure/sensors/lidar_decoder.py:52-63`. It "accumulates in odom": `nav2_params.yaml:228-229` | Not stated. `simulator.md:274` only says "a cleaner lidar" |
| A5 | Lidar range | 12 m. `sim.launch.py:36`, `sim.yaml:27`, `sim_driver.py:222` | The stack is set up for about 20 m. `nav2_params.yaml:17`, `:240-242`, `:292-294`, `mapper_params_online_async.yaml:33`. The real lidar's range is not written anywhere in the repo | No reason given for 12 |
| A6 | Lidar rate | Steady 5 per second. `sim.launch.py:35` | Cloud 3 to 6.8 per second, `/scan` 3.2 to 5. `docs/wiki/driver.md:44`, `:49`, `docs/wiki/test-log.md:27`, `:46` | Yes. Picked from the lab numbers. `simulator.md:199` |
| A7 | Lidar noise and surfaces | 0.01 m random error, every wall reflects. `sim.launch.py:38`, `model.sdf.template:102-106` | Unknown noise. Glass, mirrors, people | Admitted. `simulator.md:274` |
| A8 | Points on Oak's own body | Deleted inside a box 0.32 m front and back, 0.18 m each side. The lidar also cannot see Oak's drawn legs. `sim.yaml:28-29`, `sim_driver.py:79-82`, `:225-226`, `model.sdf.template:79-82` | No filter in our code. A comment says the real cloud has none, "measured": `robot.launch.py:188-189`. But on 2026-11-08 Oak saw returns 0.05 and 0.20 m past its nose in an empty hallway and froze. Cause unknown. `test-log.md:31` | Yes in the simulator. Open question on Oak |
| A9 | Body size | The body that can hit things is 0.70 x 0.31 m, centred on `base_link`, from 0.08 m to 0.40 m high. `model.sdf.template:28-35`, `src/go2_sim/go2_sim/world_tools.py:51-57` | Nav2 and the auto mapper plan for 0.81 x 0.40 m: nose 0.36 m ahead, tail 0.45 m behind, 0.20 m each side. `nav2_params.yaml:217`, `:265`, `src/go2_auto_mapper/go2_auto_mapper/recovery.py:13-19`, `obstacle_sensing.py:15-20`. The same numbers run in the simulator. The real body has moving legs | Partly. `model.sdf.template:28` says the box is the real Go2 size. `recovery.py:68-70` says Nav2's outline is "a little larger" than the body. Nothing explains the 0.10 m tail gap or the centred box |
| A10 | How it moves | A box that slides at the asked speed, sideways too, no friction. Caps 1.0 m/s, 0.6 m/s sideways, 2.0 rad/s. Speed-up caps 1.5 m/s2 and 3.0 rad/s2. `model.sdf.template:20-25`, `:122-126`, `sim.yaml:18-22`, `sim_driver.py:128-138` | Walks on legs. The driver rounds each command to 2 decimals and sends it on with no cap. `.../infrastructure/webrtc/webrtc_adapter.py:192-200`. The robot's own limits are not in the repo | Yes. "No legs, on purpose": `simulator.md:36` |
| A11 | Command delay and stop time | 0.1 s delay. Stops after 0.5 s with no command. `sim.yaml:16-17` | Unknown. One number: 0.11 m of travel in the 2 s after a stop. `test-log.md:52` | Placeholder. TODO in `sim.yaml:3` and `simulator.md:210` |
| A12 | Clock speed | The explorer's waits count real seconds (`explorer.py:2437`, `recovery.py:22-33`). A slow laptop makes the simulator run slower than real time, so waits end early. `simulator.md:273`. Each run records its speed factor: `src/go2_sim/go2_sim/trials.py:222-226` | Always real time | Known limit, written down |
| A13 | The room | `placeholder_rooms`: a made-up 10 x 8 m flat. `ite_hallways`: guessed sizes. Flat floor, no people. `simulator.md:49`, `:53-59` | The real lab | Placeholder. A tape-measured lab world is issue #28 |
| A14 | Where Oak starts | Random spot with 0.7 m of free floor, fixed by the seed. `trials.py:605`, `:653-655`, `world_tools.py:414-427`. Single runs default to x 2.5, y 2.0. `sim.launch.py:29-31` | Wherever a person puts Oak down. "Large clear area": `docs/wiki/scoreboard-oak-lab-test.md:31` | Yes for the simulator |
| A15 | First seconds after start | The stand-in driver holds back lidar clouds until odometry has started, or SLAM draws fake walls. `sim_driver.py:105-109`, `:209-210` | The real driver has no such guard. `ros2_publisher.py:185-215` | Unclear. The code comment says the real driver "has odometry before its first cloud" (`sim_driver.py:108`). The wiki says the real driver "might hit the same thing" (`simulator.md:267`). They disagree |
| A16 | Explorer settings per batch | The trial runner can change any explorer setting for a batch. `trials.py:74-82`, `:614-616`. Some logged runs came from local branches with edited code, for example `blocked_timeout` 2.0 while master has 0.0. `docs/wiki/experiments.md:169`, `explorer.py:147` | The lab test ran defaults from branch `noah/feat/scoreboard`. `scoreboard-oak-lab-test.md:10`, `:110` | Yes, it is the experiment tool. But it means a sim score and an Oak score may come from different settings |
| A17 | Other programs sharing the laptop | RViz, Foxglove and the gamepad are off. No camera unless asked. No 3D map file saved. Gazebo draws the lidar on the CPU. `sim.launch.py:153-155`, `:87`, `simulator.md:32` | RViz, Foxglove and gamepad on by default, camera video decoded, 3D map file saved every 10 s. `robot.launch.py:87-92`, `:28`, `.../presentation/go2_driver_node.py:92` | Yes. "Minus the extras a sim does not need": `sim.launch.py:148`. Effect on timing not measured |

## B. Only changes what is measured (9)

| # | What | Simulator | Real Oak | On purpose? |
| --- | --- | --- | --- | --- |
| B1 | When a run stops | Time limit 900 s. Stall rule: under 0.25 m of true movement in 180 s. Start-up limit 180 s. `trials.py:597-603`, `:461-470`. Past batches used other limits: 300 / 180 / 120 s, then 60 s, and 600 s in the hallway script. `docs/wiki/simulation-run-settings.md:19`, `:89`, `scripts/run_ite_hallways.ps1:7`, `:40-43` | A person presses Ctrl+C after about 30 to 60 s. No time limit, no stall rule. `scoreboard-oak-lab-test.md:113-117` | Yes for both. The changing limits between batches are not explained |
| B2 | How "finished" is decided | The old scorer looks for the words "Exploration complete" in the log. `trials.py:40`, `:167-169`, `:471-482` | The scoreboard closes on the explorer's "completed" or "stopped" event, or when a person asks. `src/go2_scoreboard/go2_scoreboard/recorder.py:178`, `:224-233` | Yes. The old scorer is kept as a cross-check |
| B3 | Collisions | A touch sensor on the body box, read 20 times a second. A new bump counts after 1 s with no contact. `model.sdf.template:112-119`, `trials.py:43`, `:189-195`, `recorder.py:157-163`, `:196-202`. Gazebo keeps pushing, so one long scrape is one bump: `simulator.md:275` | A person calls a "mark collision" or "mark human touch" command. `scoreboard-oak-lab-test.md:119-129`, `recorder.py:128-130` | Yes. Oak has no bumper |
| B4 | Map completeness | Scored against a true map made from the world file. `trials.py:409`, `:507-509`, `world_tools.py:22-23` | No true map, so the score is empty. `scoreboard-oak-lab-test.md:23-25` | Yes. Waits on a measured lab map |
| B5 | Distance | Two numbers: true distance and odometry distance. `trials.py:197-210`, `:496-502` | Odometry distance only. `recorder.py:181-184` | Yes |
| B6 | The scoreboard's clock | Started with `use_sim_time:=true`. `trials.py:404`. The rest of the stack uses the normal clock | Not set, so the normal clock. `scoreboard-oak-lab-test.md:92` | No comment explains it. Effect not checked |
| B7 | What is saved about the run | Source `sim`, a label with variant and seed, start pose, and a settings record with launch arguments and explorer overrides. `trials.py:402-420` | Source `oak` and a hand-typed label. Settings record is empty. Start pose is 0, 0, 0. `scoreboard-oak-lab-test.md:92`, `recorder.py:47-58` | Not stated. In both, code version is "unknown" inside Docker: `recorder.py:56`, `docker/docker-compose.yml:6-12` |
| B8 | Where results land | `src/go2_sim/results/`. `trials.py:85-90` | `src/go2_scoreboard/results/`. `scoreboard-oak-lab-test.md:167` | Not stated. Git ignores both: `src/go2_sim/.gitignore:1`, `src/go2_scoreboard/.gitignore:1` |
| B9 | Number of scorers | Two, plus a check that they agree. `trials.py:304-325` | One | Yes |

## C. Harmless (9)

| # | What | Simulator | Real Oak | On purpose? |
| --- | --- | --- | --- | --- |
| C1 | Start-up file | `sim.launch.py` includes `robot.launch.py` with `driver:=false`, and RViz, Foxglove and gamepad off. `sim.launch.py:150-155` | `robot.launch.py`, the Docker default. `docker/Dockerfile:80`, `docs/wiki/lab-checklist.md:30`, `scoreboard-oak-lab-test.md:61`, `docs/wiki/wsl.md:112` | Yes. "The real stack, unchanged, minus the driver": `sim.launch.py:148` |
| C2 | Other launch files build `/scan` differently | Not used | Not used by any lab guide. `robot_cpp.launch.py:179-192`: from `/pointcloud/filtered`, 0.2 m below to 2.0 m above. `mapping.launch.py:122-134`: 1 m below to 3 m above, nothing closer than 0.3 m. `navigation.launch.py:145-157`: 0.2 m below to 2.0 m above | Drift, not a choice. `docs/wiki/lidar.md:61` warns about it. It becomes a real difference only if someone starts one on Oak |
| C3 | Python or C++ lidar code | Python. `robot.launch.py:215-241` | Python. C++ runs only in the three files in C2. It lacks the reset service, the refresh squares and saving in the map frame. `src/lidar_processor/lidar_processor/lidar_to_pointcloud_node.py:139`, `:167`, `:171` against `src/lidar_processor_cpp/src/lidar_to_pointcloud_node.cpp:109-115`. `lidar.md:63` | Yes, C++ is a variant. Neither is in the `/scan` path under `robot.launch.py` (`:176`) |
| C4 | How the explorer is started | Three ways. RViz panel: the program with no settings, `src/go2_rviz_plugins/src/auto_mapper_panel.cpp:209-216`. `explore:=true`: a launch node that passes `vlm_mode`, `sim.launch.py:168-175`. Trials: `ros2 run`, plus overrides if asked, `trials.py:77`, `:444` | `ros2 run go2_auto_mapper explorer`, no settings. `scoreboard-oak-lab-test.md:110`, `docs/wiki/commands.md:68`. The lab checklist never mentions the explorer | Yes. "Exactly as the RViz panel does": `trials.py:443`. Defaults are the same |
| C5 | Odometry rate | 15 per second. `sim.yaml:13` | 13 to 20 per second. `driver.md:43`, `test-log.md:27` | Yes, set near the measured rate |
| C6 | Network | ROS domain 42, or 50 and up in trials. Local machine only. `sim.launch.py:50`, `:81-84`, `trials.py:345-348` | Default domain, host network. `docker-compose.yml:20` | Yes. So sim commands can never reach Oak: `simulator.md:47` |
| C7 | 3D map file | Not saved. `sim.launch.py:87` | Saved every 10 s. `robot.launch.py:28` | Yes |
| C8 | Missing senses | No `/imu`, no `/go2_states`. Camera off unless asked. Legs frozen in a standing pose. `simulator.md:45`, `sim.launch.py:39-40`, `:98`, `sim_driver.py:37`, `:250-255` | All present | Yes |
| C9 | Robot address | `ROBOT_IP=sim`, no connection. `sim.launch.py:85-86` | The real address, over WebRTC | Yes |

### Environment defaults the explorer reads

From `explorer.py:118-133` and `:936`: `VLM_MODE` (default `off`), `VLM_URL`, `VLM_MODEL`, `NAV_LOG_DIR` (default empty, so no log) and `VLM_API_KEY`. The Docker compose file sets none of them (`docker-compose.yml:6-12`). So they are unset in both the simulator and the lab, unless a person exports them.

One trap, found by reading: in trials, `-- vlm_mode:=active` turns the simulated camera on (`sim.launch.py:98`) but does not reach the explorer. The explorer only gets `vlm_mode` from the launch file when `explore:=true` (`sim.launch.py:168`), and trials always pass `explore:=false` (`trials.py:355`). To turn the VLM on in trials it has to be `--explorer-param vlm_mode:=active` or the `VLM_MODE` variable.

## Wiki lines that look wrong

- `docs/wiki/lidar.md:61` says `navigation.launch.py` uses a 0.3 m minimum and a 1 m below to 3 m above band. The file has 0.1 m and 0.2 m below to 2.0 m above (`navigation.launch.py:150-156`). Only `mapping.launch.py` has the other values.
- `sim_driver.py:108` and `simulator.md:267` disagree on whether the real driver can send a cloud before odometry (A15).

## Not verified, because nothing was run

- A1: that the empty speed field changes how Nav2 drives. The Nav2 source is not in this repo.
- A5: that 12 m against 20 m changes the map in the lab's rooms.
- A9: that the smaller simulated body lets Oak fit through gaps the real Oak cannot.
- A12: how far below real time the trials really ran. The factor is in each batch's CSV, which sits on a laptop.
- A17: that laptop load changes lidar or command timing.
- B6: what the mixed clocks do to the scoreboard's times.
- The live value of every setting. This is what the files declare, not a dump from running nodes.
- The default settings of `pointcloud_to_laserscan` that `robot.launch.py` leaves unset (farthest range, angle step). They are the same in both setups, but the values come from a package outside this repo.

## Only a person with Oak in the lab can confirm

1. Does the real cloud have points on Oak's own body or the floor near its feet? What were the close returns on 2026-11-08 (A8)?
2. How wrong is real odometry after a known walk and a known turn (A2)?
3. How much does `base_link` height drift and tilt while walking (A3)?
4. How long from a command to movement, and how long to stop when commands end (A11)?
5. What top speed and turn rate does Oak really reach when asked for 0.5 m/s and 1.0 rad/s? Does it step sideways at 0.10 m/s as the recovery move expects (A10)?
6. How far does the real lidar see, and how noisy is it (A5, A7)?
7. Tape-measure the body, legs included. Where is `base_link` between nose and tail (A9)?
8. Does the first cloud arrive before the first odometry message at start-up (A15)?
9. Which exact commit, start spot and settings did each past Oak run use (B7)?
