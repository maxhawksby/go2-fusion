---
title: Test log
parent: lab-setup
---

# Test log

The test log is Oak's lab notebook: one section per session with the real robot, recording what we tested, whether it passed, and what is still broken.

Part of: [Lab setup](lab-setup.md)

## How it works

This page is the log itself. Newest session first.
Each test gets one of these results: **pass**, **fail**, **partial**, or **not run**, plus a short note on what we saw.
An **Open issues** list carries over between sessions until each issue is fixed and retested on Oak.
During a session, the note-taker (often Claude, from Max's feedback and its own measurements) updates this page and commits it at the end.

It matters because robot time is scarce: one dog, one connection at a time. The log tells you what is already known, so you test what is new.

## 2026-11-08 - ISL lab, Oak, Wi-Fi (WebRTC)

Code: `noah/feat/scoreboard` at `843ad4a`.

| Test | Result | What we saw |
| --- | --- | --- |
| WebRTC and sensing preflight | pass | Oak validated once; `/odom` was about 20 Hz, `/scan` about 3.2 Hz, `/map` had one slam_toolbox publisher, and Nav2 exposed one `/navigate_to_pose` server. |
| First physical scoreboard run | pass | The ready gate succeeded before exploration. A 57.169 s run recorded 3.3359 m travel and 12.955 square meters of known map, two failed goals, one recovery, zero collisions, zero touches and no warnings. Completeness stayed null because no measured answer map exists. |
| Evidence finalization | pass | JSON, scorecard/timeline/resource CSVs, Markdown report, PGM/YAML map and a 4.29 MB rosbag finalized on the host. |
| Stop after the run | pass | Ctrl+C stopped the explorer. Cancel-all found no remaining Nav2 goals and `/cmd_vel_out` produced no active command during a five-second check. |
| Auto mapper in a clear hallway | **fail** | Oak reported obstacles 0.20 m and 0.05 m beyond its nose, failed two goals, attempted one open-space recovery and latched at `no safe direction`, although the operator saw no obstacle. The recorded raw scan did contain close returns. The bag lacked `/local_costmap/costmap`, so the next diagnostic run must record it to distinguish scan/self/floor returns from costmap or TF errors. |

No second autonomous run was attempted. The priority is now to diagnose the
false or misinterpreted close returns before tuning frontier selection or
testing a different exploration algorithm.

## 2026-10-11 - ISL lab, Oak, Wi-Fi (WebRTC)

Code: `master` at `61de971` (fusion + Luis's Oct 9-11 work), then branch `fix/keyboard-panel-idle`.
Earlier tests ran on `f85e703` (fusion before Luis's update); marked "old code" below.

| Test | Result | What we saw |
| --- | --- | --- |
| Build in Docker | pass | The first attempt ran out of memory and crashed WSL (7.4 GB, 16 parallel compiles, two builds at once). It built fine with `MAKEFLAGS=-j4 colcon build --parallel-workers 2`. |
| Connect, camera, RViz | pass | |
| Lidar `/point_cloud2` | pass | About 6.8 Hz. The 3D voxel view in RViz works. |
| Lidar topic mismatch | check | The driver publishes `/point_cloud2`, nothing publishes `/robot0/point_cloud2`, and `lidar_to_pointcloud` wasn't running in the default launch. Luis's remap is in; check which launch option starts that node. |
| Keyboard teleop (terminal) | partial | The dog moves and stops, but it "really isn't that good". Likely cause: the bugs below (stops dropped, idle zeros). Retest. |
| YOLO `object_detection` | pass | About 4.5 fps on CPU, 55-130 ms per frame. Saw person 0.86 and chair 0.34. View it at `/detections/image_annotated`. |
| twist_mux priority | pass | Object nav forward, then keyboard turn wins, then object nav resumes, then stop. Checked on `/cmd_vel_out`. |
| Stop at end of drive (old code) | **fail** | **The dog kept walking and hit a wall.** The driver dropped every zero `cmd_vel`, so stop never reached the robot. This is fixed by Luis's `robot_control_service.py` change, now on `master`. |
| Stop at end of drive (new code) | pass, confirm | Stop was delivered. Odom shows the dog moved 0.11 m in the 2 s after the stop (slowing down, not walking on). Max to confirm by eye. |
| RViz KeyboardTeleop panel idle | **fail, fixed** | The panel sent zeros about 4 times a second on `/cmd_vel_keyboard` (priority 9), which blocked Nav2, the agent and object nav. The terminal `keyboard_teleop` did the same at 10 Hz. Both now publish only while a key is held, plus one stop. On `fix/keyboard-panel-idle`. |
| Nav2 goal (old code) | fail | "Starting point in lethal space": the lidar marks the dog's own body as an obstacle. Luis's costmap `min_range: 0.1` should fix it. Retest. |
| Data stall after Ctrl+C | note | Camera kept streaming, but lidar and odom went silent. `docker compose -p go2fusion up -d --force-recreate` recovered it. |
| Gamepad LB deadman | not run | No Xbox-style gamepad. The lab uses Unitree's R3-1 remote, which bypasses ROS. |
| Not run yet | - | MapViewer click-to-walk, object nav to a chair, sit/stand, AutoMapper, TTS (needs an ElevenLabs key), Claude agent (not built). |

## Open issues

1. Retest Nav2 goals and MapViewer click-to-walk on new code.
2. Retest keyboard driving feel with both teleop fixes.
3. Find out what starts `lidar_to_pointcloud` and whether its input topic is right.
4. Confirm by eye that Oak stops cleanly when a drive source goes quiet.
5. WSL has 7.4 GB RAM: keep builds at `-j4` / 2 workers, or raise `memory=` in `.wslconfig`.
6. Gamepad LB deadman: no longer required. The team no longer uses an Xbox gamepad; it drives with the Oak ROS2 pad, which sends signals differently. The pad talks straight to Oak and skips ROS (Max, 2026-10-22).

Still to check from the fusion follow-ups (see [Decisions](decisions.md)): the driver reconnects after a Wi-Fi drop, TTS with the new 4 KB chunks, each twist_mux input actually reaching the driver, the sit message, the RViz panels, and `CONN_TYPE=webrtc` as the compose default.

## Connects to

- [Lab checklist](lab-checklist.md): how to get connected for a session.
- [Roadmap](roadmap.md) and the [Oak](../../README.md) front page: open issues show up there.
- Program pages such as [Keyboard teleop](keyboard-teleop.md), [Nav2](nav2.md) and [Go2 driver](go2-driver.md) quote these results in their "How it breaks" sections.

## How it breaks

- **It goes stale.** The last recorded session is 2026-10-11. Luis's auto mapper work and later fixes have no logged robot session yet. TODO(owner): add sessions since then.

## How to add a session

Add a new `## <date> - <place>, Oak, <Wi-Fi or Ethernet>` section above the newest session, name the code you ran (branch and commit), then one table row per test. Never write network names, addresses or keys in it. Move fixed items out of Open issues only after a retest on Oak.

## Who's on it and what's next

Max keeps the log. Next: the retests above, and Luis's full exploration tests with the auto mapper.

<!-- sources: docs/TEST_LOG.md (whole file, folded in here and deleted 2026-10-22); docs/FUSION.md "Follow-ups for the team" 4; README.md open issues; team update 2026-10-18 (Luis next: full exploration tests); git log c18ca87 -->
