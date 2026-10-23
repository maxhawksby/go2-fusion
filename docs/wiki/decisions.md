---
title: Decisions
parent: team
---

# Decisions

This page explains the big choices behind Oak's code and why we made them, so you never have to guess why something is the way it is.

Part of: [Team](team.md)

It is the team's full decision log: how the repo was born (the fusion), every fusion decision, what was left out, what was decided since, and the follow-ups still open.

## How this repo was born: the fusion

This repo combines two codebases for our Unitree Go2 robot dog, Oak:

- **Our team fork** of the Go2 ROS 2 SDK ([LuisMend12/go2_ros2_sdk](https://github.com/LuisMend12/go2_ros2_sdk), branch `master` at `cd7be62`). It keeps the full upstream history, our Docker dev workflow, WSL fixes, AES-key support for new firmware, and our own packages.
- **Riley Francis's final repo, [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav)**, copied from commit `a85fe39` (2026-09-28, Riley's last commit). Riley built the YOLO object detector, the object-following node, the LLM command agent and the speech-to-text tool. Thank you, Riley.

How it was done: our fork is the base, because Riley's repo carried older copies of the same driver packages, missing newer upstream fixes. Riley's own packages were copied in unchanged, then changed only as the table below says.
The repo starts with a **fresh git history** (one first commit), so old commits containing lab access details never leave the team fork. For the full history before the fusion, see the team fork and Riley's repo.
(The local clone of Riley's repo had one later commit, `482e346` by Luis, which only adds an empty `COLCON_IGNORE` file; nothing was taken from it.)

## Every fusion decision

| # | What | Choice | Why | Files |
| --- | --- | --- | --- | --- |
| 1 | Object detector | Bring in Riley's `object_detection` (YOLOv8). Its pose topic is renamed `/detected_objects` to `/detected_objects/poses`. Keep `coco_detector` for now. Declare its missing `cv_bridge` and `geometry_msgs` deps. Add `ultralytics==8.4.174` to `requirements.txt`. | YOLO is more accurate and gives rough positions. `coco_detector` already uses `/detected_objects` with a different message type, and two types on one topic break it. `ultralytics` installs in the same pip command as our CPU-only torch, so it does not pull the multi-GB CUDA torch (checked: torch stays `2.14.1+cpu`). | `src/object_detection/`, `requirements.txt` |
| 2 | Drive-command collisions | Every drive source gets its own twist_mux input (table below). Changed: `keyboard_teleop`, the RViz KeyboardTeleop panel, `go2_llm_agent`, `object_navigation`. | Before, the keyboard, the agent and object navigation all published `/cmd_vel`, the same input as Nav2, so whoever published last won. Now manual driving beats autonomy, and the node with no obstacle avoidance yields to everything. | `src/go2_robot_sdk/config/twist_mux.yaml`, `src/keyboard_teleop/`, `src/go2_rviz_plugins/src/`, `src/go2_llm_agent/`, `src/object_navigation/` |
| 3 | `go2_llm_agent` | Brought in as-is, except the topic change (2) and the sit fix (11). Its pip deps (langchain, langgraph, fastapi, ...) are **not** added to the image. | Its replacement was still being decided (see "Decided since"). It builds, but does not run. | `src/go2_llm_agent/` |
| 4 | `object_navigation` | Brought in, on the lowest twist_mux priority. | Useful demo of "walk to the chair", but it has no obstacle avoidance. | `src/object_navigation/` |
| 5 | `speech_processor` | Take Riley's 4 KB / 0.2 s audio chunks (was 16 KB / 0.15 s), his fix that sends the topic name `rt/api/audiohub/request` instead of the number `1003`, his removal of two entry points whose files never existed, and his `tts_cache/` .gitignore. | Riley tuned the chunking on the robot; the other three are plain bug fixes. | `src/speech_processor/` |
| 6 | Docker | Keep our dev workflow (deps-only image, mounted source, build on start). Default `CONN_TYPE` is now `webrtc`; added `shm_size: 1g`. Same in `docker-compose.windows.yml`. | The lab uses Wi-Fi. Docker's default 64 MB of shared memory can run out with ROS 2 DDS and camera images. | `docker/docker-compose*.yml` |
| 7 | RViz config | Keep ours (Map display off). | RViz's Map display crashes under WSL's software renderer. Luis's MapViewer panel covers it. | (unchanged) |
| 8 | Riley's extras | Skip `vlmap_go2` and the `vlmaps` / `gaussian-splatting` submodules. Bring only `vad_listen.py` + its requirements to `host_tools/speech_to_text/`. Fold Riley's docs into our docs instead of copying `docs/`. | See "Left out" below. | `host_tools/speech_to_text/` |
| 9 | Luis's RViz panels branch | Merged `fix/ros2-llm-nav-submodule` (new `go2_rviz_plugins` package + a command cheat sheet, now the [Command cheat sheet](commands.md) page). Fixed the cheat sheet's Windows-only paths and container names. | Finished work that was not on master yet. Its dependencies were already declared, and it builds. | `src/go2_rviz_plugins/` |
| 10 | Fast lidar branch | Not merged (`fix_hz_lidar_stream`). | It also deletes `coco_detector`, Nav2/SLAM configs and more, and no longer applies to the refactored driver. See the follow-ups. | (none) |
| 11 | Sit message | Standardize on our `parameter="{}"`. Riley's agent sent `""`. | Matches `front_leg_controller`'s sit_controller. Only the agent's sit was changed; its other sport commands still send `""`. | `src/go2_llm_agent/go2_llm_agent/` |
| 12 | `.gitmodules` | Removed the leftover `ROS2-LLM-Nav` entry. The `aioice` submodule stays at `src/go2_robot_sdk/external_lib/aioice`. | The submodule pointer was deleted earlier (`6ca4977`) but its entry was left behind. | `.gitmodules` |
| 13 | Gamepad | Gamepad publishes on `/cmd_vel_joy` (priority 10) and only while LB is held (deadman button). Remapped in all four launch files that start it. | An idle gamepad repeats "speed 0" 20 times a second. On the shared `/cmd_vel` input that froze the agent and object navigation, and the keyboard outranked the gamepad. Tested in Docker with simulated input. | `src/go2_robot_sdk/config/twist_mux.yaml`, `src/go2_robot_sdk/launch/` |
| 14 | Lab access details | Removed from README. | They don't belong in a repo. Ask a teammate for them privately. This is also why the repo starts with a fresh history. | `README.md` |
| 15 | Team fork, Oct 9-11 | Ported Luis's 6 commits after `7a3d4a9` (`2fee8d1` to `d0a0ba5`): driver auto-reconnect with backoff, mid-session WebRTC drop detection, explicit stop on a zero `cmd_vel`, video survives bad frames, lidar decoder buffer check, `sit_controller` waits for the driver, lidar remap `robot0/point_cloud2` to `point_cloud2` in `robot.launch.py`, `cyclonedx` typo fixed to `cyclonedds`, `webrtc_web.launch.py` TTS node fixed, Nav2 `use_sim_time: False`, RViz panels in the default config, Julia pinned to 1.13.1, Foxglove port back to 8765 (override with `FOXGLOVE_PORT`), coco_detector rate limit and auto device, lidar processor fixes, `src/front_leg_controller/README.md`. Changed on the way in: (a) Luis's gamepad remap duplicated ours, kept one; (b) `twist_mux` starts if `joystick` **or** `teleop` is on (Luis gated it on `joystick` only, which would cut off keyboard, Nav2, agent and object navigation with `joystick:=false`); (c) each reconnect gets its own drop signal, so a late close from the old connection can't kill the new one; (d) a failed reconnect cancels all robots' control loops before retrying; (e) kept `shm_size: 1g` alongside the Foxglove port fix; (f) `coco_detector` keeps only the newest camera frame, so its new rate limit actually stops it falling behind on CPU. | Keep the fusion current with the team's work. | see commit |
| 16 | Idle keyboard zeros | The RViz KeyboardTeleop panel and `keyboard_teleop` publish only while a key is held, plus one stop. | Found on the robot 2026-10-11 (see [Test log](test-log.md)): both sent zeros nonstop on `/cmd_vel_keyboard` (priority 9), which blocked Nav2, the agent and object navigation. Same bug as the idle gamepad (row 13). | `src/go2_rviz_plugins/src/keyboard_teleop_panel.cpp`, `src/keyboard_teleop/` |
| - | Docs | README, AGENTS.md, CONTRIBUTING.md, the Windows guide and package READMEs updated for the new packages and topics. | Keep the docs true. | docs |

Row 13 today: the code still works this way, but the team now drives with the Oak ROS2 pad instead of an Xbox gamepad, so testing LB is no longer required (see [Gamepad](gamepad.md)).

## twist_mux priorities

Defined in `src/go2_robot_sdk/config/twist_mux.yaml`. [twist_mux](twist-mux.md) forwards the highest-priority input that has published in the last 0.5 s to the driver (`cmd_vel_out`). People outrank robots:

| Priority | Topic | Who publishes it |
| --- | --- | --- |
| 10 | `/cmd_vel_joy` | Gamepad, only while LB (deadman button) is held |
| 9 | `/cmd_vel_keyboard` | `keyboard_teleop`, RViz KeyboardTeleop panel |
| 6 | `/cmd_vel_mapper` | [Auto mapper](auto-mapper.md) recovery moves (backing up or sliding when stuck; added after the fusion) |
| 5 | `/cmd_vel` | Nav2 (also the auto mapper's goals and the map viewers, through Nav2) |
| 4 | `/cmd_vel_agent` | [LLM agent](llm-agent.md) |
| 3 | `/cmd_vel_object_nav` | [Object navigation](object-navigation.md) |

A new drive source should get its own row here rather than sharing a topic.

## Left out, and why

- **`vlmap_go2`**: an early experiment that only republishes `/pointcloud/filtered` (it projects points without any frame transform, then drops the color). No language features yet.
- **`vlmaps` and `gaussian-splatting` submodules**: nothing in Riley's ROS code uses them, and both need an NVIDIA GPU with CUDA, which our CPU-only image does not have. Worth revisiting only for a real semantic-mapping effort.
- **Riley's music / YouTube / play-file audio scripts** (`src/speech_processor/scripts/`, `play_audio.py`): standalone demo tools, not part of the robot stack.
- **`host_tools/speech_to_text/audio_server.py` and `speech_input_prototype.py`**: older stubs replaced by `vad_listen.py`.
- **Riley's `docs/` folder**: most of it describes packages we already document. The parts about his packages now live on their wiki pages.
- **Riley's copies of `go2_robot_sdk`, `go2_interfaces`, `lidar_processor(_cpp)`, `speech_processor`**: ours are newer (for example, AES-key support for firmware 1.1.15+). Only his speech fixes were taken.
- **Riley's Dockerfile and compose setup**: ours avoids rebuilding the image for every code change and uses CPU-only torch. Only `shm_size` was taken.
- **Riley's `setup.cfg` pytest block for `go2_robot_sdk`**: it forces a coverage flag that breaks `colcon test`.

## Decided since

- **2026-10-08: rebuild the LLM agent on Claude.** Riley's agent needed close to a rewrite either way (fixed plans, lost turn direction, no stop, broken live object lookup), and Claude's tool use would do it in less code with fewer dependencies. **Superseded and dropped** by the next decision; its build plan was deleted on 2026-10-22 (git history keeps it).
- **Ollama agent direction** (Adib's branch, dated 2026-10-12; confirmed as the team's direction on 2026-10-22). The agent runs a free local Qwen model through Ollama, so it needs no paid API key, no billing account and no extra Python libraries. Simulation is the default; real moves reuse `/cmd_vel_agent`. Voice and object navigation are out of scope for this first step. Until it merges, the agent on `master` builds but `ros2 run go2_llm_agent ...` fails on missing imports. See [LLM agent](llm-agent.md).
- **2026-10-22: the wiki is a graph.** `README.md` is the center page, Oak. Hubs and one page per program live in `docs/wiki/`, and a check fails when code changes without its page.
- **2026-10-22: one home for team knowledge.** Explanations live only in `README.md` and `docs/wiki/`. The old guides (test log, lab checklist, this fusion record, the WSL, Windows and Mac guides, the command cheat sheets) were folded into wiki pages and deleted; READMEs next to code keep only how to build, run and test.
- **2026-10-22: review only for merges into `master`.** Your own branches are a sandbox; merging into `master` needs one teammate's approval and green checks.

## Follow-ups for the team

1. **Lidar topic mismatch (verify on the robot first).** Luis's remap (row 15) now fixes this for `robot.launch.py`; confirm on the robot and check `robot_cpp.launch.py`. `lidar_processor`'s `lidar_to_pointcloud` subscribes to `/robot0/point_cloud2` in single-robot mode, but the driver publishes `/point_cloud2`. Check on the robot before changing it.
2. **Faster lidar decoder.** The `fix_hz_lidar_stream` branch swaps the WASM lidar decoder for an LZ4 one (claimed about 2 Hz to about 62 Hz). Do not merge the branch (it deletes too much); if low lidar rate hurts mapping, port just its `lidar_decoder_lz4.py` into the current driver.
3. **Retire `coco_detector`.** Once `object_detection` also publishes standard `vision_msgs/Detection2DArray`, remove `coco_detector` and decide whether YOLO should take over `/detected_objects`.
4. **Robot re-test.** Partly done: the 2026-10-11 lab session in the [Test log](test-log.md) passed connect, camera, RViz, lidar, YOLO speed on CPU and the twist_mux priority order, and found the idle keyboard bug (row 16). Still to check: the driver reconnects after a Wi-Fi drop, a drive source going quiet stops the dog (zero `cmd_vel` is now sent), TTS with the new 4 KB chunks, each twist_mux input actually reaching the driver, the sit message, the RViz panels, and `CONN_TYPE=webrtc` as the compose default.
5. **Gamepad: LB deadman test no longer required.** The team no longer drives with an Xbox gamepad; it uses the Oak ROS2 pad, which sends signals differently. The code is unchanged: a pad on `/joy` still drives through `teleop_twist_joy` on `/cmd_vel_joy` (priority 10) only while LB (button 4) is held, which was verified with simulated input in Docker. The Oak ROS2 pad talks straight to Oak and skips ROS (Max, 2026-10-22).
6. **`webrtc_web.launch.py` skips twist_mux.** That launch file connects the driver straight to `/cmd_vel`, so the keyboard, agent and object-navigation topics do nothing there. Only `robot.launch.py` / `robot_cpp.launch.py` use the priority table.

## Add a decision

When the team decides something that changes how code works, add it under "Decided since" with the date, what was chosen, and why.

<!-- sources: docs/FUSION.md (whole file: What this repo is, Decisions 1-16, twist_mux priorities, Left out, Decided after the fusion, Follow-ups; folded in here and deleted 2026-10-22); docs/plans/pending/2026-10-08-claude-llm-agent.md (deleted 2026-10-22); origin/adib/feat/llm-agent FUSION.md diff (2026-10-12 Ollama decision); 2026-10-22 decisions: Max via coordinator (contract), AGENTS.md Git and Wiki sections, scripts/wiki/README.md -->
