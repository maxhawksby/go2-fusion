---
title: LLM agent
parent: brains
code: [src/go2_llm_agent/**]
verified: 96e1ea24f469
---

# LLM agent

The LLM agent lets you type a plain-English command like "walk forward for three seconds then turn left" and have Oak do it, so people can talk to Oak instead of steering it.

Part of: [Brains](brains.md)

## How it works

An *LLM* (large language model) is the kind of AI behind chatbots. Familiar, complete commands use a small local parser immediately; other wording goes to Qwen for interpretation. Both paths produce the same action plan, such as "walk forward 3 s" and "turn left 90 degrees". The agent runs each step by publishing speeds on its own drive topic. The local parser skips the model call; it does not skip the agent or its movement controller.

**Two versions exist right now.**

- **On upstream `master` before this branch merges (Riley's original):** built by Riley Francis on OpenAI's paid GPT-4o-mini, using the LangChain and LangGraph libraries. We brought it in almost as-is. Its Python libraries are deliberately not installed in our Docker image, so it builds but `ros2 run` fails on missing imports.
- **On branch `adib/feat/llm-agent` (the team's direction):** Adib replaced the paid model with **Ollama**, a free program that runs AI models on your own computer, running a small **Qwen** model (`qwen3:4b-instruct-2507-q4_K_M`). No API key and no extra Python libraries are needed.

In Adib's version, a local checker validates every plan before anything moves. Walking uses 0.5 m/s and turning uses 1.5 rad/s. Positive durations and angles have no fixed upper cap, and there is no fixed action-count or total-plan duration cap. Input and model-output sizes remain bounded; incomplete plans do not execute. Typing `stop` cancels locally without waiting for Qwen. Angle turns ("turn left 90 degrees") watch Oak's measured heading on `/odom`. The controller slows near the target, corrects overshoot and waits for fresh readings to show a stable final heading. Missing or stale feedback refuses the turn. A completion deadline stops stalled turns; it never declares an angle reached. Physical turning accuracy needs lab measurement.

Example, in simulation (the default, which moves nothing):

```text
go2> walk backward for one second
[SIMULATION] velocity: forward=-0.5 m/s, left-turn=0 rad/s
Finished publishing this plan; sent stop. Robot movement is not acknowledged.
```

**The Claude plan was dropped.** On 2026-10-08 the team planned to rebuild the agent on Anthropic's Claude. That plan is superseded by Adib's Ollama agent, because Ollama runs free and local with no paid API key, and its plan file was deleted (git history keeps it). See [Decisions](decisions.md).

### Riley's version (on master) in detail

| Module | What it does |
| --- | --- |
| `agent.py` | Plans and runs several motion commands from one plain-English request |
| `ros2_motion.py` | Live backend: publishes `/cmd_vel_agent`, `/tts` and `/webrtc_req` |
| `ros2_mock_motion.py` | Mock backend that prints commands instead of moving the robot |
| `speech_bridge.py` | FastAPI server (`POST /command`) that feeds transcripts to the agent |

The planner returns a `Plan`: a list of `Step`s, each an action plus an optional `duration_s` and `label`. The executor then calls exactly one tool per step. The tools are `walk` (forward for a time at a speed), `turn_in_place`, `sit` and `stretch` (each followed by a wait and a recovery stand), `stand_up`, `stand_down`, `recovery_stand`, and `look_for_object` (mock mode only).

In live mode it publishes `/cmd_vel_agent` (walk and turn), `/webrtc_req` (Sport API: stand up 1004, stand down 1005, recovery stand 1006, sit 1009, stretch 1017) and `/tts` (text for `speech_processor`'s `tts_node` to speak). It subscribes to nothing. In mock mode it publishes nothing at all.

Setup: set `OPENAI_API_KEY` and `ROBOT_ENV` in the environment, or in a gitignored `.env` file in the folder you run from; never commit the key. `ROBOT_ENV` defaults to `testing`, which uses the mock backend and never moves the robot; `ROBOT_ENV=live` drives the real robot. In live mode, start with tiny moves ("walk forward for half a second") in an open area, with the keyboard or gamepad ready; both override the agent. For speech out, start `ros2 run speech_processor tts_node` with an ElevenLabs key.

Adding an action to Riley's version: add a method to both backends (`MockMotionController` in `ros2_mock_motion.py` and `RosMotionController` in `ros2_motion.py`), add an `@tool` wrapper in `agent.py` and put it in the `tools` list, add its name to `Step.action`'s `Literal` and to the planner's system prompt, then test with `ROBOT_ENV=testing` first and live at low speed.

## Connects to

- [twist_mux](twist-mux.md): the agent drives on `/cmd_vel_agent`, priority 4. The gamepad (10), keyboard (9) and Nav2 (5) all override it.
- [Posture](posture.md): Riley's version also sends sit, stand and stretch on `/webrtc_req`. Adib's version leaves posture out for now.
- [Voice](voice.md): Riley's version had a small web server (`speech_bridge`, `POST /command` on port 8000) that the microphone tool sends text to. Adib's branch disables it; typed commands come first.
- [Object detection](object-detection.md): Riley's `look_for_object` was meant to ask the detector what it sees, but it only works in mock mode.
- [Roadmap](roadmap.md) and [Decisions](decisions.md): why the agent changed direction.

## How it breaks

From Riley's version, as recorded in the package README and the dropped Claude plan (bugs found by reading the code, not by running it):

- It crashes without `OPENAI_API_KEY`, even in mock mode, because the AI client is created on import.
- `look_for_object` always errors in live mode.
- `look_for_object` calls `find_object()`, which only the mock backend has; it is not wired to `object_detection`. A fix would subscribe to `/detected_objects_dict` and look the label up there.
- Importing `agent.py` (which `speech_bridge` does) starts the motion backend as a side effect.
- The plan is fixed before anything runs and never sees tool results, turn direction can get lost ("turn right" turns left), it loops forever when no tool is called, there is no stop and no speed cap, and the web handler blocks while the robot moves.
- `ros2_motion.py` has an unused `dance()` stub.

From Adib's branch README:

- **Cannot reach Ollama:** keep the Ollama app running and check from inside Docker with `ros2 run go2_llm_agent agent --check`.
- **Turns report missing heading:** `/odom` is not arriving. Fix the driver side; the agent will not fake a turn by time.
- On an 8 GB Mac, heading checks sometimes timed out while Qwen was loaded, and passed once it was unloaded. Use `--timeout 60 --keep-alive 0` there.
- `stop` only stops this agent. It is not an emergency stop for all of Oak.

## Who's on it and what's next

**Adib** built the Ollama version and tested it in mock mode; the team update says it was also tried on Oak. Next, Adib moves to odometry reliability (why position data sometimes stops) and turn accuracy (requested vs measured rotation). TODO(Adib): when the branch merges into `master`. The branch has been rebased onto the current workspace layout: its package and tests now live under `src/go2_llm_agent/`.

## How angle turns are measured

Go2 has built-in position and orientation feedback. In this workspace, the driver
converts the WebRTC `rt/utlidar/robot_pose` stream into `/odom`; the agent reads its
orientation quaternion (four numbers describing rotation), converts it to heading,
and tracks actual changes. It counts a full 360-degree spin across the +/-180-degree
wrap instead of treating a full circle as zero. This was already measurement-based;
the previous controller's weakness was staying at full speed until the first target
crossing and never checking the final stopped angle.

The new controller compares the requested angle with the measured angle on every
update. It uses up to 1.5 rad/s far from the target, slows as the error shrinks, and
can make a slow reverse correction after an overshoot. It sends zero while checking
that the measured error stays within 1 degree (or half the requested angle for very
small turns), with no more than 0.5 degree of movement across at least three fresh
readings spanning 0.35 seconds. The terminal reports measured rotation and remaining
error. Repeated old readings cannot count as settling; lost feedback, changed frames,
a large pose jump or an unfinished turn aborts the remaining actions.

The odometry subscriber keeps only the latest sample. Its quaternion uses the ROS
x/y/z/w order; Unitree's separate IMU message uses a different array representation,
so the agent does not interchange those messages or silently switch sources. Timed
turns explicitly requested in seconds remain timed; angle goals always need heading.

**One degree is a tolerance on reported odometry, not a promise of physical accuracy.**
Sensor drift, delayed messages, foot slip and the real robot's response can make the
physical angle differ. First compare `/odom` with Gazebo's independent true pose,
then measure Oak against a physical reference in the lab. The simulator's error and
latency settings are test assumptions, not measurements of Oak. It slides a body;
it does not reproduce the real robot's leg dynamics. Do not call the result 100% exact.

Unitree's official [sport state](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg)
and [IMU state](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/IMUState.msg)
definitions document its position, orientation and yaw-rate feedback.

## Requests and interpretation

| Request | Result |
| --- | --- |
| `walk forward for 20 seconds then turn left` | Walk 20 s, stop, turn left 90 degrees, stop |
| `walk backward` | Default 1 s backward |
| `spin around` | Full 360-degree left turn |
| `spin clockwise 720 degrees` | Two full right turns |
| `move forward for two minutes` | Convert minutes to 120 seconds |
| `walk for one second` | Ask for a direction |
| `walk forward for three steps` | Unsupported: footsteps are not counted |

Qwen can interpret wording beyond the old small keyword list. Use `--plan-only`
to inspect unfamiliar wording: model interpretations are not independently verified
word by word by default. `--strict-language` restores the conservative source-word
and quantity checks and sends every request to Qwen. `--llm-only` always asks Qwen,
useful for measuring the model separately from the fast parser. Negated or conditional
movements, distances, requested speeds, postures and continuous movement remain outside
this typed milestone. No duration means the documented default, not indefinite walking.

## Set up the Ollama version

Run Ollama on the host or a shared model server. On a Mac, running it natively
lets it use the Apple GPU; the ROS agent remains in Docker. Install the model once:

```shell
ollama pull qwen3:4b-instruct-2507-q4_K_M
```

Follow [Mac setup](mac-setup.md) or [WSL and Windows](wsl.md) to start the workspace.
From the repository root on a Mac, rebuild the agent once and enter Docker:

```shell
bash docker/mac/go2.sh rebuild go2_llm_agent
bash docker/mac/go2.sh shell
```

The default Ollama server is `http://host.docker.internal:11434`. Change it with
`--ollama-url` (or `OLLAMA_BASE_URL`) for a shared server; change the model with
`--model` (or `OLLAMA_MODEL`). `--check` checks reachability and the installed model.
No extra pip installs are needed. Speech recognition, speech output, posture and
object navigation are outside this typed-command feature; the old speech bridge
entry point exits with an explanation instead of starting an HTTP server.

The default is mock mode, which prints simulated commands and sends no ROS messages.
`--plan-only` also never publishes, even if `--mode ros` is supplied. For live use,
start the driver and twist_mux separately before the agent:

```shell
ros2 run twist_mux twist_mux --ros-args --params-file /ros2_ws/src/go2_robot_sdk/config/twist_mux.yaml
ros2 run go2_llm_agent agent --mode ros --timeout 0 --keep-alive 0
```

This does not start a connection to Oak. Keep only one direct driver connection;
see [Robot link](robot-link.md). `stop`, `quit`, Ctrl+C and terminal EOF cancel this
agent's pending actions. Cancellation during interpretation discards the eventual
model reply; another request waits for it to finish. There are no automatic retries
that could replay a movement.

Model options:

| Option | Meaning |
| --- | --- |
| `--timeout 0` | Wait without a model socket timeout; this is separate from movement duration |
| `--timeout 60` | Allow 60 seconds per model socket operation; default is 30 |
| `--keep-alive 0` | Release Qwen memory after each reply; non-parser requests reload it |
| `--keep-alive 5m` | Keep Qwen loaded for five minutes, reducing reload delay but using more RAM |

On the tested 8 GB Mac, heading tests sometimes lost fresh feedback with Qwen
resident and passed after it was unloaded. This suggests resource contention;
it does not establish a single cause. The local parser does not load Qwen at all.
With `--timeout 0`, a stuck model request can wait indefinitely. `stop` cancels its
pending actions but another request waits for the HTTP call to finish; `quit` or
Ctrl+C closes the agent. A finished plan means publishing finished, not that the
robot acknowledged physical travel. If walking never starts, check that twist_mux
subscribes to `/cmd_vel_agent` and publishes `/cmd_vel_out` for the driver.

Inside Docker, run the isolated DDS test with simulated heading (domain 87 is
separate from the normal robot session):

```shell
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87 python3 /ros2_ws/src/go2_llm_agent/test/ros_motion_smoke.py
```

## The code

Folder: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_llm_agent

Adib's version (from his branch README), inside the Docker container:

```shell
ros2 run go2_llm_agent agent --check                 # is Ollama reachable, is the model installed?
ros2 run go2_llm_agent agent                         # type at the go2> prompt; simulation only
ros2 run go2_llm_agent agent --plan-only --command "walk forward for 3 seconds then turn left"
python3 -m unittest discover -s /ros2_ws/src/go2_llm_agent/test -p test_agent.py -v
```

Riley's version (on master; its pip deps are not in the image, so these fail on missing imports there):

```shell
ros2 run go2_llm_agent agent            # interactive text prompt
ros2 run go2_llm_agent speech_bridge    # HTTP bridge on port 8000
curl -X POST http://localhost:8000/command -H "Content-Type: application/json" \
  -d '{"text":"walk forward for one second"}'    # test the bridge without a microphone
```

Adib's version only publishes to ROS with `--mode ros`. Do that only in the lab, next to Oak, with the keyboard ready.

<!-- sources: src/go2_llm_agent/README.md (folded in 2026-10-22); docs/FUSION.md rows 2, 3, 11 and "Decided after the fusion"; docs/plans/pending/2026-10-08-claude-llm-agent.md; origin/adib/feat/llm-agent (38a6980: go2_llm_agent/README.md, commands.py, agent.py, FUSION.md and plan-file diffs); AGENTS.md; team update (Adib) week of 2026-10-18 -->

## Optional Gazebo turn check

This test uses the existing `go2_sim` body model and simulated driver on an empty
flat floor. It starts only headless Gazebo, the ROS bridge, sim_driver and twist_mux;
there is no WebRTC driver, robot connection, mapping, speech or model inference.
The test observer compares the agent's reported odometry with Gazebo's independent
true pose for 30-degree left, 90-degree right, 180-degree left and full right turns.
Only the temporary test world loses its unused lidar sensors; teammate files stay
unchanged. Logs and JSON results go to a printed `/tmp/go2-gazebo-turn-check-*` path.

Review the scope before starting it. Inside Docker:

```shell
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 timeout --signal=INT --kill-after=8s 240s python3 /ros2_ws/src/go2_llm_agent/test/gazebo_turn_check.py
```

Use `--noise-yaw 0.01 --seed 17` for a second run with the simulated driver's normal
heading-noise setting. With no added noise, the test checks a 1.5-degree ground-truth
tolerance as well as the reported-heading tolerance. With noise it reports the true
error without claiming that accurate sensor tracking guarantees accurate physical
rotation. The driver also models 0.1-second command delay and acceleration limits.
These are simulator assumptions until they are measured on Oak. The test refuses
to run outside local-only ROS domain 88 and stops its own subprocesses on exit.

### Checked in Gazebo (2026-11-08)

One clean run and one noisy run, four turns each, local-only domain 88. Both used
0.1-second command delay and the existing simulated driver's acceleration limits.
These are simulated true angles after settling, not measurements of physical Oak:

| Requested | True angle, clean odometry | True angle, heading noise 0.01 |
| --- | --- | --- |
| 30 degrees left | 29.90 degrees | 30.73 degrees |
| 90 degrees right | 90.02 degrees | 90.90 degrees |
| 180 degrees left | 180.14 degrees | 179.48 degrees |
| 360 degrees right | 360.15 degrees | 360.68 degrees |

Largest true-angle error: 0.15 degree clean, 0.90 degree noisy. All eight requests
settled within the reported-heading tolerance. This small test batch is regression
evidence, not a guarantee of those errors on other runs or on Oak. The 36 offline
tests, isolated DDS test with inertia and different speed response, and full
15-package source build also passed. No live robot commands were sent.
