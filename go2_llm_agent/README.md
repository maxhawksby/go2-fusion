# Typed commands with local Ollama

Type a request into this agent's terminal. It asks your local Qwen model to interpret
the request, checks the proposed actions, and then simulates them. Walking can use
seconds; turning can use an angle or a named full spin. Simulation is the default:
it imports no ROS backend and publishes no messages, even when Oak's driver
is running in another terminal.

```text
Your text -> Ollama/Qwen -> validated action plan -> simulation or ROS controller
```

Ollama is the program running the model. Qwen is the downloaded model. This ROS
package is the application connecting the model's interpretation to the existing
robot control route. The model does not generate executable Python or shell code.

This replaces Riley Francis's original GPT/LangChain/LangGraph agent for the first
typed-command milestone. No OpenAI, Anthropic or ElevenLabs API key is required.
Voice input, speech output, posture commands and object navigation are not included.

**Development scope:** this branch develops typed commands only. No speech-to-text
or text-to-speech functionality is being developed. The existing host microphone
tool and `speech_processor` package are outside this feature.
The old `speech_bridge` entry point explains that it is disabled rather than starting
an HTTP server with the previous agent's import side effects.

## Before running

1. Run Ollama on your Mac (outside Docker so it can use the Apple GPU). The ROS
   agent still runs inside Docker. If the model is already installed, skip the pull:

   ```bash
   ollama pull qwen3:4b-instruct-2507-q4_K_M
   ```

2. Start the Mac Docker workspace using [macos_setup.md](../macos_setup.md). From the
   **go2-fusion repository** on your Mac, rebuild this package once:

   ```bash
   bash docker/mac/go2.sh rebuild go2_llm_agent
   bash docker/mac/go2.sh shell
   ```

3. Inside Docker, check that the server is reachable and the model is installed:

   ```bash
   ros2 run go2_llm_agent agent --check
   ```

The check prints the server, model name and model digest (the downloaded version's
identifier). The default server is `http://host.docker.internal:11434`, which addresses
the host from Docker Desktop. No additional pip installs or Docker image rebuild
are needed: this agent uses Python's standard library and existing ROS dependencies.

## First use: simulation

In the Docker terminal:

```bash
ros2 run go2_llm_agent agent
```

Type at the **`go2>` prompt**, not in Ollama's separate `>>>` chat:

```text
go2> walk backward for one second
Plan: ... walk, backward, duration_s=1 ...
[SIMULATION] velocity: forward=-0.2 m/s, left-turn=0 rad/s
[SIMULATION] velocity: forward=0 m/s, left-turn=0 rad/s
Completed; stopped.
go2> quit
```

Negative forward velocity means backward; a negative left-turn velocity means a
right turn. Motion duration begins after model interpretation finishes. The first
request can be slower because the model needs to load. `stop` cancels locally,
without waiting for another model response. Ctrl+C, EOF and `quit` also cancel.

One request runs at a time. While it is running, another movement request is
rejected rather than queued. If you stop during model interpretation, the eventual
reply is discarded; wait for that request to finish before submitting another.

For a single request or interpretation without simulated execution:

```bash
ros2 run go2_llm_agent agent --command "turn right for half a second"
ros2 run go2_llm_agent agent --plan-only --command "walk forward for 3 seconds then turn left"
```

`--plan-only` never starts ROS, including when combined with `--mode ros`.

## Supported requests and defaults

You do not need to supply seconds for every action. The application applies these
fixed, documented defaults; the model cannot choose arbitrary defaults.

| Request | Result |
| --- | --- |
| `walk forward for three seconds then turn left` | Walk forward for 3 s, stop, then turn left 90°, stop |
| `turn left` / `turn right` | Turn 90° in the requested direction |
| `spin around` | One full 360° turn to the left |
| `spin clockwise` | One full 360° turn to the right |
| `turn left 180 degrees` | Turn left through 180° |
| `walk backward` | Walk backward for the default 1 s |
| `walk for one second` | Ask for the missing direction |
| `turn right for two seconds` | A timed right turn for 2 s, without an angle goal |
| `walk forward for three steps` | Unsupported: footsteps are not counted or converted to seconds |
| `stop` / `please stop` / `cancel` | Cancel this agent's movement and pending actions locally |

Walking speed is 0.2 m/s; turning speed is 0.5 rad/s. Each timed movement is limited
to 5 seconds and each angle turn to 360°. Plans contain at most 4 actions and have a
45-second execution budget, including bounded turn timeouts. Specified quantities
are never shortened or substituted. Distances, requested speeds, continuous motion,
postures and object navigation remain outside this milestone. The entire plan is
checked before the first action; partially unsupported requests do not execute.

**Angle turns use feedback.** The existing agent node subscribes to `/odom` and
tracks changes in reported heading, including wraparound at ±180°. A full spin
therefore counts a full revolution rather than stopping immediately when its final
orientation matches its starting orientation. An angle goal completes within a 2°
reported-heading tolerance. Physical accuracy depends on the robot's odometry and
stopping response and still needs lab validation; this is not precision positioning.
Simulation integrates the commanded angular velocity to provide simulated heading.

Fresh heading is checked before any mixed plan that needs an angle turn begins.
Missing/stale heading, a changed odometry frame, motion in the opposite direction,
or a turn timeout stops execution and skips the remaining actions. There is no
fallback that guesses turn completion from elapsed seconds. `stop` also interrupts
angle turns immediately. Timed walks and explicitly timed turns do not need heading.

The local validator checks structure, limits, action order and each action's stated
quantity or documented default. Negated and conditional movements are refused.
These checks recognize a conservative vocabulary and cannot prove that a model
understood arbitrary English. The regression suite is evidence for its test cases,
not a guarantee for every phrasing. No speech-to-text or text-to-speech development
is included in this branch.

## ROS output and live use

ROS publishing requires the explicit option `--mode ros`. It publishes
`geometry_msgs/Twist` on the existing `/cmd_vel_agent` input, with priority 4 in
twist_mux. It does not start a driver, establish a robot connection, use `/tts`, or
send posture API requests. When a driver is running in the same ROS domain, these
messages can move Oak. `stop` cancels **this agent**; it is not a global emergency
stop and does not override higher-priority keyboard/joystick/Nav2 commands.

First verify publishing in an isolated domain with the smoke test below. Live
testing is a separate lab step, after reviewing the interpreted requests. Use the
team's `robot.launch.py` / `robot_cpp.launch.py` route with twist_mux;
`webrtc_web.launch.py` does not use this agent's input (see [FUSION.md](../FUSION.md)).

## Model settings and future lab computer

CLI options override environment defaults:

| Setting | Default |
| --- | --- |
| `--ollama-url` / `OLLAMA_BASE_URL` | `http://host.docker.internal:11434` |
| `--model` / `OLLAMA_MODEL` | `qwen3:4b-instruct-2507-q4_K_M` |
| `--timeout` | 30 seconds per HTTP socket operation; maximum configurable value 120 s |
| `--keep-alive` / `OLLAMA_KEEP_ALIVE` | `5m` keeps the model loaded; `0` releases model memory after each reply |

On the tested 8 GB Mac, the simulated ROS heading test sometimes timed out or lost
fresh heading with Qwen resident, and passed after Qwen was unloaded. This suggests
resource contention; it does not establish a single cause or replace live testing.
For initial lab testing on this machine, use:

```bash
ros2 run go2_llm_agent agent --timeout 60 --keep-alive 0
```

This still starts in simulation. Each request reloads Qwen, so interpretation takes
longer, but model memory is released before motion. A lab model server or a machine
with more memory can use the default `5m`. Ollama documents this behavior in its
[chat API](https://docs.ollama.com/api/chat). The heading freshness checks stay
enabled in either mode; lost telemetry stops the plan.

The client requests a JSON schema, temperature 0, a short 2048-token context and
at most 512 output tokens. Truncated, malformed, unexpected or out-of-range plans
do not execute. HTTP failures are reported; there are no automatic retries that
could replay a movement.

Windows Docker Desktop uses the same agent commands after the team's Windows
setup. On Linux or a shared server, supply a server address reachable from the ROS
container. For example (replace the address with the actual lab server):

```bash
ros2 run go2_llm_agent agent --ollama-url http://192.168.50.200:11434 --check
```

For the future lab computer, download the model there once and configure its
network access. This agent can use that server by changing the URL. A shared
laptop-facing command service and ownership/queuing for Oak are future work; this
milestone has a local terminal interface, not a multi-user HTTP command service.

## Verification (inside Docker)

Run the offline regressions; these do not require Ollama or publish ROS messages:

```bash
python3 -m unittest discover -s /ros2_ws/src/go2_llm_agent/test -p test_agent.py -v
```

Run the opt-in real-model regression cases. This only interprets requests:

```bash
python3 /ros2_ws/src/go2_llm_agent/test/benchmark_ollama.py
```

Results include the model digest, exact interpretations and response times in
`/tmp/go2-ollama-benchmark.json`. Interpretations are checked against expected
actions, directions, durations and angles. This uses your local model without API charges.

Test actual DDS messages in domain 87, local-only, isolated from the normal robot
session. The test refuses to run without both environment settings:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87 python3 /ros2_ws/src/go2_llm_agent/test/ros_motion_smoke.py
```

## Troubleshooting

- **Cannot reach Ollama:** keep the Ollama application running. Use `--check` from
  Docker, not just `ollama list` on the host. Verify the server URL before changing
  networking settings; Docker can already reach the native server on the tested Mac.
- **Model missing:** pull the exact model tag on the computer hosting Ollama. A
  chat request does not automatically download an absent model.
- **Busy after stop:** model inference is still finishing. Its result is discarded;
  the current HTTP request is bounded by the socket timeout.
- **Slow replies:** use `ollama ps` on the host to check model placement. Keep one
  model loaded and measure memory use with RViz/Docker running. Increase timeout
  only after confirming the server is healthy.
- **Angle turns report missing heading:** confirm fresh `/odom` messages in the
  same ROS domain. Fix the driver/odometry source; do not substitute a timed turn
  when the request asks for an angle.
- **Old OpenAI imports:** ensure this branch is mounted into Docker and rebuild
  `go2_llm_agent` using the command above. `ROBOT_ENV=live` no longer selects live
  mode; simulation stays default unless `--mode ros` is explicitly supplied.
