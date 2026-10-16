# go2_dashboard_bridge

This package keeps the Oak Control browser UI independent of the team's changing ROS nodes. The browser calls a
small HTTP API; this bridge is the only layer that knows the current ROS topic names and detector payload format.

## Safety defaults

The bridge starts read-only. `GET /api/status` and `GET /api/objects` work, but every command endpoint returns HTTP
403 until `allow_commands` is explicitly enabled. Dashboard velocity control and autonomous navigation are not
implemented in this milestone.

## Run

The package runs inside the project's ROS Docker container. On Windows, port 8080 is published by
`docker-compose.windows.yml`.

```shell
ros2 run go2_dashboard_bridge dashboard_bridge --ros-args -p host:=0.0.0.0
```

To enable the curated Sport actions and `/tts` publishing while physically present with the robot:

```shell
ros2 run go2_dashboard_bridge dashboard_bridge --ros-args -p host:=0.0.0.0 -p allow_commands:=true
```

The browser UI remains a separate host process at <http://localhost:3000>. Its API base is
`http://localhost:8080`.

The bridge defaults to `127.0.0.1`. The explicit `host:=0.0.0.0` override lets Docker publish it, while the Windows
Compose file binds the host port to `127.0.0.1` so other devices on the lab network cannot reach it.

## Stable API

| Endpoint | Purpose |
| --- | --- |
| `GET /health` | Bridge process health |
| `GET /api/status` | Robot heartbeat, command gate, and ROS topic availability |
| `GET /api/objects` | Normalized object list from the configured detector topic |
| `GET /api/camera.jpg?source=raw\|annotated` | Latest JPEG from the selected camera topic |
| `POST /api/action` | Curated Unitree Sport action |
| `POST /api/tts` | Publish text on `/tts` |
| `POST /api/command` | Deterministic voice phrase mapping for actions and speech |

ROS topic parameters are `state_topic`, `objects_topic`, `sport_topic`, and `tts_topic`. A future detector can keep
publishing the existing JSON contract, remap `objects_topic`, or add a new normalization function here. The browser
does not need to change.
