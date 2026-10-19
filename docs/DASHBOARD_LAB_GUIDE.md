# Oak Control lab quick start

Use this guide when the Windows Docker image has already been built. These
steps start the current source code through the fast, mounted workspace. Do
not run `docker compose build` unless `requirements.txt` or the Dockerfile
changed.

Run every PowerShell block from the repository root. While the dashboard is
still under review, that may be the separate `go2-fusion-dashboard` worktree:

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion-dashboard"
```

## Before starting

1. Power on Oak.
2. Connect Windows to the robot's lab Wi-Fi.
3. Close the Unitree mobile app because Oak accepts only one WebRTC client.
4. Start Docker Desktop and wait until it reports that the engine is running.

Check Docker and initialize the pinned submodule:

```powershell
docker info
git submodule update --init --recursive
```

## Terminal 1: start ROS and connect to Oak

```powershell
$env:ROBOT_IP="192.168.50.150"
$env:CONN_TYPE="webrtc"

docker compose -p go2fusion -f docker/docker-compose.windows.yml up -d
docker compose -p go2fusion -f docker/docker-compose.windows.yml logs -f unitree_ros
```

On the first start after source or branch changes, the incremental ROS build
may take one or two minutes. Wait for:

```text
Summary: ... packages finished
Robot 0 validated and ready
```

Press `Ctrl+C` after those messages. This stops only the log display; the
container continues running. The RViz `could not connect to display` warning
is expected in Windows Docker.

## Terminal 2: start object detection

The raw camera does not need this process. The annotated camera and detected
object list do.

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion-dashboard"

docker compose -p go2fusion -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 run object_detection detector_node"
```

Wait for:

```text
RGB object detector with memory ready
```

YOLO uses noticeable CPU. Leave this terminal open only while testing object
detection. Press `Ctrl+C` here when annotated video is no longer needed.

## Terminal 3: start the dashboard ROS bridge

This command enables posture, gesture, StopMove, TTS, and deterministic voice
requests. Website WASD remains disabled.

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion-dashboard"

docker compose -p go2fusion -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 run go2_dashboard_bridge dashboard_bridge --ros-args -p host:=0.0.0.0 -p allow_commands:=true"
```

Wait for:

```text
Dashboard API listening on 0.0.0.0:8080; commands ENABLED
```

Keep this terminal open. Press `Ctrl+C` to disable the dashboard bridge.

## Terminal 4: start the website

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion-dashboard"

node host_tools/control_dashboard/server.mjs
```

Open <http://localhost:3000> and perform a hard refresh with `Ctrl+Shift+R`.
Then:

1. Switch to **Robot Mode**.
2. Confirm the robot heartbeat is active.
3. Select **Raw camera**.
4. Select **Object detections** after the detector is ready.
5. Test one posture or gesture at a time while watching Oak.

The **Stop Request** button sends Unitree StopMove once. It is not an
emergency stop and does not lock Nav2 or another ROS motion publisher.

## Quick status checks

From another PowerShell window:

```powershell
Invoke-RestMethod http://127.0.0.1:8080/health
Invoke-RestMethod http://127.0.0.1:8080/api/status | ConvertTo-Json -Depth 6
```

Foxglove remains available at `ws://localhost:8765`.

## Stop everything

Press `Ctrl+C` in the detector, bridge, and website terminals. Then run:

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion-dashboard"
docker compose -p go2fusion -f docker/docker-compose.windows.yml down
```

Closing the browser does not stop the website, bridge, detector, or robot
connection.

## Common problems

### Port 3000 is already in use

```powershell
$listener = Get-NetTCPConnection -LocalPort 3000 -State Listen
Stop-Process -Id $listener.OwningProcess
node host_tools/control_dashboard/server.mjs
```

### The bridge says port 8080 is already in use

Another bridge is already running inside the container. Stop its terminal. If
that terminal is no longer available, restart the container and repeat the
ordered steps:

```powershell
docker compose -p go2fusion -f docker/docker-compose.windows.yml restart unitree_ros
docker compose -p go2fusion -f docker/docker-compose.windows.yml logs -f unitree_ros
```

### The annotated feed is offline

Keep Terminal 2 open and check:

```powershell
docker compose -p go2fusion -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 topic info /detections/image_annotated"
```

The topic should have one publisher from `object_detector` and one subscriber
from `go2_dashboard_bridge`.

### Commands are disabled

Restart Terminal 3 with `-p allow_commands:=true`. Only enable commands while
physically present with Oak and watching it.

### TTS is unavailable

The current dashboard branch still launches the ElevenLabs configuration.
Local Piper speech will become available after the tested Piper feature is
integrated.
