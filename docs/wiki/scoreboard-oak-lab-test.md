---
title: Scoreboard Oak lab test
parent: scoreboard
---

# Scoreboard Oak lab test

This is the supervised lab checklist for the scoreboard's first real-Oak run.
It validates measurement and evidence collection; it is not an algorithm-tuning
session. Use `noah/feat/scoreboard`, not an experimental auto-mapper branch.

Part of: [Scoreboard](scoreboard.md)

## Success criteria

- Oak connects once over WebRTC and `/odom`, `/scan` and `/map` are available.
- The scoreboard reports ready before exploration begins.
- One short, supervised mapping run records distance, known area and structured
  auto-mapper events.
- The run finalizes `run.json`, CSVs, a report, final map and rosbag on the host.
- No collision occurs. Unexpected movement ends the test immediately.

Oak has no ground-truth floor map during this test, so completeness is expected
to be `null`. Known area, distance, goals, events and the saved map are the
valid real-robot measurements.

## Before connecting

1. Confirm no teammate or Unitree phone application is using Oak's single
   WebRTC connection.
2. Use a large clear area. Keep a spotter and the physical controller ready.
3. Keep credentials, keys and lab network details only in the current terminal.
   Never add them to a command log, screenshot or Git file.
4. Open three PowerShell windows at this worktree:

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion\.worktrees\noah-scoreboard"
```

5. Start Docker Desktop, then verify the engine and robot port:

```powershell
docker info
$env:ROBOT_IP = "<robot-ip>"
Test-NetConnection $env:ROBOT_IP -Port 9991
```

Do not continue unless the port check succeeds. Set `ROBOT_AES_KEY` in the
terminal only if this Oak firmware requires it.

## 1. Start the robot stack

In terminal 1:

```powershell
$env:ROBOT_IP = "<robot-ip>"
$env:CONN_TYPE = "webrtc"
# If required: $env:ROBOT_AES_KEY = "<device-key>"

docker compose -p go2scoreboard -f docker/docker-compose.windows.yml down
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml up -d
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml logs -f unitree_ros
```

Wait for `Robot 0 validated and ready`. Ignore the expected RViz display error
on Windows. Do not rebuild the image in the lab.

Open Foxglove at `ws://localhost:8765` and add `/map`, `/scan`, `/odom`,
`/local_costmap/costmap`, `/cmd_vel_out`, `/auto_mapper/events` and
`/scoreboard/status`.

## 2. Read-only preflight

In terminal 2, run each check separately:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "timeout 10 ros2 topic hz /odom"
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "timeout 10 ros2 topic hz /scan"
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 topic info /map"
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 action info /navigate_to_pose"
```

The `timeout` commands may end with a timeout exit code after printing rates;
that is normal. Do not start autonomous motion if `/scan` or `/odom` is absent,
the map is not publishing, or the Foxglove data is stale.

## 3. Arm the scoreboard

In terminal 2:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 run go2_scoreboard recorder --ros-args -p source:=oak -p run_label:=oak_lab_baseline"
```

Leave it running. Wait for `Scoreboard waiting for exploration`.

In terminal 3, confirm the recorder and bag are ready:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 service call /scoreboard/ready std_srvs/srv/Trigger '{}'"
```

Do not start the mapper unless the response reports success.

## 4. Run one short supervised mission

Keep the physical controller with the spotter. In terminal 3:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 run go2_auto_mapper explorer"
```

For the first run, observe only long enough to see a startup scan, at least one
goal event and some map growth. Stop after roughly 30--60 seconds. Do not test
the experimental frontier-heading branch, VLM tuning or recovery maneuvers in
this session. Continue longer only if the short run is controlled and the
spotter agrees.

If a person touches Oak to prevent harm, record it from another terminal:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 service call /scoreboard/mark_human_touch std_srvs/srv/Trigger '{}'"
```

If a collision occurs, stop the mission and mark it:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 service call /scoreboard/mark_collision std_srvs/srv/Trigger '{}'"
```

## Emergency abort sequence

The ROS commands below are not a physical emergency stop. Use the controller
and the lab's physical stop procedure first if Oak is unsafe.

1. Press Ctrl+C in the explorer terminal.
2. Cancel every active Nav2 goal:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'"
```

3. If a ROS source continues commanding motion, hold the highest-priority lane
   at zero in a separate terminal until the explorer and Nav2 goal are stopped:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 topic pub --rate 10 /cmd_vel_joy geometry_msgs/msg/Twist '{}'"
```

Press Ctrl+C on the zero publisher only after autonomous publishers are stopped.

## 5. Finalize and inspect evidence

After the explorer stops, request finalization from terminal 3:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml exec unitree_ros bash -ic "ros2 service call /scoreboard/finish std_srvs/srv/Trigger '{}'"
```

If the service is already gone because the recorder finalized itself, that is
not a failure. Ctrl+C in the recorder terminal is the fallback and preserves a
partial report.

On the host, locate the newest result:

```powershell
$results = "src\go2_scoreboard\results"
$latest = Get-ChildItem $results -Directory |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1
$latest.FullName
Get-ChildItem $latest.FullName
```

Confirm the folder includes `run.json`, `scorecard.csv`, `report.md`,
`timeline.csv`, `resources.csv`, `final_map.yaml`, `final_map.pgm` and `bag/`.
Record the branch and commit, duration, end reason, distance, known area, goal
counts, touches/collisions, warnings and artifact path in the
[Test log](test-log.md). `COMPLETE` means evidence finalization succeeded; it
does not mean the whole floor was mapped.

## 6. Shut down

Stop any remaining foreground commands, then:

```powershell
docker compose -p go2scoreboard -f docker/docker-compose.windows.yml down
docker ps --filter "name=go2scoreboard"
```

The second command should show no running `go2scoreboard` container. Close the
Foxglove connection and release Oak for the next teammate.

## What not to do today

- Do not tune frontier selection, VLM parameters or recovery behavior on Oak.
- Do not merge or test the uncommitted frontier-heading experiment on Oak.
- Do not run another WebRTC client at the same time.
- Do not claim a completeness percentage without a measured answer map.
- Do not spend lab time rebuilding Docker or redesigning the dashboard.
