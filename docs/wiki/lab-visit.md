---
title: Next lab visit
parent: lab-setup
---

# Next lab visit

The run sheet for the next time we are in the lab with Oak: what to bring, what to run, and in what order. The [Lab checklist](lab-checklist.md) gets you connected. This page says what to do once you are. After the visit, write the results in the [Test log](test-log.md) and rewrite this page for the visit after.

Part of: [Lab setup](lab-setup.md)

## Why this visit matters

We now prove fixes in the [Simulator](simulator.md) before Oak (issue #58). The simulator still differs from Oak in ways only Oak can measure (issue #109). This visit collects those measurements (issue #103), and it looks again at the 2026-11-08 failure, where Oak saw "obstacles" just past its nose in an empty hallway and froze.

Goals, most important first:

1. **One bag of `/point_cloud2` and `/odom`**, started before the driver connects. The scoreboard's own bag does not record `/point_cloud2` (`src/go2_scoreboard/go2_scoreboard/bag_recorder.py`), so we record a second bag next to it.
2. **Oak standing still in an empty hallway** for 10 s, inside that bag. This shows whether the cloud has points on Oak's own body or the floor (the close returns from 2026-11-08).
3. **Body measured with a tape**, standing, legs included.
4. **One supervised auto mapper run on current `master`**, with its scorecard.
5. If there is time, the extra measurements at the end.

## Before you leave

- [ ] Pull the latest `master` and build at home: `cd docker && docker compose run --rm unitree_ros true`. Never build in the lab.
- [ ] Ask a teammate for the robot address, the lab network, door codes and, if needed, the AES key. Never write them in this repo.
- [ ] Tell the team you will hold Oak's one connection.
- [ ] Bring a tape measure, a phone for video, and the physical controller. Charge the laptop.
- [ ] Have a spotter. Nobody runs the auto mapper alone.

## 1. Connect

Follow steps 1 and 2 of the [Lab checklist](lab-checklist.md): join the robot's network, `ping <robot address>`, close the Unitree phone app.

## 2. Start the stack and the bag together

The bag must start before the driver connects, so it catches whether the first cloud arrives before the first odometry. Start the container in the background, then the bag right away.

Terminal 1, from the repo's `docker/` folder:

```shell
ROBOT_IP=<robot address> CONN_TYPE=webrtc docker compose up -d
docker compose exec unitree_ros bash -ic "ros2 bag record -o /ros2_ws/src/go2_scoreboard/results/oak_lab_bag_$(date +%Y%m%d_%H%M) /point_cloud2 /odom /scan /tf /tf_static /cmd_vel_out /local_costmap/costmap"
```

With firmware 1.1.15 or newer, put `ROBOT_AES_KEY=<key> ` in front of the first command. On Windows, add `-f docker-compose.windows.yml` to every `docker compose` command.

Leave the bag running for the whole visit. It saves into `src/go2_scoreboard/results/` on your laptop, which git ignores.

Terminal 2, watch the driver:

```shell
docker compose logs -f unitree_ros
```

Wait for `Robot 0 validated and ready`.

## 3. Check the data is flowing

Terminal 3. Each `timeout` ends with an error code after it prints the rates. That is normal.

```shell
docker compose exec unitree_ros bash -ic "timeout 10 ros2 topic hz /odom"
docker compose exec unitree_ros bash -ic "timeout 10 ros2 topic hz /point_cloud2"
docker compose exec unitree_ros bash -ic "timeout 10 ros2 topic hz /scan"
```

Last time: `/odom` about 20 Hz, `/scan` about 3.2 Hz. If `/scan` or `/odom` is missing, stop and fix that first. See "Reading the result" in the [Lab checklist](lab-checklist.md).

## 4. Stand still in an empty hallway

No ROS command needed. The bag is already recording.

- [ ] Put Oak, standing, in the middle of an empty hallway. Nobody within 2 m.
- [ ] Wait 10 s. Write down the clock time.
- [ ] In RViz, look for any dots within 0.5 m of Oak. Take a screenshot.
- [ ] Turn Oak (controller) to face the long end of the hallway. Wait 10 s. Write down the time, and the distance to the far wall if you know it. This shows how far the real lidar sees (the simulator and the stack both assume 20 m; nobody has measured Oak's).

## 5. Measure the body

- [ ] Standing, legs included: length nose to tail, and width.
- [ ] Height of the top of the body from the floor.
- [ ] Take one photo from the side with the tape in view.

The planners assume 0.81 x 0.40 m (`nav2_params.yaml`). The simulator's body is 0.70 x 0.31 m.

## 6. One supervised auto mapper run

Run the scoreboard and explorer exactly as in steps 3 to 5 of the [Scoreboard Oak lab test](scoreboard-oak-lab-test.md), with these changes:

- Run on current `master`, not `noah/feat/scoreboard`.
- On Linux or WSL, drop the PowerShell `-p go2scoreboard -f docker/docker-compose.windows.yml` part. Use `docker compose exec unitree_ros bash -ic "..."` from the `docker/` folder.
- Start the explorer with `ros2 run go2_auto_mapper explorer` in a terminal, not from the RViz panel. If RViz crashes, the run keeps going.
- Use `run_label:=oak_lab_2` for the recorder.

The short version, one command per terminal:

```shell
docker compose exec unitree_ros bash -ic "ros2 run go2_scoreboard recorder --ros-args -p source:=oak -p run_label:=oak_lab_2"
docker compose exec unitree_ros bash -ic "ros2 service call /scoreboard/ready std_srvs/srv/Trigger '{}'"
docker compose exec unitree_ros bash -ic "ros2 run go2_auto_mapper explorer"
```

Stop after 30 to 60 s, or sooner if anything looks wrong. Then finish:

```shell
docker compose exec unitree_ros bash -ic "ros2 service call /scoreboard/finish std_srvs/srv/Trigger '{}'"
```

**If Oak is unsafe, use the controller first.** The ROS abort steps are in "Emergency abort sequence" on the [Scoreboard Oak lab test](scoreboard-oak-lab-test.md) page.

## 7. Extra measurements, only if there is time

- [ ] **Odometry error.** Tape 2 m on the floor. Drive Oak straight along it with the keyboard. Compare with how far `/odom` says it went (`ros2 topic echo /odom --once` before and after).
- [ ] **Command delay and stop time.** Film Oak from the side with the laptop screen in view. Start and stop a straight drive. The video shows the time from key press to first step, and from key release to stop.
- [ ] **Body tilt.** Drive back and forth for 10 s while the bag records. Nothing to do by hand; we read the tilt from `/tf` later.

## 8. Wrap up

- [ ] Ctrl+C the bag in terminal 1. Check it saved: `docker compose exec unitree_ros bash -ic "ros2 bag info /ros2_ws/src/go2_scoreboard/results/<bag folder>"`.
- [ ] `docker compose down`, then release Oak for the next teammate.
- [ ] Add a dated section to the [Test log](test-log.md): code commit, each step above with pass or fail, the clock times from step 4, the body sizes, and the scorecard numbers.
- [ ] Put the bag where the team can reach it, and link it on issue #103.

## What not to do

- Do not tune the auto mapper, VLM or recovery moves on Oak this visit.
- Do not run a second WebRTC client (phone app or another laptop) at the same time.
- Do not rebuild the Docker image in the lab.
- Do not commit the robot address, network details or AES key.

## Connects to

- [Lab checklist](lab-checklist.md): connecting to Oak.
- [Scoreboard Oak lab test](scoreboard-oak-lab-test.md): the full run and abort steps.
- [Simulator](simulator.md): what these measurements will be compared with.
- [Test log](test-log.md): where the results go.
