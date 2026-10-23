---
title: Posture
parent: brains
code: [src/front_leg_controller/**]
---

# Posture

Posture covers Oak's built-in moves, like sit, stand up, stretch and wave hello, which Oak performs by itself when you send it one short command.

Part of: [Brains](brains.md)

## How it works

The Go2 has a library of ready-made moves, the same ones Unitree's phone app shows as quick-action buttons. Each move has a number, called an *API ID*. For example, Stand Up is 1004, Sit is 1009 and Stretch is 1017.

To trigger a move, a program publishes one message on the `/webrtc_req` topic, with the move's API ID and the robot's sport command channel (`rt/api/sport/request`). The [Go2 driver](go2-driver.md) carries it to Oak, and Oak does the rest.

The `front_leg_controller` package holds two programs:

- **`sit_controller`** works for real. It waits until the driver is listening on `/webrtc_req` (so the command is not lost), sends Sit once, and exits. It is the best example of how to send any move.
- **`front_leg_controller`** is a placeholder. It reads `/joint_states` and logs pretend leg motion, but never sends a real command.

### All moves and their API IDs

The source of truth is `ROBOT_CMD` in the driver's `go2_robot_sdk/domain/constants/robot_commands.py`.

| Movement and posture | API ID |
| --- | --- |
| Damp | 1001 |
| BalanceStand | 1002 |
| StopMove | 1003 |
| StandUp | 1004 |
| StandDown | 1005 |
| RecoveryStand | 1006 |
| Euler | 1007 |
| Move | 1008 |
| Sit | 1009 |
| RiseSit | 1010 |
| SwitchGait | 1011 |
| Trigger | 1012 |
| BodyHeight | 1013 |
| FootRaiseHeight | 1014 |
| SpeedLevel | 1015 |
| TrajectoryFollow | 1018 |
| ContinuousGait | 1019 |
| GetBodyHeight | 1024 |
| GetFootRaiseHeight | 1025 |
| GetSpeedLevel | 1026 |
| SwitchJoystick | 1027 |
| Pose | 1028 |
| GetState | 1034 |
| EconomicGait | 1035 |
| StandOut | 1039 |
| FreeWalk | 1045 |
| Standup | 1050 |
| CrossWalk | 1051 |

| Fun and show moves | API ID |
| --- | --- |
| Hello | 1016 |
| Stretch | 1017 |
| Content | 1020 |
| Wallow | 1021 |
| Dance1 | 1022 |
| Dance2 | 1023 |
| Scrape | 1029 |
| FrontFlip | 1030 |
| FrontJump | 1031 |
| FrontPounce | 1032 |
| WiggleHips | 1033 |
| FingerHeart ("show love") | 1036 |
| Handstand | 1301 |
| CrossStep | 1302 |
| OnesidedStep | 1303 |
| Bound | 1304 |
| MoonWalk | 1305 |

## Connects to

- [Go2 driver](go2-driver.md): delivers `/webrtc_req` to the robot.
- [LLM agent](llm-agent.md): Riley's version sends sit, stand up, stand down, recovery stand and stretch the same way.
- [Voice](voice.md): speech out also travels on `/webrtc_req`, on the audio channel instead.
- The Oak Control dashboard (Noah, pull request #14) has quick-action buttons for posture and gestures. See [People](people.md).

## How it breaks

- **These commands move the real robot.** Only send them when you are in the lab, watching Oak, in a cleared area.
- **A message sent before the driver is up is silently dropped.** That is why `sit_controller` waits for a listener first (a fix ported from Luis's work, [Decisions](decisions.md) row 15).
- **The `parameter` field matters.** We standardized on `"{}"` (empty JSON) for Sit. Riley's agent sent `""`; only its Sit was changed ([Decisions](decisions.md) row 11). A few moves, like Move, BodyHeight and SpeedLevel, need real arguments; check Unitree's API docs.
- Sit and stand were **not run yet** in the 2026-10-11 lab session ([Test log](test-log.md)).

## Who's on it and what's next

Nobody is working on posture this week. TODO(Max): whether the placeholder `front_leg_controller` node should be finished or removed.

## The code

Folder: https://github.com/maxhawksby/go2-fusion/tree/master/src/front_leg_controller

The full list of moves is in the tables above.

With the robot stack running:

```shell
docker exec -it docker-unitree_ros-1 bash -ic 'ros2 run front_leg_controller sit_controller'
```

Or send any move from the command line, for example FingerHeart (1036):

```shell
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1036, parameter: '{}', priority: 0}"
```

<!-- sources: src/front_leg_controller/README.md; src/front_leg_controller/front_leg_controller/front_leg_controller.py; src/go2_auto_mapper/README.md "Known limitations"; src/go2_llm_agent/README.md; docs/FUSION.md rows 11, 15; docs/TEST_LOG.md 2026-10-11; origin/feat/desktop-app-prototype (PR #14) -->
