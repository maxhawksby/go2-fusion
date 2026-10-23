---
title: Object navigation
parent: brains
code: [src/object_navigation/**]
---

# Object navigation

Object navigation makes Oak walk up to something its camera has seen: tell it `chair_1`, and Oak turns toward the chair and walks until it is about half a meter away.

Part of: [Brains](brains.md)

## How it works

[Object detection](object-detection.md) watches the camera and keeps a short memory of what it saw, with a rough position for each object. It publishes that memory as a list on `/detected_objects_dict`. Each object gets an ID like `chair_1` or `person_2`.

When you publish an ID on `/navigate_to_object`, this node locks onto that object. Ten times a second it:

1. Checks the object is still in the detector's memory. If not, it stops and gives up.
2. Turns toward it, faster when the angle is bigger: turn speed = 1.5 x angle, capped at 1.0 rad/s.
3. Walks forward only when roughly facing it (angle under 0.3 rad): forward speed = 0.5 x distance, capped at 0.4 m/s.
4. Stops when closer than 0.5 m.

**It has no obstacle avoidance.** It drives in a straight line toward a rough guess of where the object is. It uses no map, no Nav2 path planning and no TF lookup. That is why it has the lowest priority on the drive referee: everything else overrides it.

## Connects to

- [Object detection](object-detection.md): the source of `/detected_objects_dict`. Objects are forgotten 20 seconds after the detector last saw them.
- [twist_mux](twist-mux.md): it drives on `/cmd_vel_object_nav`, priority 3, the lowest. The gamepad, keyboard, Nav2 and the [LLM agent](llm-agent.md) all win over it.
- [Keyboard teleop](keyboard-teleop.md) and [Gamepad](gamepad.md): your way to stop it. Driving with either overrides it.

| Direction | Topic | What |
| --- | --- | --- |
| Listens | `/detected_objects_dict` | The detector's object memory (JSON text) |
| Listens | `/navigate_to_object` | The target ID, such as `chair_1` |
| Publishes | `/cmd_vel_object_nav` | Walk and turn speeds, 10 times a second |

## How it breaks

- **The target ID must match exactly.** A name that is not in the memory is ignored with a warning.
- **It assumes object positions are already relative to Oak's body**, and they are only as good as the detector's rough estimate.
- **It stops if the object leaves the memory**, for example when the camera loses sight of it for 20 s.
- **An idle keyboard or gamepad used to freeze it.** Before the fixes in [Decisions](decisions.md) (rows 13 and 16), idle controllers kept sending "speed 0" on higher-priority inputs, which blocked object navigation.
- In the 2026-10-11 lab test, the referee worked: object nav drove forward, the keyboard took over for a turn, then object nav resumed. "Object nav to a chair" end to end was **not run yet** ([Test log](test-log.md)).

## Who's on it and what's next

Written by Riley Francis in [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav). Nobody is working on it this week. Max's open YOLO memory work (pull request #11, objects remembered on the map) touches this node for small style fixes. TODO(Max): who owns object navigation next, and whether it should go through Nav2 for obstacle avoidance.

## The code

Folder: https://github.com/maxhawksby/go2-fusion/tree/master/src/object_navigation

Inside the container, with the robot stack and object detection running:

```shell
ros2 run object_navigation navigation_node
ros2 topic echo /detected_objects_dict                                  # pick an ID
ros2 topic pub /navigate_to_object std_msgs/String "data: 'chair_1'" --once
```

To stop it, drive with the keyboard or gamepad, or press Ctrl+C on the node. Only run it with someone watching Oak.

<!-- sources: src/object_navigation/README.md; src/object_navigation/object_navigation/navigation_node.py; docs/FUSION.md rows 2, 4, 13, 16 and twist_mux table; docs/TEST_LOG.md 2026-10-11; AGENTS.md; PR #11 / origin/feat/yolo-map-memory (2e66325) -->
