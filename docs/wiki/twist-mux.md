---
title: twist_mux
parent: driving
code: [src/go2_robot_sdk/config/twist_mux.yaml]
verified: 0671c7464e84
---

# twist_mux

twist_mux is Oak's referee for driving: many programs want to move the dog, and twist_mux picks exactly one of them to obey at any moment.

Part of: [Driving](driving.md)

## How it works

Every drive source publishes a velocity command (a `geometry_msgs/Twist`: forward speed, sideways speed, turn speed) on its **own** ROS topic, called an input or lane.
twist_mux listens to all the lanes at once.
Each lane has a **priority** number and a **timeout** of 0.5 seconds.
A lane counts as "active" if it published something in the last 0.5 seconds.
twist_mux forwards the active lane with the highest priority to `/cmd_vel_out`, which is the only drive topic the [Go2 driver](go2-driver.md) listens to.

The lanes, from `src/go2_robot_sdk/config/twist_mux.yaml`:

| Priority | Topic | Who publishes it |
| --- | --- | --- |
| 10 | `/cmd_vel_joy` | [Gamepad](gamepad.md) via `teleop_twist_joy`, only while button 4 (LB) is held |
| 9 | `/cmd_vel_keyboard` | [Keyboard teleop](keyboard-teleop.md) and the [RViz KeyboardTeleop panel](rviz-panels.md) |
| 6 | `/cmd_vel_mapper` | [Auto mapper](auto-mapper.md) recovery moves (beats Nav2, loses to people) |
| 5 | `/cmd_vel` | [Nav2](nav2.md), and anything else publishing plain `cmd_vel` |
| 4 | `/cmd_vel_agent` | [LLM agent](llm-agent.md) |
| 3 | `/cmd_vel_object_nav` | [Object navigation](object-navigation.md), which has no obstacle avoidance, so it yields to everyone |

Example: Nav2 is walking Oak to a goal on priority 5.
You press W in keyboard teleop, so priority 9 becomes active and wins at once.
You let go, the keyboard sends one stop and goes quiet, and 0.5 seconds later Nav2's lane is the highest active one again.
This exact sequence (object nav, then keyboard wins, then object nav resumes) passed on the robot on 2026-10-11.

## The one rule for new code

**A new drive source gets its own lane.** Add a row to `twist_mux.yaml` and publish there.
Before the fusion, the keyboard, the agent and object navigation all published on `/cmd_vel`, the same lane as Nav2, so whoever published last won.
That is why the lanes exist.

A second rule: **do not publish when idle.** A lane that keeps sending zeros stays "active" and blocks every lower lane.
Both the gamepad and the keyboard tools had this bug, and both were fixed to publish only while driving, plus one stop at the end.

## Connects to

- [Keyboard teleop](keyboard-teleop.md), [Gamepad](gamepad.md), [RViz panels](rviz-panels.md): the human lanes.
- [Nav2](nav2.md), [Auto mapper](auto-mapper.md), [LLM agent](llm-agent.md), [Object navigation](object-navigation.md): the autonomous lanes.
- [Go2 driver](go2-driver.md): reads `/cmd_vel_out` and sends it to the robot.

## Which launch files start it

- `robot.launch.py` (the default in Docker) starts twist_mux if `joystick` **or** `teleop` is on. Both are on by default.
- `navigation.launch.py` and `mapping.launch.py` start it only if `joystick` is on. So `joystick:=false` there cuts off every lane, not just the gamepad. The same bug was fixed in `robot.launch.py` but not in these two. TODO(owner): fix or confirm these launch files are unused.
- `webrtc_web.launch.py` skips twist_mux completely and connects the driver straight to `/cmd_vel`, so the keyboard, agent and object-navigation lanes do nothing there.

## How it breaks

- **A lane that never goes quiet** blocks everything below it (the idle-zeros bug above, found on the robot 2026-10-11).
- **Shared lanes** make sources fight. Give each source its own.

## Who's on it and what's next

TODO(owner): no one owns twist_mux right now. Luis added the mapper lane for the auto mapper's recovery moves.

## The code

Config: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/config

Watch the referee's output without the robot (in a container shell, see [Docker](docker.md)):

```shell
ros2 topic echo /cmd_vel_out
```

Then drive with `teleop` in a second shell and watch your commands come through.

<!-- sources: src/go2_robot_sdk/config/twist_mux.yaml; src/go2_robot_sdk/launch/robot.launch.py create_teleop_nodes; navigation.launch.py and mapping.launch.py twist_mux condition; webrtc_web.launch.py remap cmd_vel_out->cmd_vel; docs/FUSION.md rows 2, 13, 15(b), 16, "twist_mux priorities", follow-up 6; docs/TEST_LOG.md twist_mux priority pass; AGENTS.md Design rules -->
