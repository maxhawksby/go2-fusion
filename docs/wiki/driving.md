---
title: Driving
parent: oak
---

# Driving

Welcome to Driving: this is how a person steers Oak by hand, and how Oak decides whose drive command to obey when several programs want to move it at once.

Part of: [Oak](../../README.md)

## What this part of Oak does

Oak can be driven by many sources: a person at a keyboard, a person with a gamepad, buttons inside RViz (the 3D viewer), Nav2 (the path planner), the auto mapper, the AI agent, and the object follower.
Each source sends the same kind of message: a **velocity command**, written in ROS as a `geometry_msgs/Twist`.
A Twist says "go forward this fast (meters per second), slide sideways this fast, and turn this fast (radians per second)".

If every source published on one shared topic, whoever spoke last would win, and Oak would jerk between them.
So each source gets its own lane (its own ROS topic), and one small program, **twist_mux**, acts as a referee.
It forwards only the most important lane that is currently talking, and passes that one command to the [Driver](driver.md), which sends it to the robot.

A simple rule falls out of this: **people outrank robots**.
If you grab the keyboard while Nav2 is driving, your keys win immediately.
When you let go, your lane goes quiet, and after half a second Nav2 takes over again.

## Pages, in reading order

1. [twist_mux](twist-mux.md) - the referee. Read this first: it explains the lanes and their priorities.
2. [Keyboard teleop](keyboard-teleop.md) - drive Oak from a terminal with W, A, S, D. The easiest way to start.
3. [RViz panels](rviz-panels.md) - three buttons-and-maps panels inside RViz: click on the map to send Oak somewhere, run the auto mapper, or drive with the keyboard without a terminal.
4. [Gamepad](gamepad.md) - the hand-controller lane. The team now uses the Oak ROS2 pad, not an Xbox gamepad, so the old hold-LB test is no longer needed.

"Teleop" is short for teleoperation: driving a robot remotely by hand.

## The lanes at a glance

| Priority (higher wins) | Topic | Who drives on it |
| --- | --- | --- |
| 10 | `/cmd_vel_joy` | Gamepad |
| 9 | `/cmd_vel_keyboard` | Keyboard teleop and the RViz KeyboardTeleop panel |
| 6 | `/cmd_vel_mapper` | Auto mapper recovery moves |
| 5 | `/cmd_vel` | Nav2 (and anything that goes through Nav2, like map clicks) |
| 4 | `/cmd_vel_agent` | The AI agent |
| 3 | `/cmd_vel_object_nav` | Object navigation |

The winner comes out on `/cmd_vel_out`, the only drive topic the driver listens to.
The full story, and why each lane sits where it does, is on the [twist_mux](twist-mux.md) page.

## Safety habits

- **Test without the robot first.** There is one Oak and it accepts one connection at a time. Run your drive program in Docker and watch what it publishes with `ros2 topic echo /cmd_vel_keyboard` (or your own lane). The [README](../../README.md) dry run shows exactly this.
- **Never send motion to the live robot unless you mean to**, and only with someone watching Oak.
- **Stopping matters as much as going.** In an early lab test, a stop command was dropped and Oak walked into a wall. That bug is fixed, but always keep a hand near Space (keyboard stop).

## How Driving connects to the rest of Oak

- **[Driver](driver.md):** receives the winning command on `/cmd_vel_out` and sends it to the robot over Wi-Fi.
- **[Maps & Nav](maps-nav.md):** [Nav2](nav2.md) and the [auto mapper](auto-mapper.md) drive on their own lanes. The [map viewers](map-viewers.md) and the RViz MapViewer panel send goals to Nav2 rather than driving directly.
- **[Brains](brains.md):** the [LLM agent](llm-agent.md) and [object navigation](object-navigation.md) drive on the two lowest lanes, so a person can always take over.
- **[Lab setup](lab-setup.md):** everything here runs inside the Docker container described there.

## Who's on it

- Luis made the RViz KeyboardTeleop panel react instantly to key presses (merged 2026-10-19) and added the AutoMapper panel's Scan and Reset maps buttons.
- Max fixed the "idle zeros" bug (both keyboard tools used to send stop commands nonstop, which blocked Nav2 and the agent) on 2026-10-11.
- TODO(owner): a person who owns Driving as a whole, and the next step after the open retest of keyboard driving feel.
