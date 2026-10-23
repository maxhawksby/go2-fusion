---
title: Keyboard teleop
parent: driving
code: [src/keyboard_teleop/**]
---

# Keyboard teleop

Keyboard teleop lets you drive Oak from a terminal with W, A, S, D, and it is the easiest way to move the robot by hand or to test driving without the robot at all.

Part of: [Driving](driving.md)

## How it works

`keyboard_teleop` is a small Python ROS node.
It only sends movement commands and never connects to the robot itself, so the robot stack must already be running for Oak to move.
It reads key presses straight from your terminal and turns them into velocity commands (`geometry_msgs/Twist`) on `/cmd_vel_keyboard`.
That topic is the keyboard's lane in [twist_mux](twist-mux.md), with priority 9: only the gamepad beats it, and it beats Nav2 and all the autonomous programs.

| Key | Action |
| --- | --- |
| `W` / `S` | Forward / backward |
| `A` / `D` | Turn left / right |
| `Q` / `E` | Slide (strafe) left / right |
| `Z` / `X` | Slower / faster |
| `Space` | Stop |
| `Esc` | Quit |

It starts at 0.5 m/s forward and 1.5 rad/s turning. Each Z or X press changes speed by 0.1 m/s and 0.3 rad/s, within 0.1-1.0 m/s and 0.3-2.5 rad/s.

A terminal cannot tell when you *release* a key; it only sees repeated presses.
So each direction stays on for 0.4 seconds after its key was last seen, then zeroes out by itself.
That lets you curve (hold W, tap D) and means Oak stops on its own if you walk away.
While moving, the node publishes 10 times a second; when motion ends it sends **one** stop and then goes silent, so it never blocks Nav2 or the agent.

## Connects to

- [twist_mux](twist-mux.md): forwards `/cmd_vel_keyboard` to the driver when it is the highest active lane.
- [RViz panels](rviz-panels.md): the KeyboardTeleop panel does the same job inside RViz and uses the same topic.
- [Go2 driver](go2-driver.md): receives the result on `/cmd_vel_out`.

## How it breaks

- **Not a real terminal:** it needs an interactive terminal. If input is piped or redirected, it logs an error and does nothing.
- **Idle zeros (fixed):** it used to publish zeros 10 times a second even when idle, which blocked Nav2, the agent and object navigation. Found on the robot 2026-10-11 and fixed to publish only while driving.
- **Driving feel:** on 2026-10-11 the dog moved and stopped, but driving "really isn't that good". A retest after the fixes is still open in the [test log](test-log.md).
- **A fix does not show up:** Python edits normally need only a restart, but a stale install can hide them. Rebuild with `cbs keyboard_teleop`.

## Who's on it and what's next

Max fixed the idle-zeros bug. Next is the open retest of driving feel on Oak. TODO(owner): an owner for this package.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/keyboard_teleop

Try it with no robot (from the `docker/` folder; see [Docker](docker.md)):

```shell
docker compose run --rm --name oak-dry unitree_ros bash
teleop
```

`teleop` is a shortcut for `ros2 run keyboard_teleop keyboard_teleop`.
In a second terminal, watch what it sends: hold W and a forward speed appears; let go and one stop follows.

```shell
docker exec -it oak-dry bash -ic 'ros2 topic echo /cmd_vel_keyboard'
```

With the robot stack already running, use `docker compose exec unitree_ros bash -ic teleop` instead.

<!-- sources: src/keyboard_teleop/README.md; src/keyboard_teleop/keyboard_teleop/keyboard_teleop.py (speeds, AXIS_HOLD_TIMEOUT 0.4, PUBLISH_RATE_HZ 10, publish-only-while-moving); docker/dev_aliases.sh (teleop alias, cbs); README.md dry run; docs/FUSION.md row 16; docs/TEST_LOG.md 2026-10-11 keyboard partial and idle fail; docs/USEFUL_COMMANDS.md "Rebuilding after a source change"; git log c18ca87 -->
