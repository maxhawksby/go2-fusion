---
title: Gamepad
parent: driving
code: [src/go2_robot_sdk/config/joystick.yaml]
verified: c0df4a2289b2
---

# Gamepad

The gamepad lane lets a person drive Oak with a hand controller, and it outranks every other drive source.

Part of: [Driving](driving.md)

## Which pad the team uses

The team no longer drives with an Xbox gamepad, and the LB hold-to-drive (deadman) test is no longer required.
The team uses the **Oak ROS2 pad**, which sends its signals differently from an Xbox pad.

How it connects: it talks straight to Oak, like Unitree's R3-1 remote, and skips our ROS software (Max, 2026-10-22). So it never goes through the referee (twist_mux), and nothing in this repo can block or override it. The gamepad lane described below only matters if someone plugs in a pad that publishes `/joy`.

The code below is unchanged, so it still describes what happens if any standard pad is plugged in and publishes `/joy`.

## How it works

Two standard ROS programs do the work, both started by the default launch file `robot.launch.py`:

1. **`joy_node`** (package `joy`) reads a controller plugged into your computer and publishes its sticks and buttons on `/joy`. Its settings come from `joystick.yaml`: device 0 and a 0.05 dead zone, so a resting stick reads zero.
2. **`teleop_twist_joy`** (node name `go2_teleop_node`) turns those sticks into velocity commands (`geometry_msgs/Twist`) on `/cmd_vel_joy`. Its settings live in the `go2_teleop_node` section of `twist_mux.yaml`, which is the file the launch files give it.

`/cmd_vel_joy` is the top lane in [twist_mux](twist-mux.md), priority 10, so an active pad beats the keyboard, Nav2 and everything else.

In `twist_mux.yaml` the teleop node drives only while button 4 (LB on an Xbox pad) is held, a **deadman button**, and sends nothing when you let go.
That stops an idle pad from repeating "speed 0" about 20 times a second, which used to freeze the agent and object navigation.
The same section sets which stick axes drive: forward from axis 1 and sideways from axis 3 (both scaled to 0.5 m/s), and turning from axis 6 (scaled to 1.0 rad/s).

Separately, the [Go2 driver](go2-driver.md) also reads `/joy` itself: button 0 sends Stand Up and button 1 sends Stand Down. These go straight to the robot, not through twist_mux.

## Connects to

- [twist_mux](twist-mux.md): `/cmd_vel_joy`, priority 10.
- [Go2 driver](go2-driver.md): reads `/joy` for stand up / stand down.
- Launch switch: `joystick:=false` turns off `joy_node` and `teleop_twist_joy`.

## How it breaks

- **Never tested on Oak with an Xbox-style pad.** On 2026-10-11 the lab had none (see [Test log](test-log.md)). The deadman logic was tested only in Docker with simulated input. The team now uses the Oak ROS2 pad instead, so this test is no longer needed.
- **The Oak ROS2 pad always wins.** It talks to Oak directly, so a person holding it can stop the dog even if our software misbehaves, and our code can't stop the pad.
- **On Windows without WSL** there is no `/dev/input`, so the container cannot see a controller at all (see [WSL and Windows](wsl.md)).

## Who's on it and what's next

TODO(owner): no one owns the gamepad lane. The Oak ROS2 pad needs no code from us.

## The code

Config: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/config

Check it without the robot, on a Linux or WSL host with a pad plugged in (the default compose file maps `/dev/input` into the container):

```shell
docker compose exec unitree_ros bash -ic 'ros2 topic echo /joy'
docker compose exec unitree_ros bash -ic 'ros2 topic echo /cmd_vel_joy'
```

Press buttons to learn their numbers in the first; hold button 4 and move a stick to see commands in the second.

<!-- sources: src/go2_robot_sdk/config/joystick.yaml (joy_node); src/go2_robot_sdk/config/twist_mux.yaml (go2_teleop_node: require_enable_button, enable_button 4, axes, scales); src/go2_robot_sdk/launch/robot.launch.py, robot_cpp.launch.py, navigation.launch.py, mapping.launch.py (joy_node parameters=joystick, teleop_node parameters=twist_mux); src/go2_robot_sdk/application/services/robot_control_service.py handle_joy_command; presentation/go2_driver_node.py joy subscription; docker/docker-compose.yml devices; docs/WINDOWS_SETUP.md "Why a separate compose file"; docs/FUSION.md row 13 and follow-up 5; docs/TEST_LOG.md "Gamepad LB deadman: not run" and open issue 6; Max 2026-10-22: team uses the Oak ROS2 pad, LB deadman not required -->
