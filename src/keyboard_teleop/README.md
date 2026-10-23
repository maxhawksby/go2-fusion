# keyboard_teleop

Drive the Go2 from a terminal keyboard: publishes `geometry_msgs/msg/Twist` on `/cmd_vel_keyboard` (twist_mux priority 9).

## Run

Needs an interactive terminal. From `docker/`, with the robot stack running:

```shell
docker compose exec unitree_ros bash -ic teleop    # teleop = ros2 run keyboard_teleop keyboard_teleop
```

`W`/`S` forward-back, `A`/`D` turn, `Q`/`E` strafe, `Z`/`X` speed down/up, `Space` stop, `Esc` quit. Python edits need no rebuild: quit and rerun.

Full explanation: [Keyboard teleop wiki page](../../docs/wiki/keyboard-teleop.md)
