# object_navigation

Walks the robot straight toward an object `object_detection` has seen (publish an ID like `chair_1`) until it is about 0.5 m away. **No obstacle avoidance** - only run it with someone watching the robot.

## Run

Inside the container, with the robot stack and `object_detection` running:

```shell
ros2 run object_navigation navigation_node
ros2 topic echo /detected_objects_dict                                  # pick an ID
ros2 topic pub /navigate_to_object std_msgs/String "data: 'chair_1'" --once
```

To stop it, drive with the keyboard or gamepad (they override it), or Ctrl+C the node.

Full explanation: [Object navigation wiki page](../../docs/wiki/object-navigation.md)
