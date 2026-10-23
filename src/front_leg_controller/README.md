# front_leg_controller

Two nodes: `sit_controller` sends a real Sit command (API 1009) to the robot over `/webrtc_req`; `front_leg_controller` is an inert placeholder that only logs.

## Run

With the robot stack running (this moves the real robot - only in the lab, watching Oak, in a cleared area):

```shell
docker exec -it docker-unitree_ros-1 bash -ic 'ros2 run front_leg_controller sit_controller'
```

Send any other move from the command line, for example FingerHeart (1036):

```shell
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1036, parameter: '{}', priority: 0}"
```

Full explanation, including every move's API ID: [Posture wiki page](../../docs/wiki/posture.md)
