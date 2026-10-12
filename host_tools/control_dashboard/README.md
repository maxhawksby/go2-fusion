# Oak Control dashboard prototype

This is Noah's laptop-side interface for the Go2 fusion. It starts in Mock Mode. Robot Mode talks only to the local
`go2_dashboard_bridge` API and shows live ROS topic status plus normalized object detections.

## Run

From the repository root:

```powershell
node host_tools/control_dashboard/server.mjs
```

Open <http://localhost:3000> in Chrome or Edge. Those browsers may also provide the Web Speech API used by the
push-to-talk prototype. Typed commands always work.

Stop the server with `Ctrl+C`.

## Try it

- Start the simulated connection and watch the sensor cards change state.
- Hold WASD or the on-screen direction buttons. Releasing the control produces one mock stop event.
- Press the voice button to start listening, then press it again to stop and process the recognized phrase.
- Try `stand up`, `sit down`, `stretch`, `recover`, `show me love`, `dance`, `say I am here`, or `stop`.
- Use the quick-action buttons and inspect the command history.
- Switch to Robot Mode to read the bridge at `http://localhost:8080`.

## Safety boundary

Mock Mode cannot reach ROS. The bridge starts read-only, and live actions return an error unless it is deliberately
started with `allow_commands:=true`. Live WASD and voice movement remain disabled in Robot Mode.

## Planned integration

Inside the running ROS container, build and start the bridge read-only:

```shell
cbs go2_dashboard_bridge
ros2 run go2_dashboard_bridge dashboard_bridge
```

When physically present with the robot, opt into posture, gesture, Stop, and TTS commands:

```shell
ros2 run go2_dashboard_bridge dashboard_bridge --ros-args -p allow_commands:=true
```

The next control milestone is a dedicated twist-mux input for the dashboard. It needs speed limits, press-and-hold
publishing, one stop on release, focus/connection-loss stops, and a robot test before live WASD is enabled.

The Go2 product image in `assets/unitree-go2.jpg` comes from the
[official Unitree store](https://shop.unitree.com/en-au/products/unitree-go2).
