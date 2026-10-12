# Oak Control dashboard prototype

This is Noah's laptop-side interface prototype for the Go2 fusion. It is deliberately isolated from ROS and cannot
send anything to the robot. It demonstrates the intended controls, command interpretation and visualization layout
while the shared stop and keyboard-control fixes are reviewed.

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
- Try `stand up`, `sit down`, `stretch`, `recover`, `move forward`, `turn left`, `dance`, or `stop`.
- Use the quick-action buttons and inspect the command history.

## Safety boundary

The prototype has no ROS dependency, network client or robot credentials. Every command stays in the browser. The
future ROS adapter should be added only after the zero-velocity and keyboard-idle fixes reach `master` and are verified
on the robot.

## Planned integration

1. Add a small ROS 2 adapter behind an explicit Mock/Robot mode boundary.
2. Give the dashboard its own twist-mux input rather than sharing another controller's topic.
3. Publish posture and Sport API actions on `/webrtc_req`.
4. Reuse `/tts`, `/detected_objects_dict`, `/navigate_to_object`, and the existing Foxglove bridge.
5. Keep `stop`, speed limits, press-and-hold movement, and connection loss handling local and deterministic.
