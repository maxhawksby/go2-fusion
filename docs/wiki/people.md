---
title: People
parent: team
code: [CONTRIBUTING.md, AGENTS.md]
---

# People

Here are the five teammates who build Oak, what each one works on, and the people whose work Oak is built on.

Part of: [Team](team.md)

**How to reach anyone:** ask in the team Discord. For a question about a specific page, you can also open a GitHub issue labelled `wiki:<page>`.

**See who is working on what right now:** open the team wiki on your laptop (`npm install` once, then `npm run wiki` from the repo folder; it needs Node.js 18+ and Python 3.8+). Each person's color marks their open work and recent changes, and clicking a name shows their pages.

## Luis

- **GitHub:** [LuisMend12](https://github.com/LuisMend12) (commits also show as LuisM23). TODO(Max): Luis's full name.
- **Focus:** the auto mapper, so Oak explores and maps a room by itself.
- **Pages to ask about:** [Auto mapper](auto-mapper.md), [SLAM](slam.md), [Nav2](nav2.md), [RViz panels](rviz-panels.md), [Maps & Nav](maps-nav.md). He also added the auto mapper's own [twist_mux](twist-mux.md) lane, `/cmd_vel_mapper` (priority 6).
- **Current work:** Oak kept stopping because floor points that drifted in the 3D point cloud were marked as obstacles, and a 0.5 m lidar cut-off hid nearby walls. Luis took the cloud out of Nav2's obstacle maps, used a filtered 2D scan, lowered the minimum range to 0.1 m, and added RViz buttons to Scan (a 360 degree turn so SLAM can find itself) and Reset maps. The explorer now picks goals by how much unexplored space they reveal, and frontier search is about 100x faster. Merged to `master`.
- **Next:** full exploration tests on Oak.

## Adib

- **GitHub:** [AdibRouf](https://github.com/AdibRouf). Adib Rouf.
- **Focus:** the [LLM agent](llm-agent.md), then odometry.
- **Pages to ask about:** [LLM agent](llm-agent.md). He also wrote the macOS Docker setup (pull request #6).
- **Current work:** replaced the paid OpenAI model with Ollama running a local Qwen model, on branch `adib/feat/llm-agent`. Tested in mock mode.
- **Next:** odometry reliability (why position data sometimes stops) and turn accuracy (requested vs measured rotation).

## Noah

- **GitHub:** [noahsteinberg](https://github.com/noahsteinberg). Noah Steinberg.
- **Focus:** the Team 48 **Oak Control dashboard**, a browser page for watching and controlling Oak, plus `go2_dashboard_bridge`, the ROS 2 program it talks to.
- **Pages to ask about:** [Voice](voice.md) (Piper review), [Posture](posture.md) (dashboard quick actions), [Lab setup](lab-setup.md). He also moved the packages into `src/` and organized the root docs (pull requests #7 and #9).
- **Current work:** verified live sensors and object detection over WebRTC and Foxglove; camera feed selection, voice input, robot actions, command parsing, local-network security and control safety. The bridge starts read-only. Wrote a repeatable lab-testing guide. On branch `feat/desktop-app-prototype` (pull request #14) and `noah/tts-piper-review` (pull request #13).
- **Next:** the dashboard's own drive input with a motion lock, before live keyboard driving is turned on there.

## Jeth

- **GitHub:** commits appear as `jevgome`. TODO(Max): confirm that is Jeth's GitHub username, and Jeth's full name.
- **Focus:** Piper text-to-speech, so Oak can talk without a paid service.
- **Pages to ask about:** [Voice](voice.md).
- **Current work:** local speech plays through Oak's speaker using a separate `unitree_webrtc_connect` driver and Unitree's megaphone mode. Audio is choppy and drops the start of short clips. On branch `jeth/feat/tts`.
- **Next:** fix the packet loss, then connect text-to-speech to the voice command pipeline.

## Max

- **GitHub:** [maxhawksby](https://github.com/maxhawksby). Max Hawksby.
- **Focus:** computer vision, and the wiki.
- **Pages to ask about:** [Object detection](object-detection.md), [Team](team.md), and the wiki itself.
- **Current work:** YOLO object detection that remembers objects on the map, to make mapping rooms easier (pull request #21). Also open: a navigation safety fix that stops on stale sensors (pull request #20).
- **Next:** TODO(Max).

## How we work together

- **Claim your work.** Tell your agent what you're starting; it opens or finds the issue, tags the page, and assigns you, so your name shows on the map.
- **Put your name on your branch.** Start it with your id from [team.yml](../team.yml): `max/fix/nav-obstacle-safety`. Anyone can see who owns it.
- **Say what each commit does.** One short sentence, starting with the part of Oak it touches: `Auto mapper: skip frontiers it can't reach`. Your branch's messages show on your People tab, so they tell the team what you're on.
- **Ask for review.** A pull request with Who, What, Why, Before and After posts to #feature-revisions; one teammate approves before it reaches `master`. Full steps: [CONTRIBUTING.md](../../CONTRIBUTING.md).

## People Oak builds on

- **Riley Francis** wrote [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav): the YOLO detector, [Object navigation](object-navigation.md), the first [LLM agent](llm-agent.md) and the speech-to-text tool. See [Decisions](decisions.md).
- **@abizovnuralem** and contributors wrote [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk), the base of the driver. Go there for upstream support.
- **@tfoldi** wrote [go2-webrtc](https://github.com/tfoldi/go2-webrtc), used to connect over Wi-Fi.

<!-- sources: team updates week of 2026-10-18 (contract); git log --all authors; gh pr list (PRs #6, #7, #9, #10, #11, #13, #14 and authors); origin/feat/desktop-app-prototype host_tools/control_dashboard/README.md and src/go2_dashboard_bridge/README.md; origin/jeth/feat/tts (jevgome); origin/adib/feat/llm-agent; docs/FUSION.md; README.md credits -->
