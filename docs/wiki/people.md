---
title: People
parent: team
code: [CONTRIBUTING.md, AGENTS.md]
verified: af488b6dcf11
---

# People

Here are the five teammates who build Oak, what each one works on, and the people whose work Oak is built on.

Part of: [Team](team.md)

**How to reach anyone:** ask in the team Discord. For a question about a specific page, you can also open a GitHub issue labelled `wiki:<page>`.

**See who is working on what right now:** open the team wiki on your laptop (`npm run wiki` from the repo folder; it needs Node.js 18+). Each person's color marks their open work and recent changes, and clicking a name shows their pages.

## Luis

- **GitHub:** [LuisMend12](https://github.com/LuisMend12) (commits also show as LuisM23). TODO(Max): Luis's full name.
- **Lane:** exploration brain: the auto mapper, frontier choice and Nav2 tuning ([Roadmap](roadmap.md), issue [#25](https://github.com/maxhawksby/go2-fusion/issues/25)).
- **Pages to ask about:** [Auto mapper](auto-mapper.md), [SLAM](slam.md), [Nav2](nav2.md), [RViz panels](rviz-panels.md), [Maps & Nav](maps-nav.md). He also added the auto mapper's own [twist_mux](twist-mux.md) lane, `/cmd_vel_mapper` (priority 6).
- **Current work:** their open [issues and pull requests](https://github.com/maxhawksby/go2-fusion/issues?q=is%3Aopen+involves%3ALuisMend12) on GitHub.

## Adib

- **GitHub:** [AdibRouf](https://github.com/AdibRouf). Adib Rouf.
- **Lane:** localization: the TF tree, odometry timing, SLAM settings, drift and turns ([Roadmap](roadmap.md), issue [#26](https://github.com/maxhawksby/go2-fusion/issues/26)).
- **Pages to ask about:** [LLM agent](llm-agent.md). He also wrote the macOS Docker setup (pull request #6).
- **Current work:** their open [issues and pull requests](https://github.com/maxhawksby/go2-fusion/issues?q=is%3Aopen+involves%3AAdibRouf) on GitHub.

## Noah

- **GitHub:** [noahsteinberg](https://github.com/noahsteinberg). Noah Steinberg.
- **Lane:** the scoreboard: run logger, recordings and replay ([Roadmap](roadmap.md), issue [#27](https://github.com/maxhawksby/go2-fusion/issues/27)).
- **Pages to ask about:** [Voice](voice.md) (Piper review), [Posture](posture.md) (dashboard quick actions), [Lab setup](lab-setup.md). He also moved the packages into `src/` and organized the root docs (pull requests #7 and #9).
- **Current work:** their open [issues and pull requests](https://github.com/maxhawksby/go2-fusion/issues?q=is%3Aopen+involves%3Anoahsteinberg) on GitHub.

## Jeth

- **GitHub:** commits appear as `jevgome`. TODO(Max): confirm that is Jeth's GitHub username, and Jeth's full name.
- **Lane:** the simulator and the answer key: a Gazebo world of the lab room and tape-measured floor plans ([Roadmap](roadmap.md), issue [#28](https://github.com/maxhawksby/go2-fusion/issues/28)).
- **Pages to ask about:** [Voice](voice.md).
- **Current work:** their open [issues and pull requests](https://github.com/maxhawksby/go2-fusion/issues?q=is%3Aopen+involves%3Ajevgome) on GitHub.

## Max

- **GitHub:** [maxhawksby](https://github.com/maxhawksby). Max Hawksby.
- **Lane:** recovery, the plan and the technical report: sensor watchdog, save and resume the map, failure drills ([Roadmap](roadmap.md), issue [#29](https://github.com/maxhawksby/go2-fusion/issues/29)).
- **Pages to ask about:** [Object detection](object-detection.md), [Team](team.md), and the wiki itself.
- **Current work:** their open [issues and pull requests](https://github.com/maxhawksby/go2-fusion/issues?q=is%3Aopen+involves%3Amaxhawksby) on GitHub.

## How we work together

- **Claim your work.** Tell your agent what you're starting; it opens or finds the issue, tags the page, and assigns you, so your name shows on the map.
- **Put your name on your branch.** Start it with your id from [hypatia.yml](../hypatia.yml): `max/fix/nav-obstacle-safety`. Anyone can see who owns it.
- **Say what each commit does.** One short sentence, starting with the part of Oak it touches: `Auto mapper: skip frontiers it can't reach`. Your branch's messages show on your People tab, so they tell the team what you're on.
- **Ask for review.** A pull request with Who, What, Why, Before and After posts to #feature-revisions; one teammate approves before it reaches `master`. Only changes to Oak's software need that approval; wiki, guides and team tooling skip it and Max merges them once checks pass. Full steps: [CONTRIBUTING.md](../../CONTRIBUTING.md).

## People Oak builds on

- **Riley Francis** wrote [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav): the YOLO detector, [Object navigation](object-navigation.md), the first [LLM agent](llm-agent.md) and the speech-to-text tool. See [Decisions](decisions.md).
- **@abizovnuralem** and contributors wrote [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk), the base of the driver. Go there for upstream support.
- **@tfoldi** wrote [go2-webrtc](https://github.com/tfoldi/go2-webrtc), used to connect over Wi-Fi.

<!-- sources: team updates week of 2026-10-18 (contract); git log --all authors; gh pr list (PRs #6, #7, #9, #10, #11, #13, #14 and authors); origin/feat/desktop-app-prototype host_tools/control_dashboard/README.md and src/go2_dashboard_bridge/README.md; origin/jeth/feat/tts (jevgome); origin/adib/feat/llm-agent; docs/FUSION.md; README.md credits -->
