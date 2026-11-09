---
title: Team
parent: oak
---

# Team

Welcome to the Team hub: this is where you meet the five people building Oak, see where we are headed, and learn why things are the way they are.

Part of: [Oak](../../README.md)

## Who works on Oak

Oak is built by **Team 48**, a five-student senior design team (the dashboard's title reads "UConn Team 48"). Each person has a main focus this week:

| Person | Working on now | Closest pages |
| --- | --- | --- |
| **Luis** | Auto mapper: making Oak explore a room reliably | [Auto mapper](auto-mapper.md), [SLAM](slam.md), [Nav2](nav2.md) |
| **Adib** | Local Ollama/Qwen [LLM agent](llm-agent.md); next, odometry and turn accuracy | [LLM agent](llm-agent.md) |
| **Noah** | Mapping scoreboard: run recording, reports and replay; later the Oak Control dashboard | [Scoreboard](scoreboard.md), [Maps and navigation](maps-nav.md) |
| **Jeth** | Piper text-to-speech through Oak's speaker | [Voice](voice.md) |
| **Max** | Computer vision: YOLO object memory; leads the wiki | [Object detection](object-detection.md), this hub |

More detail on each person, and how to reach them, is on [People](people.md).

## How we decide direction

- The team's goal and current milestone are on the [Roadmap](roadmap.md).
- Big choices, and why we made them, are written down in [Decisions](decisions.md).
- Each person picks up a focus area and works on their own branch. Team members post weekly updates; the "Who's on it" sections across this wiki come from those.
- TODO(Max): who has the final say on direction (the whole team, a team lead, or a course advisor), and when the team meets.

## How we work together

- **Branches are your sandbox.** Push whatever you like to your own branch; no review needed.
- **Merging into `master`** needs a GitHub pull request with one teammate's approval and green automatic checks. Nobody pushes to `master` directly.
- **Review is for the robot.** A change to Oak's software (`src/`, `docker/`, `host_tools/`) or its build check needs one teammate's approval, even if it already works on Oak. Wiki pages, guides and team tooling (wiki site, Discord bot) skip it: once the checks pass, Max merges them.
- **Tasks live as GitHub issues**, labelled `wiki:<page>` (for example `wiki:maps-nav`). Take one by assigning it to yourself.
- **Keep the wiki true.** If you change a program's code, update its wiki page in the same pull request. A check fails otherwise.
- **Test without the robot first.** There is one Oak, and it accepts one connection at a time.
- **Never commit keys, passwords, or lab Wi-Fi details.** Ask a teammate for access details privately.

The same rules are written for humans in `CONTRIBUTING.md` and for AI coding agents in `AGENTS.md`.

## The pages, in reading order

1. [People](people.md): one section per teammate: focus, current work, and the pages to ask them about.
2. [Roadmap](roadmap.md): the big vision, the milestone we are on, and what comes next.
3. [Decisions](decisions.md): how two codebases were fused into this repo, and every key choice since.

## How Team connects to the other hubs

Every hub has a "Who's on it" section that points back here.

- [Maps & Nav](maps-nav.md) is where the current milestone (Oak maps a room by itself) lives.
- [Brains](brains.md) is where the biggest change of direction happened: the agent moved to a free local model.
- [Senses](senses.md) holds Max's object memory work.
- [Lab setup](lab-setup.md) holds the [Test log](test-log.md), whose open issues feed the roadmap.
- [Driving](driving.md) explains the drive referee that several [Decisions](decisions.md) were about.

## Thanks

Oak stands on other people's work: the open-source [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) by @abizovnuralem and its contributors, and Riley Francis's [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav), which brought the YOLO detector, object navigation, the first LLM agent and the speech-to-text tool. See [People](people.md).

<!-- sources: team updates week of 2026-10-18 (contract); AGENTS.md (Git, Wiki, design rules); README.md "Team and how we work"; origin/feat/desktop-app-prototype index.html title "Oak Control | UConn Team 48"; docs/FUSION.md; README.md credits -->
