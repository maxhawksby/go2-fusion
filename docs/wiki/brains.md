---
title: Brains
parent: oak
---

# Brains

Welcome to Brains, the part of Oak that decides *what to do*: it turns a sentence like "walk forward then sit" into moves, walks over to objects the camera has spotted, and gives Oak a voice.

Part of: [Oak](../../README.md)

## What this part does

The other hubs give Oak a body and senses. [Driving](driving.md) moves it, the [Driver](driver.md) talks to the real robot, and [Senses](senses.md) sees the world. Brains sits on top of all of that.

None of the programs here move Oak's legs directly. They work by sending messages on *topics* (named channels that ROS 2 programs use to pass data). For example, a brain program asks to walk by publishing a speed on its own drive topic. A referee called [twist_mux](twist-mux.md) then decides whose drive command wins. People with a keyboard or gamepad always win over the brains, so a human can take over at any moment.

Most of this hub came from Riley Francis's project [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav), which our team folded into this repo (see [Decisions](decisions.md)). Some of it works today, and some is still being rebuilt.

## The pages, in reading order

1. [LLM agent](llm-agent.md): reads a plain-English command and turns it into a short list of robot actions. An *LLM* (large language model) is the kind of AI behind chatbots. The team is moving it to a free model that runs on your own laptop.
2. [Object navigation](object-navigation.md): give it the name of an object the camera saw, like `chair_1`, and Oak walks toward it. It has no obstacle avoidance, so someone must watch.
3. [Voice](voice.md): speech in (your microphone, turned into text) and speech out (text, spoken through Oak's speaker).
4. [Posture](posture.md): sit, stand, stretch and other built-in moves, sent as one-off commands.

## What works today

- **Object navigation** works. In the 2026-10-11 lab test it drove Oak forward, gave way when the keyboard took over, then carried on ([Test log](test-log.md)).
- **Posture** works through `sit_controller`, which sends a real Sit command.
- **Speech out** works with a paid ElevenLabs voice on `master`. A free local voice (Piper) is being tested on branches.
- **The LLM agent** builds but does not run on `master`. Its new version lives on a branch.

## How Brains talks to the rest of Oak

| Topic | What travels on it | Who uses it here |
| --- | --- | --- |
| `/cmd_vel_agent` | Walk and turn speeds (priority 4 in twist_mux) | [LLM agent](llm-agent.md) |
| `/cmd_vel_object_nav` | Walk and turn speeds (priority 3, the lowest) | [Object navigation](object-navigation.md) |
| `/webrtc_req` | One-off robot commands like Sit (1009) or Stand Up (1004) | [Posture](posture.md), [LLM agent](llm-agent.md), [Voice](voice.md) |
| `/tts` | Text for Oak to speak | [Voice](voice.md), [LLM agent](llm-agent.md) |
| `/detected_objects_dict` | The list of objects the camera remembers, with rough positions | [Object navigation](object-navigation.md), from [Object detection](object-detection.md) |
| `/odom` | Where Oak thinks it is and which way it faces | The new [LLM agent](llm-agent.md) uses it to check turns |

Higher priority wins in twist_mux: the gamepad (10) and keyboard (9) beat Nav2 (5), which beats the agent (4), which beats object navigation (3). Keyboard and gamepad are your emergency brake. See [twist_mux](twist-mux.md).

## Connects to other hubs

- [Senses](senses.md): [Object detection](object-detection.md) feeds object navigation the objects it has seen.
- [Driving](driving.md): every move from Brains goes through [twist_mux](twist-mux.md).
- [Driver](driver.md): the [Go2 driver](go2-driver.md) carries `/webrtc_req` and `/tts` to the real robot.
- [Maps & Nav](maps-nav.md): Nav2 outranks the brains on the drive referee.
- [Team](team.md): the [Roadmap](roadmap.md) and [Decisions](decisions.md) explain why the agent changed direction.

## Who's on it

- **Adib**: the LLM agent, now on a local Ollama model. Next he moves on to odometry (position tracking) and turn accuracy.
- **Jeth**: Piper text-to-speech, with Noah reviewing.
- **Max**: object detection memory, which object navigation reads.

See [People](people.md) for how to reach everyone.

## Safety first

There is one Oak, and it accepts one connection at a time. Test with mock modes and `ros2 topic echo <topic>` first. Only send real moves when you are in the lab, next to Oak, with the keyboard or gamepad ready to take over.
