---
title: Roadmap
parent: team
---

# Roadmap

This page explains where Oak is headed: the big dream, the milestone we are working on right now, and the next concrete steps.

Part of: [Team](team.md)

## The vision

Real service dogs are very expensive. Our dream is that a robot dog costing about **$3,000**, with computer vision, could do some of that work: for example, guiding a blind person through a building while talking to them through earbuds.

To get there, Oak needs **autonomous mapping**: it must move on its own, with nobody holding the controller, building a map as it goes and finding its way on that map. That needs every hub working together: [Senses](senses.md) to see, [Maps & Nav](maps-nav.md) to map and plan, [Brains](brains.md) to understand and speak, and [Driving](driving.md) as the safety net.

## The current milestone

**Oak maps and explores a room by itself.**

Concretely: start the [Auto mapper](auto-mapper.md), and Oak picks spots at the edge of what it has seen (*frontiers*), walks to them with [Nav2](nav2.md), and builds the map with [SLAM](slam.md), until the room is covered. Luis's latest work fixed the main reasons it kept stopping; it is merged to `master` and now needs full tests on Oak.

## Next steps

From the team's updates (week of 2026-10-18):

1. **Full exploration tests on Oak** with the new auto mapper (Luis).
2. **Odometry and turns:** find why position data sometimes stops, and compare requested vs measured rotation (Adib). Good odometry helps both mapping and the agent's angle turns.
3. **Object memory on the map:** YOLO remembers where objects are, so a map can say "chair here" (Max, pull request #11).
4. **Local voice:** fix Piper's packet loss, then connect speech to voice commands (Jeth, with Noah).
5. **Oak Control dashboard:** a dedicated drive input and motion lock before live driving from the browser (Noah, pull request #14).
6. **Local LLM agent:** bring Adib's Ollama agent into `master` (see [LLM agent](llm-agent.md)).

From the open issues of the last robot test ([Test log](test-log.md)):

1. Retest Nav2 goals and click-to-walk in the [map viewers](map-viewers.md).
2. Retest how keyboard driving feels after the teleop fixes.
3. Find what starts the lidar-to-pointcloud node and check its input topic ([Lidar](lidar.md)).
4. Confirm by eye that Oak stops cleanly when a drive source goes quiet.

There are no open GitHub issues yet. When you take on one of these, open an issue labelled `wiki:<page>` and assign yourself.

TODO(Max): the order of these steps, and dates for the milestone.

## Plans on file

No team plans are on file. The one old plan, rebuilding the LLM agent on Claude (2026-10-08), was dropped and deleted on 2026-10-22: the team chose Adib's free local Ollama/Qwen agent instead (see [Decisions](decisions.md) and [LLM agent](llm-agent.md)). Two ideas from it may come back later: Oak answering out loud through `/tts`, and describing what its camera sees.

## Further out

These come from [Decisions](decisions.md) follow-ups, not from a team plan yet:

- A faster lidar decoder (about 2 Hz to about 62 Hz claimed), if a slow lidar hurts mapping.
- Retire the older `coco_detector` once YOLO publishes the standard detection message.
- Semantic maps (maps that know what things are) would need a GPU; Riley's experiments were left out for now.

<!-- sources: vision, $3,000 figure and milestone: Max via coordinator 2026-10-22 (README.md "Why we're building it"); team updates week of 2026-10-18; gh pr list (#11, #14) and gh issue list (no open issues, 2026-10-22); docs/TEST_LOG.md "Open issues" 1-4; docs/plans/pending/2026-10-08-claude-llm-agent.md; origin/adib/feat/llm-agent plan-file note; docs/FUSION.md "Follow-ups" 2-3 and "Left out" -->
