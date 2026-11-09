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

## The project brief

Oak is our UConn Senior Design project, **"Autonomous Active Exploration and Mapping Using a Robot Dog"**, run by Dr. Jonathan Shihao Ji. Oak must explore an unfamiliar indoor space in the ITE building on its own and build an accurate floor map, with nobody driving it.

He grades six things: **map completeness, mapping accuracy, exploration time, navigation reliability (success rate), obstacle avoidance** (including people walking by), and **recovery** from failed attempts. He also expects a simulator setup, a try at LLM, VLM or VLA approaches to exploration, live demos on Oak, a benchmark table and a technical report. Demo day is the last week of the spring semester (May).

## The scorecard

Every lab run should fill one row of this table. The measurement backend is the
[Scoreboard](scoreboard.md); team tracking remains in
[#24](https://github.com/maxhawksby/go2-fusion/issues/24) and implementation in
[#27](https://github.com/maxhawksby/go2-fusion/issues/27).

| Grade | How we measure it | Pass mark |
| --- | --- | --- |
| Completeness | Mapped floor vs a tape-measured plan of the room | at least 95% |
| Accuracy | Wall lengths and door positions vs tape measure | median error 5 cm or less |
| Exploration time | Start to "complete", vs driving the same room by hand | faster than by hand |
| Reliability | Runs finished with no human touch | at least 9 of 10 |
| Obstacle avoidance | Collisions; a person steps in front 3 times | 0 collisions; stops, then resumes |
| Recovery | Planned failures: blocked door, lidar cut, Wi-Fi drop, SLAM restart | recovers from at least 4 of 5 |
| Laptop load | CPU and RAM during a run | 70% CPU or less, no WSL crash |

## Phases

A phase ends only when its gate passes on Oak.

0. **Measure** (Oct 26 - Nov 8): fix the TF links, merge the sensor watchdog, build the run logger, 3 recorded runs. *Gate:* the first real scorecard, even if every number is bad.
1. **Reliable** (Nov 9 - Dec 6): replay recorded runs, a simulator of the lab room, save and resume the map, failure drills. *Gate:* 8 of 10 runs with no hands.
2. **Fast and accurate** (Dec 7 - Feb 7): a planner made for a dog that turns in place, face the frontier on arrival, drift and turn tests, a lighter launch for laptops. *Gate:* every pass mark met in the lab room.
3. **Smart exploration** (Feb 8 - Mar 21): a vision-language model ranks frontiers from the camera, YOLO labels the map; A/B test against plain frontiers. *Gate:* a measured answer to "does AI help?".
4. **Prove it at ITE** (Mar 22 - demo day): 2-3 tape-measured ITE spaces, 10 scored runs each, benchmark table, report, two rehearsals.

## Who owns what

Claim your issue by assigning yourself.

- **Luis - exploration brain:** the [Auto mapper](auto-mapper.md), frontier choice, [Nav2](nav2.md) tuning. [#25](https://github.com/maxhawksby/go2-fusion/issues/25)
- **Adib - localization:** the [TF tree](tf.md), odometry timing, [SLAM](slam.md) settings, drift and turns; later leads Phase 3. [#26](https://github.com/maxhawksby/go2-fusion/issues/26)
- **Noah - scoreboard:** run logger, recordings, replay; the Oak Control dashboard as mission control. [#27](https://github.com/maxhawksby/go2-fusion/issues/27)
- **Jeth - simulator and answer key:** Gazebo world of the lab room, tape-measured floor plans. [#28](https://github.com/maxhawksby/go2-fusion/issues/28)
- **Max - recovery, plan, report:** sensor watchdog, save and resume the map, failure drills, the technical report. [#29](https://github.com/maxhawksby/go2-fusion/issues/29)

**Parked until Phase 2 passes:** speech output, driving from the dashboard, voice commands, the general LLM agent, wiki polish. Each comes back when it serves a scorecard row.

**Weekly rhythm:** one lab session (record every run), then a 20-minute scorecard review; each person picks the worst number they own. A change merges only if a recording or replay shows it did not make a number worse.

## Plans on file

No team plans are on file. The one old plan, rebuilding the LLM agent on Claude (2026-10-08), was dropped and deleted on 2026-10-22: the team chose Adib's free local Ollama/Qwen agent instead (see [Decisions](decisions.md) and [LLM agent](llm-agent.md)). Two ideas from it may come back later: Oak answering out loud through `/tts`, and describing what its camera sees.

## Further out

These come from [Decisions](decisions.md) follow-ups, not from a team plan yet:

- A faster lidar decoder (about 2 Hz to about 62 Hz claimed), if a slow lidar hurts mapping.
- Retire the older `coco_detector` once YOLO publishes the standard detection message.
- Semantic maps (maps that know what things are) would need a GPU; Riley's experiments were left out for now.

<!-- sources: project brief, grades, deliverables, demo month: Dr. Ji's Senior Design brief via Max 2026-10-25; scorecard, phases, lanes: Max's decisions 2026-10-25 (issues #24-#29); vision, $3,000 figure and milestone: Max via coordinator 2026-10-22 (README.md "Why we're building it"); team updates week of 2026-10-18; gh pr list (#11, #14) and gh issue list (no open issues, 2026-10-22); docs/TEST_LOG.md "Open issues" 1-4; docs/plans/pending/2026-10-08-claude-llm-agent.md; origin/adib/feat/llm-agent plan-file note; docs/FUSION.md "Follow-ups" 2-3 and "Left out" -->
