---
extends:
when: Improving the auto mapper in rounds, where each round starts from what Oak or the simulator was seen doing.
---

# Oak revision loop

One round is observe, report, file, research, fix, retest. Repeat until the done line of issue #58 holds.

1. **Preflight.** Run `docker info`. Check the robot answers on port 9991 at `ROBOT_IP`. Ask the human to confirm a spotter, the physical controller and a free WebRTC connection. If any check fails, stop and say which one. Never move Oak without that confirmation in the current chat.
2. **Observe.** Run [the Oak lab test](../../docs/wiki/scoreboard-oak-lab-test.md) on `master` with the scoreboard recording. When Oak is not available, run the same mission in [the simulator](../../docs/wiki/simulator.md).
3. **Report what was seen.** Read `run.json`, `report.md` and `timeline.csv` from the newest results folder. State only what they show: end reason, distance, known area, goals, touches, collisions, warnings. Add the run to the [test log](../../docs/wiki/test-log.md).
4. **File.** Open one GitHub issue per defect, with its `wiki:<slug>` label and the path to the evidence. Search open issues first and comment on a match instead of opening a duplicate.
5. **Research before design.** For each issue, look for work that already solves it before writing new code: a Nav2 plugin or behavior, a maintained ROS 2 Humble package (frontier exploration, SLAM, recovery), or published work on the Unitree Go2. Write in the issue what you found, with links, and whether to adopt it, adapt it or reject it and why. Write new code only when nothing fits. This is the "Extend before adding" rule in [AGENTS.md](../../AGENTS.md).
6. **Fix.** One issue per branch, through the pstack playbook that matches it (Bug fix or Feature), then Opening a PR.
7. **Retest.** Rerun the mission in the simulator after every fix and compare the scoreboard numbers before and after. Add the issue to the Oak retest list. Run Oak retests only when the human says the lab is ready, starting again from step 1.
8. **Next round.** Pick the worst open defect and go to step 5.

## Reporting to the human

Lead with the one thing that matters most: a safety problem, a blocker, or the result. Five lines at most. Counts and links, not logs. Detail goes in the issue, not the chat.
