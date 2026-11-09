---
title: Experiments
parent: lab-setup
---

# Experiments journal

A lab notebook for every simulator run of the auto mapper. One entry per experiment, newest first.

Part of: [Lab setup](lab-setup.md). Related: [Simulator](simulator.md), [Test log](test-log.md) (real-Oak sessions).
World: placeholder_rooms, headless Gazebo, seed 1 (same start spots every time), stall rule: stuck if Oak moves < 0.25 m in 180 s.
Times are local (EST). Branches named max/automapper-sim* are local experiment branches; a step reaches master only through a PR after a 5-run check.

## What we've learned (keep current)
- Luck is big. The same start spot scored 48.2 % in one run and 99.0 % (finished) in another. Never trust 2 runs; confirm with 5+.
- The #1 killer is the latch: after backing off a wall, the explorer says "no safe direction: rear and both sides lack clearance" and holds still. It only re-checks on a new scan, and a still robot sees the same scan, so it never clears. That ended 8 of 10 baseline runs.
- Giving up a goal the instant a wall is within 0.3 m (blocked_timeout 0.0) feeds the latch. Waiting 2 s raised the 2-run average from 65.8 % to 91.5 % (step 1), but both runs still ended in the latch.
- Releasing the latch does not help: Oak is wedged with its nose 0.05-0.10 m from a wall, so it re-latches at once (step 2, 16 releases per run, 0 escapes).
- The latch is real, not a sensor quirk: Oak ends up crosswise in tight spots with 0.1-0.3 m front and back, and the only escape tried is one fixed 0.3 m move that cannot fit (step 4).
- Padding Nav2's footprint 0.10 m does not stop the latch, and makes the last frontiers unreachable (step 3).
- SLAM can freeze right after the startup scan; the explorer then waits forever (step 3, first run 1). A recovery case for #29.
- Runs can map 99 % and still not "finish": the latch stops Oak before the explorer declares the room complete.
- Collisions have been 0 in every run so far. The sim has no people and no narrow doors, so that grade is untested here.
- Noah's scoreboard (#47) runs in the sim and agrees with the old scorer after
  the 2026-11-06 measurement-boundary fix below. A trial now stops the explorer
  before either scorer finalizes. Physical Gazebo travel and reported `/odom`
  are kept separately so parity compares the same public signal.
- Making every Nav2 goal finish facing its direction of travel was denied in a
  matched five-run test. Mean completeness fell from 77.4% to 62.9%, and goal
  results fell from 22 reached / 11 failed to 8 reached / 47 failed. A plausible
  travel direction is not the same as an unconstrained final yaw; the added yaw
  requirement made many otherwise useful goals harder for Nav2 to satisfy.

## 2026-11-08 11:16 - Face each frontier goal in the direction of travel
- Branch: local `noah/exp/frontier-heading`, based on scoreboard commit
  `843ad4a`. The two source edits remain uncommitted. Compared against the same
  five seeds from the 2026-11-06 ten-run baseline.
- Hypothesis: the old identity quaternion makes every Nav2 goal finish at map
  yaw zero. Facing from Oak's current position toward the selected goal should
  avoid an unnecessary final turn and leave Oak better positioned to scan.
- Change: calculate the goal quaternion from the map-frame vector from Oak to
  the selected frontier. Frontier scoring, goal positions, Nav2 parameters,
  recovery behavior and the VLM were unchanged.
- Matched results (baseline -> travel-heading completeness):
  - Seed 28000: 98.5% -> 95.6% (-2.9 points).
  - Seed 28001: 99.4% -> 71.7% (-27.7 points).
  - Seed 28002: 46.8% -> 50.2% (+3.4 points).
  - Seed 28003: 98.6% -> 47.4% (-51.2 points).
  - Seed 28004: 43.7% -> 49.6% (+5.9 points).
- Aggregate: mean completeness 77.4% -> 62.9%; stuck runs 2/5 -> 3/5;
  formal completions stayed 0/5; collisions stayed 0. Baseline reached 22 of
  35 sent goals and failed 11; the change reached only 8 of 55 and failed 47.
  Logs repeatedly report DWB `No valid trajectories` and failed Nav2 goals.
- Scoreboard evidence: all five run folders finalized and completeness and
  collision parity held. Three distance checks passed. Two reported distance
  parity failures because `/odom` contained a discontinuity larger than 1 m;
  both run reports preserved that warning instead of silently counting it.
- Verdict: denied. Do not commit, push, merge or test this heading change on
  Oak. The final goal orientation is an additional planner constraint, not a
  harmless hint, and this implementation materially reduced reliability.
- Next: keep the unchanged baseline automapper for the first supervised Oak
  scoreboard run. Preserve these artifacts as negative evidence. Do not start
  another orientation experiment until the real baseline is recorded; a later
  design should explicitly evaluate position-only goal completion or a
  frontier-facing yaw rather than assuming travel direction is better.

## 2026-11-07 17:25 - A/B test: wait 2 seconds at an obstacle
- Branch: `noah/feat/scoreboard` (`52d1536`) with no automapper source change. The trial runner temporarily set `blocked_timeout:=2.0` and labeled the result `blocked_grace_2s`.
- Hypothesis: giving Nav2 two seconds to steer before cancelling a blocked goal should reduce entries into the no-safe-direction latch.
- Comparison: two identical seeds and start poses, 300 s run timeout and 180 s no-progress timeout.
  - Default: 54.9 % and 99.1 %, 77.0 % mean, 36.1 m total, 0/2 finished, 0 collisions. The first run latched; the second timed out near complete.
  - Two-second grace: 51.7 % and 99.6 %, 75.7 % mean, 31.6 m total, 0/2 finished, 0 collisions. The first run still latched; the second timed out near complete after repeated failed goals.
- Verdict: denied at the smoke-test level. It did not improve mean coverage, formal completion or the latch, so keep the default and do not open an automapper PR for this setting.
- Measurement finding: both default runs exposed a start-boundary mismatch. The independent scorer counted `/odom` from process creation while the scoreboard began at `exploration_started`; startup-scan drift caused distance parity failures. The runner now waits for the same structured start event and resets physical distance, odometry and collisions there. This measurement correction still needs one short simulator confirmation.

## 2026-11-07 16:27 - Step 5: try shorter checked recovery moves
- Branch: `noah/feat/recovery-steps` (`ef1fac3`), based on master after step 4. Compared against: the step-4 diagnosis; this was a two-run smoke test, not a statistical comparison.
- Hypothesis: when the normal 0.30 m recovery does not fit, a 0.15 m or 0.08 m move can safely get Oak out of the doorway latch.
- Change: try 0.30, 0.15 and 0.08 m in that order. Every candidate keeps the same full-body costmap, live-scan, stale-sensor and drift checks.
- Runs:
  - Run 1: 50.2 %, not finished, 0 collisions, 230 s, 4.6 m. All three distances were unsafe and the latch remained for 180 s.
  - Run 2: 34.5 %, not finished, 0 collisions, 218 s, 1.3 m. The first latch briefly cleared when the forward scan changed, but its escape goal failed; the next stop reported stale or missing scan, costmap or transform data and then no safe direction.
- Verdict: denied. The fallback executed and checked all three distances, but neither run found a safe shorter translation. Do not merge or test this version on Oak.
- Next: use the simulator runner's temporary `--explorer-param` support to compare the current default with `blocked_timeout:=2.0` on identical seeds. This tests the already documented step-1 result without changing the automapper or competing with its owner.

## 2026-11-06 - Ten-run scoreboard baseline after #41
- Branch: `noah/feat/scoreboard` (#47), master with #41's recovery fixes.
- Hypothesis: a larger batch should prove the score pipeline is repeatable and
  reveal whether the remaining stops are isolated or systematic.
- Runs: 10 starts, 33.7% to 99.4% completeness, 64.3% mean and 49.8% median.
  Three reached at least 98.5%, but none emitted `exploration_completed`.
  Five hit the 300 s timeout and five stopped after making less than 0.25 m of
  progress for 180 s. All 10 recorded zero collisions.
- Artifacts: every run wrote its JSON, CSV, Markdown report, map and explorer
  log. The weak runs repeatedly logged `Still stopped: no safe direction on
  the latest scan`; the high-coverage runs could reach the same latch near the
  end.
- Measurement finding: completeness and collision counts agreed, but five
  distance parity checks compared Gazebo's hidden ground-truth travel with the
  scoreboard's noisy `/odom`. Those numbers answer different questions.
- Change after review: keep physical `distance_m`, add `odom_distance_m` plus
  signed odometry error, and compare the scoreboard only with the independent
  `/odom` accumulator. This changes scoring output, not exploration behavior.
- Verdict: the scoreboard produced complete evidence, while the automapper is
  capable but not reliable. Re-run a short validation for the corrected
  distance columns, then compare any recovery change against these same seeds.

## 2026-11-06 15:56 - Scoreboard measurement-boundary validation
- Branch: `noah/feat/scoreboard` (#47), updated onto master `7572866`.
- Hypothesis: the three-run batch's parity failures came from the two scorers
  measuring different endpoints and from the legacy distance counter including
  simulator reset jumps, not from the scoreboard changing exploration.
- Evidence before the fix: a three-run batch mapped 92.0 %, 85.2 % and 16.6 %
  according to the legacy scorer (0 collisions). The scoreboard agreed on the
  first two maps but rejected large odometry jumps; on run 3 it received map
  growth while the explorer was still active and reported 35.1 %.
- Change: stop the explorer at the trial boundary before finalizing either
  scorer; reject truth-odometry steps over 1 m; preserve unknown cells when
  saving simulator maps (`free_thresh: 0.196`). No exploration behavior changed.
- Validation run: seed 27 mapped 48.6 %, stopped after the 60 s no-progress
  rule, and had 0 collisions. Legacy measured 3.3 m and 48.6 %; scoreboard
  measured 3.119 m and 48.638 %. Parity passed for distance, completeness and
  collisions.
- Verdict: confirmed on the deterministic validation run. Re-run a multi-run
  batch after #47 review for broader parity evidence; this result does not fix
  the automapper's variable coverage or latch behavior.

## 2026-11-06 12:40 - Step 4: measure why every escape is refused
- Branch: max/automapper-sim-v2-diag (local) = v2 + logging only. Compared against: nothing; this measures, it changes no behavior.
- Hypothesis: at the latch, backing up is refused by the scan-coverage rule (a lidar ray with no reading toward the move counts as unsafe), not by walls on the map.
- Change: none to behavior. At each latch, log which check (map clearance or scan coverage) rejects the rear, left and right moves, and what the scan reads behind and ahead.
- Runs:
  - Run 1: 47.2 %, not finished, 0 collisions, 222 s, 4.0 m. One latch at the same spot as earlier runs (ended 5.2, 2.1).
  - Run 2: 99.2 %, not finished, 0 collisions, 900 s (timeout), 31.8 m. One latch, then the last frontiers were never reached.
  - At every latch, all 3 moves (rear, left, right) fail BOTH checks: map clearance and scan coverage. The scan rays are real readings, not empty ones. Run 2: nearest reading ahead 0.66 m from the robot's center (nose 0.36 m), so 0.30 m of floor in front; behind 0.71-0.74 m (tail 0.45 m), so about 0.27 m behind. Run 1 is tighter behind (0.56 m, about 0.11 m).
- Verdict: hypothesis denied. The empty-ray rule is not the cause. Oak is genuinely boxed: it ends up crosswise in a narrow passage, with about 0.3 m in front and 0.1-0.3 m behind. The escape only tries one fixed 0.3 m move (plus 0.05 m margin), which cannot fit. A shorter move would fit in run 2.
- Next (step 5, off v2): when the 0.3 m escape does not fit, try shorter ones (0.15 m, then 0.08 m) before latching.

## 2026-11-06 12:10 - Step 3: pad Oak's footprint 0.10 m in Nav2
- Branch: max/automapper-sim-v4 (local) = v2 + one change. Compared against: v2 (step 1).
- Hypothesis: Nav2 drives Oak's nose to 0.05-0.10 m of walls (its controller barely weighs obstacle distance and the footprint has no padding). There the auto mapper finds no safe way out and latches. Keeping Oak 0.10 m off walls should stop it getting wedged.
- Change: footprint_padding 0.10 in both Nav2 costmaps (nav2_params.yaml).
- Runs:
  - Run 1 (first try): 4.9 %, 0 m moved. Voided: SLAM's map stopped updating right after the startup scan ("timed out waiting for fresh slam_map"), unrelated to Nav2 padding. Rerun below.
  - Run 1 (rerun): 49.1 %, not finished, 0 collisions, 213 s, 4.0 m. Quit: gave up a goal, then the same "no safe direction" latch.
  - Run 2: 99.2 %, not finished, 0 collisions, 900 s (timeout), 27.4 m. No latch at all, but the last frontiers became "no usable goal" (the padded body could not reach them), so it never declared the room complete.
- Verdict: denied. Mean 74.2 % vs 91.5 % for v2; the latch still happens, and padding adds a new failure (unreachable last frontiers). Dropped; v2 stays the base.
- Next: stop guessing; measure which check refuses every escape at the latch (step 4).

## 2026-11-06 11:50 - Step 2: release the latch after 20 s
- Branch: max/automapper-sim-v3 (local) = v2 + one change. Compared against: v2 (step 1).
- Hypothesis: if a latched stop with no safe direction hands Oak back to Nav2 after 20 s, Nav2 (which checks collisions itself) will find a way out, and runs will keep exploring instead of stalling.
- Change: new parameter latch_release_after = 20 s; after that long in a "no safe direction" hold, the latch resumes exploration. Sensor-stale latches are untouched.
- Runs:
  - Run 1: 98.6 %, not finished, 0 collisions, 456 s, 15.1 m. Released 16 times, never escaped: each release saw a wall 0.05-0.10 m ahead, backed away and latched again.
  - Run 2: 95.4 %, not finished, 0 collisions, 452 s, 12.2 m. Same loop, 16 releases.
- Verdict: denied. The average rose (97.0 % vs 91.5 %), but the mechanism never freed Oak once, so the gain is luck or extra time. Dropped; v2 stays the base.
- Next: why Oak is wedged 0.05-0.10 m from walls in the first place (step 3).

## 2026-11-06 11:40 - Step 1: wait 2 s before giving up a goal
- Branch: max/automapper-sim-v2 (local) = base + one change. Compared against: step 0 (same 2 starts).
- Hypothesis: waiting 2 s at a wall should stop the instant give-up, so Nav2 can squeeze past door jambs and fewer runs reach the latch.
- Change: blocked_timeout 0.0 -> 2.0 in explorer.py.
- Runs:
  - Run 1: 99.4 %, not finished, 0 collisions, 330 s, 19.0 m. Quit: a scan spin was aborted, then "no safe direction" latch after backing away from a wall 0.25 m ahead.
  - Run 2: 83.5 %, not finished, 0 collisions, 257 s, 10.1 m. Quit: one 2 s wait then give-up, open space unreachable, then "no safe direction" latch.
- Verdict: confirmed for completion (mean 91.5 % vs 65.8 %; run 2 went 32.7 % -> 83.5 %), unclear overall: finishes 0/2 vs 1/2, and 2 runs is luck-prone. Kept as the new base; needs the 5-run check before any PR.
- Next: the latch itself (step 2).

## 2026-11-06 11:32 - Step 0: base = master + Noah's scoreboard (#47)
- Branch: max/automapper-sim (local: master 714de91 + #47 + frontier markers for watching). Compared against: the baseline batch.
- Hypothesis: #47 changes nothing about how Oak explores; it only measures.
- Change: none to exploring.
- Runs:
  - Run 1: 99.0 %, FINISHED ("exploration complete"), 0 collisions, 143 s, 17.1 m.
  - Run 2: 32.7 %, not finished, 0 collisions, 211 s, 1.8 m. Quit: a scan spin aborted (Nav2 status 6), then the "no safe direction" latch at 5.3, 6.5.
- Verdict: confirmed (no harm seen; scoreboard parity pass on both runs). Run 1's start scored 48.2 % in the baseline: luck.
- Next: step 1, the first change.

## 2026-11-06 10:41 - Baseline batch: today's master, 10 runs
- Branch: master 714de91 (after #41's recovery fixes). Compared against: the 2026-10-28 batch (1/10 finished, mean 78.3 %, 0 collisions, 135 m).
- Hypothesis: #41's recovery fixes should beat the 10-28 numbers.
- Change: none (the yardstick).
- Runs: mean 71.0 % (34.9 to 99.4 %), 0/10 finished, 0 collisions, 124.7 m, mean 283 s.
  - 48.2, 34.9, 99.4, 97.7, 51.7, 59.4, 61.0, 99.4, 76.4, 82.0 %.
  - Why they quit: 8 of 10 latched "no safe direction" after an obstacle-ahead give-up; 2 of 10 (runs 5 and 10) had Nav2 abort 3 goals in a row and could not reach open space.
- Verdict: denied. Not better than 10-28 (0/10 vs 1/10 finished, 71.0 % vs 78.3 %); the spread is wide enough that it is not clearly worse either.
- Next: stop the instant give-up (step 1).
