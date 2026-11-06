# Oak experiments journal

A lab notebook for every simulator run of the auto mapper. One entry per experiment, newest first.
World: placeholder_rooms, headless Gazebo, seed 1 (same start spots every time), stall rule: stuck if Oak moves < 0.25 m in 180 s.
Times are local (EST). Branches named max/automapper-sim* are local experiment branches; a step reaches master only through a PR after a 5-run check.

## What we've learned (keep current)
- Luck is big. The same start spot scored 48.2 % in one run and 99.0 % (finished) in another. Never trust 2 runs; confirm with 5+.
- The #1 killer is the latch: after backing off a wall, the explorer says "no safe direction: rear and both sides lack clearance" and holds still. It only re-checks on a new scan, and a still robot sees the same scan, so it never clears. That ended 8 of 10 baseline runs.
- Giving up a goal the instant a wall is within 0.3 m (blocked_timeout 0.0) feeds the latch. Waiting 2 s raised the 2-run average from 65.8 % to 91.5 % (step 1), but both runs still ended in the latch.
- Releasing the latch does not help: Oak is wedged with its nose 0.05-0.10 m from a wall, so it re-latches at once (step 2, 16 releases per run, 0 escapes).
- Runs can map 99 % and still not "finish": the latch stops Oak before the explorer declares the room complete.
- Collisions have been 0 in every run so far. The sim has no people and no narrow doors, so that grade is untested here.
- Noah's scoreboard (#47) runs in the sim and agrees with the old scorer (parity pass on every run so far).

## 2026-11-06 12:10 - Step 3: pad Oak's footprint 0.10 m in Nav2
- Branch: max/automapper-sim-v4 (local) = v2 + one change. Compared against: v2 (step 1).
- Hypothesis: Nav2 drives Oak's nose to 0.05-0.10 m of walls (its controller barely weighs obstacle distance and the footprint has no padding). There the auto mapper finds no safe way out and latches. Keeping Oak 0.10 m off walls should stop it getting wedged.
- Change: footprint_padding 0.10 in both Nav2 costmaps (nav2_params.yaml).
- Runs: running.
- Verdict: pending.
- Next: pending.

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
