# Simulation settings behind Noah's reported scores

Recorded November 10, 2026. These are historical runs, not scores for today's
master. Percentages mean mapped floor completeness, not mission success rate.
See also the [experiments journal](wiki/experiments.md).

## November 8 comparison: placeholder rooms

The host script `run_luis_pr_offline_matrix.ps1` and saved CSV/console logs
confirm these settings. All three batches used `placeholder_rooms.sdf`, no
simulated people, headless Gazebo, three runs, seed 28 (individual seeds
28000, 28001 and 28002), a 300-second limit, a 180-second stall limit and a
120-second startup limit. No explorer parameter overrides were supplied.

| Individual seed | Master-labelled control | Older PR #54 snapshot | Older PR #55, VLM off |
| --- | ---: | ---: | ---: |
| 28000 | 89.3% | 57.0% | 74.1% |
| 28001 | 74.9% | 64.8% | 68.2% |
| 28002 | 95.5% | 46.1% | 96.1% |
| Mean | 86.6% | 56.0% | 79.5% |

Every batch had zero reported collisions and **0/3 formal completions**.
All stopped on timeout or lack of progress. PASS in the host script means the
process returned successfully; it does not mean the explorer finished the map.

The starts match across these three batches:

| Seed | World x (m) | World y (m) | Yaw (rad, rounded) |
| --- | ---: | ---: | ---: |
| 28000 | 3.525 | 5.625 | -1.548984 |
| 28001 | 6.325 | 6.675 | 0.516867 |
| 28002 | 3.525 | 5.725 | 2.171748 |

Source identity and limits:

- PR #54 ran snapshot `22266f8d3f6aecc465ffc0d2ab243ffae7b90878`.
- PR #55 ran snapshot `707569a23d54ffbccf32e1e73f9dc0564cf11050`, with
  explicit launch arguments `vlm_mode:=off camera_rate:=0`.
- These were pre-rebase snapshots, not the subsequently merged PR heads.
- The control's saved label is `same_session_master`. Its saved source
  fingerprint is `5fbbbe92eba44bd1`; `code_version` is `unknown` because the
  container did not see Git metadata. An exact historical control commit must
  not be inferred from the checkout's current HEAD. Its current clean source
  fingerprint differs, so current files are not proof of the historical source.
- The control includes the newer scoreboard instrumentation/scorer, while the
  older feature CSVs come from their older trial runners. This was a useful
  integration comparison, not a perfectly isolated experiment on one shared base.
- The lower older-PR #54 results are a regression signal to investigate, not
  proof that people tracking caused worse mapping. Start seeds do not make
  asynchronous Gazebo/Nav2/SLAM scheduling perfectly deterministic.

Commands captured for the old feature batches (run only from the intended
prepared source snapshot; these are documentation, not automatic jobs):

```shell
ros2 run go2_sim trials --world placeholder_rooms --runs 3 --seed 28 \
  --timeout 300 --stall-timeout 180 --startup-timeout 120 \
  --out /ros2_ws/src/go2_sim/results/pr54-people-seed28

ros2 run go2_sim trials --world placeholder_rooms --runs 3 --seed 28 \
  --timeout 300 --stall-timeout 180 --startup-timeout 120 \
  --out /ros2_ws/src/go2_sim/results/pr55-vlm-off-seed28 \
  -- vlm_mode:=off camera_rate:=0
```

PR #54 and #55 used separate Compose projects `go2pr54` and `go2pr55`, and
separate mounted worktrees/build volumes. The control ran in the scoreboard
worktree and wrote to `src/go2_scoreboard/results/same-session-master-control/`.
The saved control report specifies variant `same_session_master`, defaults
for launch arguments and no explorer overrides; bag recording was off.

The old simulator launch defaults inspected at the feature snapshots are lidar
5 Hz, range 12 m, 720 samples/ring and range noise 0.01 m. The simulated driver
defaults use odometry noise 0.01 m/sqrt(m), yaw noise 0.01 rad/sqrt(rad), zero
scale error/drift and command latency 0.1 s. The host commands supplied no
overrides for these. This documents declared settings, not a recovered live
parameter dump of each node.

## November 10: different hallway world, not a comparison with the above

The 3.19% smoke result used the new assumed 24 x 24 m `ite_hallways.sdf`, one
run, seed 28, a **60-second** limit, 180-second stall/startup limits and no
camera or VLM integration. The clean worktree commit recorded at startup was
`61897f113f53470ce43f3848dcc332c1d7b082d0` on `noah/feat/ite-hallway-world`.
It reused image `go2scoreboard-unitree_ros:latest` and Compose project `go2ite`.

The scoreboard recorded 3.1934%, 0.3191 m, zero collisions, one accepted goal
and no reached goals; simulator/scoreboard parity passed. The trial timed out.
This is a different world and duration, so it cannot be used as evidence that
Luis's algorithm is better or worse than the older placeholder-room runs.
The source for this world and the diagnostic tools is in draft PR #60, not
master. The diagnostic repeat enables bags and retains the same seed/world.

## Local work versus shared master

There is uncommitted collision-monitor work in Noah's main local checkout.
The historical feature comparisons ran from separate clean review worktrees;
the hallway run's startup manifest also records a clean separate worktree.
Do not equate unrelated local changes with hidden tuning of those runs.
The scoreboard UI displays archived runs from different revisions/settings;
its latest or highest row is not automatically a current-master benchmark.

## Comparing with Luis's five-run screenshot

The supplied screenshot reports means of 60.7% (master baseline), 63.8%
(changes, VLM/camera off), 65.8% (camera on, VLM off), and 78.2% (VLM active)
for starts 28000-28004. The active mean is 17.5 percentage points above that
baseline. This is promising batch-level coverage evidence, not verified
mission completion or proof of consistent improvement at every start.

TODO(noah): collect the exact commits, world, timeout, stall rule, start poses,
camera rate, VLM model/settings and raw results for each screenshot column.
Ask for completion, collision and failure counts alongside coverage. Then use
one pinned base, scorer and configuration for a fresh controlled comparison.
The screenshot values are supplied evidence; its raw logs have not been audited.
