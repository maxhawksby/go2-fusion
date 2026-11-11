# Delayed navigation result ownership

## Evidence

The weak Jeth hallway seed 28003 reached 30.8% coverage before the 300-second
limit. At ROS log time 1794349039.61 the explorer timed out a goal after 60 s.
It sent a new goal at 1794349041.11, received acceptance at 1794349041.17, and
logged a canceled result at 1794349041.24. Nav2 meanwhile continued computing
control and passing paths for that newer destination through 1794349043.22.
The explorer then issued another destination, and repeated goal preemptions
followed. The eventual log said no usable goal while many frontier regions
were still detected. Nav2 also reported unknown-region and oscillating
trajectories. This is not evidence that the corridor was fully mapped.

The old callbacks had no captured goal identity: every reply read and mutated
the explorer's current goal state. An offline fake-action reproduction showed
that an old canceled result clears a newer handle/active flag and records a
failure against the newer goal. The source defect is reproducible without
ROS nodes or motion. The recorded log ordering is consistent with this race;
the logs alone do not establish which underlying goal produced every result.

## Repair

Capture the sequence number when submitting a navigation goal and carry it
through the response and result callbacks. Ignore responses/results that no
longer own the active goal state. If a retired submission is accepted late,
use the existing orphan-action path to request cancellation and hold motion
until terminal confirmation. Preserve the shutdown wait over orphan handles.

Current goals still emit normal outcomes, and their rejection/error handling
is retained. Obstacle thresholds, planner/controller parameters, frontier
ranking, time limits, VLM settings and recovery geometry are unchanged.
This change does not add confirmation of every normal timeout cancellation;
it prevents old callbacks from attributing their outcome to another goal.

## Validation and next experiment

Tests cover late success/cancel/abort, late acceptance and terminal orphan
cleanup, late response errors, current success, and both callback registration
steps capturing the original sequence. The full functional mapper/detector
suite is run in the existing Humble Docker environment. These tests validate
state ownership, not navigation success or physical safety.

For runtime evaluation, preserve the completed Jeth five-run source and make
a separate experiment with this repair. First repeat only weak seed 28003,
using the same hallway world, 300-second limit, 180-second stall/startup
limits and bag recording. Check that late callbacks are explicitly ignored,
goal failures attach to the right requests and the preemption cascade is
reduced. Unknown-space trajectory rejection may remain and needs separate
investigation. Do not combine new frontier heuristics or TF repairs into that
first comparison. No simulation or live robot run was launched while preparing
the repair.


## Completed user-run comparison

The user completed five matched starts on clean combined source e432bdf
(recovery plus this repair), the unchanged hallway world and a 300 s limit.
VLM/camera remained disabled. The original recovery-only source is preserved.

| Seed | Recovery only | Recovery + callback repair | Difference |
| --- | ---: | ---: | ---: |
| 28000 | 56.5% | 51.4% | -5.1 pp |
| 28001 | 45.8% | 56.4% | +10.6 pp |
| 28002 | 65.0% | 42.4% | -22.6 pp |
| 28003 | 30.8% | 76.1% | +45.3 pp |
| 28004 | 64.8% | 67.9% | +3.1 pp |
| Mean | 52.6% | 58.8% | +6.3 pp |

Both sets had zero collisions and zero formal completions. Three starts
improved, two scored lower. These are single executions per start, not
replicated estimates or evidence that the repair guarantees better coverage.
One scoreboard startup timeout on the first attempt at seed 28002 is retained
as an infrastructure failure, excluded from mapping means and replaced with
one user-run retry. Valid candidate manifests all record e432bdf and no dirty
source changes. All five valid candidate runs ended at the 300 s limit.

The candidate logged four ignored retired results: one on seed 28000, two on
28002, one on 28004. Seed 28002 specifically timed out goal 4, accepted a new
goal, ignored goal 4's delayed result and later reached the newer destination.
This supplies runtime evidence that ownership filtering is exercised. Across
five runs there were 27 exploration-goal successes, three open-space successes,
seven navigation-ended-with-status messages and no no-usable-goal messages.
Removing false status attribution does not remove slow or unreachable goals:
seed 28002 still spent two long intervals waiting for Nav2 results.

Coverage was taken consistently from trial CSV. Live scoreboard distance
still undercounts; do not infer efficiency from it. A separate scoring-cost
optimization is draft PR #64 and was not included in these runs. Follow up
with that measurement validation before new frontier/controller changes.

Raw candidate sessions in the experiment's go2_sim/results/ite_hallways:

- 20261110-193928-352-goal_callback_seeds28000_28002-batch: first two rows only
- 20261110-205533-234-goal_callback_seed28002_retry-batch: replacement seed 28002
- 20261110-192312-727-goal_callback_seed28003-batch
- 20261110-200617-677-goal_callback_seed28004-batch
