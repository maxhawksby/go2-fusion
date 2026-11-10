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
