# Scoreboard distance undercount

## Recorded evidence

The completed hallway seed 28003 on callback candidate e432bdf reported
55.4162 m from independent odometry and 3.7816 m from the live scoreboard.
The saved bag contains 3519 odometry poses inside its recording window;
their summed travel is 55.4161865746 m. No consecutive recorded step exceeds
the scoreboard's 1 m discontinuity limit. Median arrival gap is 0.08095 s,
maximum 0.39297 s. The live scorecard nevertheless warns about a discarded
step over 1 m, and only 54 status messages appear over about 313 s of bag
wall time despite a nominal 1 s timer. Coverage agrees at 76.1133%.

## Measured bottleneck and narrow repair

Offline profiling in Humble Docker, capped at one CPU while the user's
separate simulation continued, measured the old score calculation at 6.8859 s
for the saved 1014 by 1006 answer-key grid. Resource sampling took 0.001 s;
checkpoint writes took 0.0508 s on container storage and 0.4474 s on the host
mount. Timings are workload-dependent samples, not controlled latency bounds.

Every map callback repeated a Python flood-fill of an unchanged answer key
in the single-threaded recorder. Preparing that mask, wall exclusion and
start-pose alignment once moves this cost before subscriptions/readiness.
The new preparation took 6.5267 s; scoring the saved SLAM map then averaged
0.005216 s across 20 calls. Coverage was identical at 76.11327575818437%.
Each map still supplies its own resolution, dimensions, origin and occupancy.
Live recording and replay retain the prepared scorer. One-shot callers keep
the original convenience function and scoring definition.

This is a demonstrated callback bottleneck consistent with sparse processed
odometry and false jump rejection. Runtime distance recovery is not yet
proven. File writes remain synchronous, so this does not promise immunity
to slow storage or other callback load. The odometry filter is unchanged;
raising its limit would hide symptoms rather than address callback work.

## Validation and next step

Regression tests check reuse without another flood-fill, changing SLAM map
origins/sizes/resolutions, rotated and translated alignment, empty truth,
and interleaved live map/odometry callbacks. The existing scoreboard tests
also cover bag replay and artifact output. No nodes or motion are created
by the new callback test. Original result folders are never rewritten.

After the current mapping comparison finishes, prepare a separate candidate
with this scoring optimization and ask the user to run one recorded start.
Compare live distance with independent trial odometry and bag reconstruction,
retaining coverage equivalence and unchanged collision counts. Keep this
change separate from navigation-goal ownership until that validation.


## First user-run runtime validation

The user completed seed 28003 on clean combined candidate 0e4cea6, keeping
Jeth recovery, the callback repair, hallway world, start pose and 300 s limit.
Session: 20261110-215445-496-scoreboard_distance_seed28003-batch;
trial directory: 20261111-025719.

Independent trial odometry was 39.4726 m; live scoreboard distance was
39.4634 m (difference 0.0092 m, about 0.023%). Trial coverage was 62.1%,
scoreboard coverage 62.0607%, and both recorded zero collisions. The trial's
combined distance/coverage/collision parity check passed. Scoreboard warnings
were empty. Timeline output contained 355 entries plus its header, versus
91 plus header in the earlier 300 s callback-only run. This supports improved
recorder responsiveness and distance accounting on this start.

The run still timed out without finishing mapping. Its 62.1% coverage was
lower than the previous callback-only run's 76.1%; execution variability and
reduced observer load mean this is not a controlled algorithm comparison.
No navigation behavior was changed by this measurement repair. This single
runtime case validates the targeted distance defect, not all workloads,
physical Oak distance, or general mapping reliability. Keep the original
runs intact and expand paired navigation testing separately.
