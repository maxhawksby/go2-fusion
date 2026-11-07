---
title: Scoreboard
parent: maps-nav
code: [src/go2_scoreboard/**]
verified: cdafdbe2e8c5
---

# Scoreboard

The scoreboard records an autonomous mapping run and turns it into evidence:
a bag for replay, a final map, a CSV row, a timeline and a readable report. It
only observes Oak. It never publishes a drive or WebRTC command.

Part of: [Maps and navigation](maps-nav.md)

## What it measures

- Time, odometry distance and known map area.
- Goal outcomes and recovery or stale-sensor events from the [Auto mapper](auto-mapper.md).
- Collisions from the simulator bumper or a manual marker on Oak.
- Human touches marked by the operator.
- Container and ROS-process CPU and memory use.
- Map completeness when a ground-truth map and start pose are supplied.

Accuracy stays `null` until the team has a measured-floor-plan scorer. A run
without an answer key also stores completeness as `null`; the scoreboard never
invents a percentage.

## Interfaces

The recorder listens to `/map`, `/odom`, `/sim/bumper` when available, and the
reliable `/auto_mapper/events` stream. It publishes a transient-local
`/scoreboard/status` once per second. The non-motion services are:

- `/scoreboard/mark_human_touch`
- `/scoreboard/mark_collision`
- `/scoreboard/finish`
- `/scoreboard/ready` (appears only after optional rosbag startup is ready)

Status contains the run state, elapsed time, distance, map area, optional
completeness, goal counts, safety/recovery counts, CPU/RAM, warnings and the end
reason. The run moves through `WAITING`, `RECORDING`, `FINALIZING`, then
`COMPLETE` or `ERROR`.

## Output

By default, each host-visible run is under
`src/go2_scoreboard/results/<timestamp>_<label>/` and the folder is ignored by
Git. It contains `run.json`, `scorecard.csv`, `report.md`, `timeline.csv`,
`resources.csv`, `final_map.yaml`, `final_map.pgm` and `bag/`. The parent
`index.csv` has one row per run. Files are replaced atomically, so an interrupted
run keeps its last complete checkpoint.

## Run it

Start the recorder before the auto mapper:

```shell
ros2 run go2_scoreboard recorder --ros-args \
  -p source:=oak -p run_label:=lab-baseline
```

Wait for `Scoreboard waiting for exploration` before starting the auto mapper.
The simulator trial runner waits for `/scoreboard/ready` automatically. This
prevents rosbag process startup from missing the first event or odometry sample.

For a run without an answer key, that is enough. To score completeness, also
pass `ground_truth_map`, `start_x`, `start_y` and `start_yaw`. The simulator's
trial runner does that automatically and keeps its older scorer as an
independent parity check.

Experiments can pass a JSON object in `configuration_json`; it is stored with
the code fingerprint and run label so later tables can separate algorithm
modes, seeds and parameter sets. The simulator runner records its `--variant`,
simulator launch arguments and temporary `--explorer-param name:=value`
overrides in that object.

Replay does not launch Oak or Nav2:

```shell
ros2 run go2_scoreboard replay /path/to/bag --out /path/to/replays
```

## How it breaks

- Missing optional topics create warnings and nullable fields instead of ending the run.
- An odometry jump larger than the configured limit is treated as a reset, not distance.
- If `ros2 bag record` exits, measurements continue and the report records a warning.
- Ctrl+C finalizes a partial report. A hard container or power failure leaves the last atomic checkpoint.
- Reusing one Docker build volume across unrelated branches can leave stale build metadata. Use a separate Compose project or build volume for clean comparison runs.

## Who's on it and what's next

**Noah** owns the scoreboard ([issue #27](https://github.com/maxhawksby/go2-fusion/issues/27)).
Unit tests, synthetic-bag replay and two fixed-seed simulator integrations pass.
The first seed-27 run on 2026-11-05 reproduced the old costmap stall and proved
that the two scorers agreed at 0.0 m, 0 collisions and 5.7% completeness. After
the recovery fix from pull request #41 was merged, the same seed traveled 2.8 m
and reached 48.0% completeness with 0 collisions before the trial runner stopped
it for making less than 0.25 m of progress for 60 seconds. The scoreboard's
independent values were 2.7235 m, 47.959% and 0 collisions; the differences are
the legacy report's rounding and slightly different final sampling time.

`COMPLETE` means the artifacts finalized successfully, not that exploration
completed the room. This run ended by a controlled finish request after the
stall. A bag-enabled seed-1 integration on 2026-11-06 also proved the startup
gate and replay path: the live score was 0.4544 m, 25.0432% and 0 collisions;
replay measured 0.4726 m, 25.0432% and 0 collisions with no missing-start
warning. The 1.8 cm distance difference is final ROS callback ordering and is
inside the simulator parity tolerance; repeated replay itself is deterministic.
The next steps are to diagnose the remaining automapper stall and complete one
supervised Oak recording. A later 10-run batch reached 33.7% to 99.4%
completeness (64.3% mean), with 0 collisions but 0 formal completions: five
runs stalled and five timed out. It also showed that simulator ground-truth
travel and Oak's noisy reported `/odom` are different measurements. The trial
runner now preserves both and validates the scoreboard against an independent
`/odom` accumulator. No Oak scorecard has been claimed yet.

<!-- sources: src/go2_scoreboard/**, src/go2_auto_mapper/go2_auto_mapper/explorer.py structured event publisher, src/go2_sim/go2_sim/trials.py -->
