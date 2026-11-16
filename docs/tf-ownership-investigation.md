# Navigation transform ownership investigation

November 10, 2026. Related to issue #26 and the hallway diagnostic under #59.

## Confirmed defect

`go2.urdf`, `go2_with_realsense.urdf` and `multi_go2.urdf` contained fixed
identity joints `map` -> `odom` and `odom` -> `base_link`. The default launch
passes these models to robot_state_publisher, which publishes fixed joints on
`/tf_static`. SLAM and the driver/simulator independently publish the same
edges on `/tf`. Adib's remote localization branch currently has no unique
commits relative to master; no unpublished localization implementation was
assumed or overwritten.

The recorded seed-28000 hallway diagnostic confirmed both identities. Its live
`odom` -> `base_link` ended at about (0.270, -0.224, 0.32) m, while the static
version remained at the origin. The live `map` -> `odom` also differed from
identity, including a -0.32 m vertical correction. This is conflicting
ownership, even though both topics use the same parent/child names.

## What offline inspection did and did not establish

The source recorded for this diagnostic was the pre-VLM hallway worktree.
The run reached one short goal, scanned, then failed to make progress on a
second goal before the 60-second timeout. Completeness was 3.2264%, scoreboard
distance 0.3512 m and collisions zero. It was not a finished mapping mission.

Replaying only the bag's transforms into two in-memory Humble TF buffers,
one original and one excluding the two static navigation edges, produced
identical latest `odom` -> `base_link` positions at all 332 scan checkpoints
where both buffers could answer. Before dynamic odometry arrived, nine
filtered-buffer lookups were unavailable while the original supplied identity.
No ROS messages were published and SLAM was not rerun. This confirms the
startup fallback and ownership problem, not a causal explanation of the stall.

At the initial spin failure, the latest published local costmap was about
0.073 s old. Sampling the rectangular footprint at the current heading and
0.3 rad further found no unknown/lethal cells; a 90-degree sample contained
one unknown cell. These samples are not Nav2's exact swept-footprint check,
and the recorded occupancy grid is not its internal costmap. Unknown-space
handling during rotation remains an investigation target, not a proven fix.
Later progress failures had a clear sampled footprint. Scans continued around
4.73 Hz with a largest recorded gap around 0.40 s.

The aggregator Header exception is separate: the default navigation scan
reads `/point_cloud2` directly, not `/pointcloud/filtered`. No Jeth recovery
or Header changes were included in this investigation.

## Narrow repair and next validation

Remove only the two fixed navigation joints and their orphan `map`/`odom`
links in the three affected models. Keep physical joints, sensor placement,
Nav2 settings, frontier ranking, driver timing and recovery behavior unchanged.
The body tree must remain connected with `base_link` as its only root.

Four structural unit cases pass across the three models and two multi-robot
prefixes. An isolated Humble robot_state_publisher check also passed all four:
18 static physical transforms for Go2 and each multi-robot prefix, 20 for the
RealSense variant, and no navigation edges. The radar frame remained present.
The check ran on localhost-only domain 141 with no driver or simulator.
This is model validation, not a navigation simulation or a live robot test.

For a navigation comparison, first run the updated hallway baseline and then
the same commit plus this model repair, using the same world/seed/limits and
bags. Inspect duplicate edges, startup TF availability, scans, map growth,
goal outcomes and collisions. The old hallway recordings stay archived; a
current-master comparison includes the newly merged #55 and must not be
presented as a one-change comparison with the old recordings.

Physical Oak validation remains pending. No claim of improved coverage,
recovery or readiness follows from removing the fixed joints alone.
