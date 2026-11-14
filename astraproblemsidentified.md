I reviewed the executable code on **`master` at `f3067b4`**, including the helpers and Nav2 configuration. **Nothing was changed or committed, and no robot or simulation was started.** The findings below describe what the code does; they do not establish which problem caused a particular failed run.

**My main assessment:** frontier exploration is a reasonable foundation. The more concerning problems are disagreements between goal selection, navigation settings, recovery behavior and what counts as success.

**1. What each helper actually does**

| File | Very simple explanation |
|---|---|
| [frontier_scoring.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/frontier_scoring.py) | Finds the edges between known and unknown space. Estimates which destination might reveal more unknown space. Also helps judge whether a previous visit made progress. |
| [obstacle_sensing.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/obstacle_sensing.py) | Checks strips of Nav2’s obstacle grid: “Is there a known obstacle ahead?” and “Does this straight direction look clear?” It does not calculate a complete route. |
| [open_space.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/open_space.py) | Searches nearby map cells for somewhere roomier when Oak gets stuck. Its search follows connected free cells, but does not fully establish that Oak’s body can travel through them. |
| [recovery.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/recovery.py) | Evaluates particular short escape movements. Checks the space the body would occupy, scan coverage, permitted drift and measurement freshness. The explorer executes the selected movement. |
| [social_space.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/social_space.py) | Draws mathematical circles around reported people, predicted positions and groups. Checks whether candidate destinations or straight lines enter those circles. |
| [vlm_ranking.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/vlm_ranking.py) | Sends camera views to an optional image-understanding model and converts its answers into destination preferences. Model requests run on a background thread. |
| [nav_log.py](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/nav_log.py) | Writes a diary: destinations, outcomes, recovery events, optional pictures and changes in the number of known map cells. |

There are also three related files that are **not directly imported helpers of the explorer**:

- **`vlm_view.py`:** a separate program that displays model descriptions and publishes a suggested direction. Active VLM exploration can use that suggestion.
- **`socialnav.py`:** prepares image history and coordinate conversions for the SocialNav model.
- **`socialnav_view.py`:** requests and displays model-generated paths. It does **not** send movement commands.

These optional model tools are separate from the basic frontier algorithm.

**2. What “defaults that ROS parameters can override” means**

Think of a parameter as a **setting knob**.

For `wall_clearance`:

- The code supplies a default of **0.5 metres**.
- At startup, a configuration can request **0.6 metres** instead.
- That run uses the supplied value without editing the Python file.

That is a normal ROS configuration mechanism. [ROS parameter documentation](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Basic/About-Parameters.rst).

**There is a limitation in this implementation:** many parameters are copied into ordinary Python variables during startup. Changing the ROS parameter afterward does not automatically update those variables. I would make these settings explicitly startup-only or implement proper updates, so the reported setting and actual behavior cannot disagree.

**3. Why check every two seconds instead of waiting for completion?**

**It already waits for the current navigation goal.** At [this check](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/explorer.py:2453), an active goal causes the exploration loop to return without choosing another destination.

The two-second timer periodically asks:

- Has a required map or position finally arrived?
- Has a recovery action finished?
- Are fresh measurements available?
- Has an action taken too long without returning a result?
- If nothing is active, can we choose another goal?

Nav2’s result callbacks report completion separately. This combination—callbacks for results and a timer for supervision—is reasonable.

However, **the obstacle watcher does not guarantee that every problem will raise an alarm**. When its scan, costmap or required position is unavailable, one normal-navigation path simply returns without requesting a stop. Also, the explorer uses the default single-threaded ROS executor, so expensive work in another callback can delay its nominal quarter-second checks. [Humble executor implementation](https://raw.githubusercontent.com/ros2/rclpy/humble/rclpy/rclpy/__init__.py).

I would keep periodic supervision and improve those gaps.

**4. How big is a cell?**

Your [SLAM configuration](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_robot_sdk/config/mapper_params_online_async.yaml:32) uses **0.05 metres per cell**.

That means:

- One cell is **5 cm × 5 cm**.
- Five cells in a straight row span **25 cm**.
- “Ignore groups smaller than five cells” means ignore frontier groups containing only one to four cells.

This filters tiny boundaries that may be noise. **It does not establish that an opening is wide enough for Oak**, and 5 cm map resolution does not mean 5 cm positioning accuracy.

The threshold is expressed in cells, so changing map resolution changes its physical meaning.

**5. Why use imaginary rays and straight-line distance?**

There are two separate decisions here.

**Estimating new information is useful:** visiting an opening into an unseen room may reveal much more than visiting another small unexplored patch beside Oak. Exploration research commonly balances expected information against the cost of reaching it. [Burgard and colleagues](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/burgard05tro.pdf).

**Using straight-line distance is cheaper to calculate, but less accurate than a planned route.**

For example, a destination may be two metres away across a wall but require fifteen metres of walking through a doorway. Your score charges for the two metres.

The imaginary rays are also an approximation:

- The default is **48 directions**, extending **4 metres**.
- Those directions are **7.5° apart**.
- At four metres, neighboring rays are roughly **half a metre apart**, so a narrow opening can fall between them.
- Rays continue through unknown cells, even though those cells could contain unseen walls.

**What I would change:** retain inexpensive scoring to select a few promising candidates, then ask Nav2 to calculate routes for that shortlist. Reject unreachable candidates and compare actual path lengths. Humble already provides `ComputePathToPose` for requesting a path without executing movement. [Humble planning action](https://api.nav2.org/actions/humble/computepathtopose.html).

**6. Why is forward missing from recovery?**

The [candidate list](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/recovery.py:347) contains:

- Normal recovery: backward, left and right.
- A special fallback mode: those directions plus backward diagonals.
- No direct forward recovery candidate.

That is understandable when the trigger is **an obstacle directly ahead**. Forward movement still happens during normal Nav2 navigation and navigation toward an open-space destination.

But this is not a complete escape strategy for every possible stuck position. I would first use a validated Nav2 route when forward space is available, rather than blindly adding forward movement to a helper whose checks were built around retreating from frontal obstacles.

There is another restriction: it generally tests **one translation length**, normally 0.3 m. A shorter move might fit where that move does not. Trying several bounded lengths is worth evaluating, but the code review cannot prove that doing so would improve your runs.

**7. The concrete problems I would address first**

**A. Cancelling a movement is sometimes treated as though it has already stopped.**

After the navigation timeout, the code requests cancellation and immediately clears the active-goal state. It can then proceed toward another decision before confirming the old action ended. See the [timeout handling](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/explorer.py:2434).

ROS distinguishes **“cancel requested/in progress”** from **“cancelled.”** [ROS action states](https://design.ros2.org/articles/actions.html).

Shutdown has a related gap: it cancels its timers, sends one zero-speed command, and can then wait seconds for cancellations. The mapper’s command priority expires after **0.5 seconds**.

**I would change:** use a consistent “stopping” state and maintain the stop override until the previous action is confirmed finished.

**B. Missing information and clear space are not consistently distinguished.**

Two confirmed behaviors:

- [The normal obstacle watcher](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/explorer.py:2144) returns when its local view is unavailable.
- `obstacle_sensing.py` looks for lethal obstacle cells; unknown and outside-the-grid samples do not count as obstacles. Another path uses that result to release a stopped condition.

“No obstacle was found” and “we have enough evidence that the area is clear” are different conclusions.

**I would change:** represent **clear, blocked and unknown** separately, and establish an explicit response to stale or missing data. Nav2 has other checks, but this watcher should not be described as a complete independent safeguard.

**C. Every destination requests the same final facing direction.**

In [send_goal()](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/explorer.py:1182), the orientation is set to quaternion `(0, 0, 0, 1)`.

That means **face the map’s zero-angle direction**. It does not mean “keep Oak’s current orientation,” despite the comment.

With the configured heading tolerance, this can request an extra turn at a destination.

**I would change:** choose a deliberate observation or arrival direction, rather than automatically requesting the same map heading everywhere.

**D. The planner and controller disagree about permitted movement.**

The [navigation configuration](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_robot_sdk/config/nav2_params.yaml:150) combines:

- A Reeds–Shepp planner that can produce reverse segments.
- A DWB controller with `min_vel_x: 0.0`, which does not generate reverse driving commands.
- No sideways velocity in normal navigation, although direct recovery uses sideways movement.

A planner’s path should match the motions its controller can execute. Humble provides planners for different motion models; choosing compatible settings matters more than selecting a supposedly “best” planner. [Humble Smac documentation](https://api.nav2.org/nav2-humble/html/md_nav2_smac_planner_README.html).

**I would change:** establish one intended movement model for Oak and align planning, control, smoothing and recovery with it.

**E. Unknown space can become free space before the planner sees it.**

Your global costmap has `track_unknown_space: False`, while the planner has `allow_unknown: false`.

That looks restrictive, but Humble’s static layer converts unknown cells to free cells when unknown tracking is disabled. The planner cannot reject unknown cells once that distinction has been erased. [Humble static-layer implementation](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/plugins/static_layer.cpp).

**I would change:** make the unknown-space policy consistent across map processing, planning and recovery. This is a configuration mismatch—not proof that it caused a particular collision.

**F. The selected destination and the route to it are checked differently.**

The explorer requires a goal centre to be about **0.5 m from walls**. A straight corridor 0.8 m wide could accommodate the configured 0.4 m-wide body when aligned, but no centre position in that corridor meets the 0.5 m requirement on both sides.

Meanwhile, `open_space.py` checks free-cell connectivity and allows diagonal steps between cells without checking the intervening corners. A free endpoint can therefore be connected in its search through a passage the body cannot traverse.

**I would change:** distinguish preferred clearance from actual collision feasibility, and validate shortlisted destinations with Nav2’s footprint-aware planner. Nav2 recommends using the actual polygon footprint for noncircular robots. [Nav2 tuning guidance](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/).

**G. The planner’s permitted endpoint error is unusually large.**

Your Smac planner tolerance is **3 metres**. Humble can return an approximate endpoint within its tolerance when it cannot reach the exact requested goal. [Humble planner implementation](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_smac_planner/src/a_star.cpp).

That is large compared with the explorer’s 0.8 m frontier offset.

**I would change:** align the planner tolerance with the observation task, and check the returned and reached position before calling a frontier visit successful. I would not simply set every tolerance to zero.

**H. “The frontier did not shrink” is an unreliable definition of failure.**

The [progress check](/Users/adibrouf/Desktop/Projects/go2-fusion/src/go2_auto_mapper/go2_auto_mapper/explorer.py:1131) can blacklist a frontier when the overlapping frontier group stays the same size or grows.

But discovering a larger room can produce **more boundary**, while also revealing useful new space. The helper counts entire new groups that overlap the old frontier, making that disagreement possible.

**I would change:** evaluate newly observed space around the intended viewpoint and repeated lack of useful observations. Keep retry suppression, but do not equate a longer boundary with failure.

**8. What seems redundant or insufficiently justified**

**Overlapping recovery responsibilities.** Nav2’s default Humble behavior tree already includes replanning and recovery actions. The explorer adds its own cancellation, spins, escape selection, direct movement and stop states. Extra supervision can be useful, but these systems need clear ownership of movement. [Default Humble navigation tree](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml).

I would simplify their coordination before adding more exceptions.

**An unconditional full spin after every successful goal.** The code schedules this by default; it does not first measure whether sufficient useful information was already collected. I would evaluate a conditional observation step, using map gain and available clearance. I would not claim that spinning always improves localization just because a comment says so.

**People avoidance claims exceed the implemented behavior.** The code applies destination preferences and can pause for six seconds. When the pause ends, it releases the stop; it does not create a new route around the person. The log saying “carrying on around them” is therefore too strong.

**Optional model complexity.** VLM ranking, VLM direction suggestions and the separate SocialNav viewer are not necessary to establish a reliable geometric mapper. I would keep active model influence out of baseline comparisons until the navigation problems are understood. Their presence is not evidence of improved coverage.

There are also timing problems in those optional paths: images can be paired with the latest pose rather than capture-time pose, and a delayed suggestion can be marked fresh when its result arrives. Those need correction before treating model recommendations as current observations.

**Duplicated configuration.** Body dimensions occur in multiple helpers and Nav2 settings. They currently agree on the main rectangle, but future edits can make them disagree. Distances expressed as fixed cell counts also change meaning if resolution changes.

**Actual unused code.** The explorer constructs an unused backup action client, contains unused backup constants, and defines `SHUTDOWN_ACCEPT_WAIT_SEC` twice. These are straightforward cleanup items.

**Logs are useful, but their labels need care.** `nav_log.py` calls the change in total known-cell count “revealed cells.” That change can include recovery movement and SLAM revisions; it is not an independent measure of room coverage or the benefit of one destination. I would keep the evidence and describe the measurement accurately.

**9. The order I would work in**

1. **Fix stopping, cancellation and missing-data behavior.**
2. **Align unknown-space policy, motion capabilities, goal heading and endpoint tolerances.**
3. **Validate candidate routes and replace frontier-shrinkage as the main progress test.**
4. **Evaluate recovery distances and conditional spins using repeatable comparisons.**
5. **Consolidate configuration and remove unused code.**
6. **Evaluate people/model features separately after the basic mapper has a dependable baseline.**

I would keep frontier detection, asynchronous Nav2 goals, periodic supervision, retry history and logging. Those are useful foundations; the strongest opportunities are making their decisions consistent and measuring what actually happens.
