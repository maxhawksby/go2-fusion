# Recorded goal-window diagnosis

Offline analysis of the five callback runs and the scoreboard-validation run.
Original bags are read-only; derived full timelines are saved in the workspace output folder.

| Run | Goals | Time in goal windows | Long windows (at least 50 s) |
| --- | ---: | ---: | --- |
| callback_28000_28001_run_01 | 8 | 209.8s | goal 7: 60.9s, 28.36m, goal_failed |
| callback_28000_28001_run_02 | 10 | 169.9s | none |
| callback_28002_run_01 | 6 | 221.4s | goal 1: 61.4s, 0.00m, goal_failed; goal 4: 61.1s, 2.30m, goal_failed; goal 5: 61.0s, 6.51m, goal_reached |
| callback_28003_run_01 | 11 | 170.4s | none |
| callback_28004_run_01 | 9 | 177.0s | goal 1: 61.7s, 0.01m, goal_failed |
| scoreboard_28003_run_01 | 9 | 113.1s | none |

Seed 28002 spent about 61 s on its first goal with zero observed translation,
then another 61 s on a later failed goal with about 2.3 m of travel. Its next
goal also lasted about 61 s but succeeded after about 6.5 m. Seed 28004 had
a roughly 62 s first goal with only 0.01 m of translation. Conversely, seed
28000 moved about 28 m during a long failed goal. Long elapsed time alone
therefore does not identify the same failure mode or justify reducing every
timeout. The post-scoreboard-fix run had only about 113 s inside explicit goal
windows; the remaining time includes scanning, sensing refresh, choosing goals
and any unobserved intervals. It cannot all be called stalled navigation.

The heading experiment targets a concrete final-yaw policy, not a proven sole
cause. Keep controller clearances unchanged. Use the new logs and per-goal
windows to determine whether near-goal orientation, translation feasibility,
selection cost or scan/refresh overhead dominates each start. Receipt-time
attribution is approximate and does not identify the underlying action UUID.
