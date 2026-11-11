# Hallway reliability experiment

The shared simulation baseline is pinned in `hallway-reliability-manifest.json`.
It contains merged Luis functionality (camera/VLM off), Jeth recovery, callback
ownership and prepared scoreboard scoring. These experimental integrations do
not replace teammate approval of the independent PRs. Historical worktrees,
including the old uncommitted heading prototype, remain untouched.

## User-run schedule

From this experiment checkout, run `./scripts/run_heading_comparison.ps1` to
inspect the schedule only. Run `./scripts/run_heading_comparison.ps1 -Run` to
execute it. The agent must not launch mapping runs. Ten sequential trials use
seeds 28000–28004, 300 seconds each, both `map_zero` and `toward_goal`, alternating
which mode runs first per seed. Both modes use the same binary/source snapshot,
world, recovery, measurement fixes and VLM-off setting. Camera is disabled by
the simulator default. No clearance/planner/controller parameters are changed.

The comparison manifest lives under ignored `go2_sim/results/ite_hallways/
comparisons/`. Each attempt points to the full saved session, including its
source commit, image identity, world checksum, trial CSV, logs, map and bag.
Keep all valid scores, including regressions. A collision or parity failure
pauses the schedule with that valid row retained. Inspect before resuming with
`-Run -ResumeFrom <comparison.json>`; only unfinished/infrastructure-failed
entries are attempted again. A different source commit cannot resume a plan.
The initial no-Run preview is not the final runnable manifest if source is
subsequently committed; create a new plan after the source is clean.

## Analysis and acceptance

Report per-seed coverage differences, completed/timeout counts, collisions,
per-goal wait/motion intervals and measurement parity. Separate infrastructure
failures from valid mapping runs. Report time-to-completion only for completed
runs, with timeouts explicitly censored. No claim of statistical significance
from five pairs. No automatic default change based on one favorable run.

If the candidate improves paired mean coverage without collisions or fewer
completions, repeat both modes with `-Seed 29` for held-out starts 29000–29004.
Otherwise diagnose the mixed results before another feature. Keep the default
`map_zero` until teammate review and held-out validation support changing it.
Both candidate and control still require supervised physical validation.

## Proposed team responsibilities

These are coordination suggestions, not assignments sent to teammates:
Noah owns manifests/results and review coordination; Jeth supplies recovery
refinement plus exact settings for his high-coverage runs; Luis owns frontier
choice and shared-seed camera/VLM ablations; Adib checks localization in long
symmetric corridors; Max reviews watchdog and recovery/execution contracts.
Obtain the current exact Jeth commit, world, seed list, run limit, launch
settings and metric definition before comparing his Discord percentages.

After selecting a stable algorithm, separately hold one starting position
fixed and evaluate four headings with two repetitions each. Do not infer a
best heading from the current seeds, which change position and yaw together.
The assumed 24 m world is not a measured ITE replica. Longer worlds, perception
models, UI consolidation and physical autonomy remain outside this experiment.
