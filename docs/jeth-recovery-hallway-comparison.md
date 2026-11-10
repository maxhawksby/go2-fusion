# Jeth recovery versus the pinned hallway baseline

This is an experimental simulation checkout, not a merge recommendation.
It reuses the completed five-run baseline rather than asking for another one.

## What is being compared

- Control source: `4c2fc492abef5587c7daef65d82102726c38b4f8`, the hallway
  branch after merged Luis #55, with VLM off and camera disabled.
- Candidate: the same control plus Jeth's recovery commit
  `fd78cac5b83596c6fa2e9a38aa3f449d3b23db75`, ported as `383ef1e`.
- A conflict between adjacent methods was resolved by retaining both Luis's
  people-yielding method and Jeth's forward-motion helper. Shutdown/event
  handling, frontier ranking and VLM code are otherwise retained.
- Only explorer recovery, its geometry helpers and recovery tests change.
  Jeth's Header repair, the TF repair, voice, collision monitor and Nav2 tuning
  are deliberately excluded so the effect of recovery can be measured.

Jeth's changes add a close-wall leaving policy with tighter drift bounds,
backward diagonals, Nav2 recovery turns toward open scan space, and a guard
against cancelling goals unnecessarily while turning. This is not a new
frontier-selection algorithm and may not resolve all navigation failures.

## Completed control

The November 10 `baseline_five_seeds` run used `ite_hallways`, seeds
28000-28004, 300-second limits, 180-second stall/startup limits, bags on,
no explorer overrides and default simulator dials. Source and world are
preserved in its session manifest. No people or active VLM were involved.

| Seed | Coverage | Independent odometry distance | End reason |
| --- | ---: | ---: | --- |
| 28000 | 55.8% | 27.1899 m | timeout |
| 28001 | 41.8% | 21.7859 m | stuck |
| 28002 | 35.1% | 11.5694 m | stuck |
| 28003 | 40.9% | 37.5785 m | timeout |
| 28004 | 25.9% | 12.1497 m | stuck |

Mean coverage: 39.9%. Formal completions: 0/5. Collisions: zero.
All five explorer logs eventually reported the no-safe-direction latch.
Three runs ended under the 180-second lack-of-progress rule.

Every control row flags distance parity failure: the live scoreboard distance
is much smaller than the independent odometry accumulator. Coverage and collision
counts agree. Do not use the UI distance values to claim a recovery improvement;
use independent CSV odometry/Gazebo distance provisionally and keep the raw bags
for the separate measurement investigation. The candidate keeps the same scorer
code, so no hidden distance repair is mixed into the algorithm comparison.

## User-run candidate

Run from this checkout in PowerShell:

```powershell
.\scripts\run_ite_hallways.ps1 -Mode Batch -Runs 5 -Seed 28 -Timeout 300 -Variant jeth_recovery_five_seeds -RecordBag
```

The same seed in the same unchanged world selects the same five starts.
Scheduling and SLAM/Nav2 remain variable; one five-run batch is evidence,
not a guarantee. Do not run another simulator concurrently. Compare per-seed
coverage, reached/failed goals, latches, duration, collisions and independent
travel. Count any new collisions explicitly; a higher mean alone is not enough.

No candidate simulation was launched during preparation. Review the saved
candidate results before proposing a production merge or an Oak trial.
