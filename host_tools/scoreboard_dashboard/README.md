# Oak Mission Scoreboard UI

This is a read-only browser display for `go2_scoreboard` results. It turns the
machine-readable run artifacts into a map, mission metrics, a timeline and an
evidence package. It runs on the Windows host, uses only Node's built-in
modules and never publishes a ROS command.

## Start it

From the scoreboard UI worktree:

```powershell
cd "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion\.worktrees\noah-scoreboard-ui"

node host_tools/scoreboard_dashboard/server.mjs
```

Open <http://127.0.0.1:3100>. The server discovers every `run.json` below
`src/go2_scoreboard/results`, including nested simulator trials. While the
recorder is active, it reloads the atomic checkpoint once per second.

The UI branch does not contain the ignored simulation artifacts from the
scoreboard worktree. To inspect the runs already generated during development,
start it with that results folder:

```powershell
node host_tools/scoreboard_dashboard/server.mjs --results-root "C:\Users\noahs\OneDrive - University of Connecticut\Fall26\CSE4939W\go2-fusion\.worktrees\noah-scoreboard\src\go2_scoreboard\results"
```

Keep that PowerShell window open while viewing the page. Press `Ctrl+C` in the
same window to stop the display.

## What to check

1. Pick a run in **Mission archive**. A `replay` row is an offline re-score of
   a bag; a `frontier_seed_*` row is the original simulator run.
   Use **Sort runs** to order the archive by date, coverage, duration or name.
   Runs without a ground-truth completeness percentage remain at the end of
   either coverage order.
2. Confirm the selected run's state and timestamp at the top. A recorder that
   stopped without finalizing is shown as `INTERRUPTED`, not as a live run.
3. Inspect **Map completeness**. This is only available when a ground-truth
   answer map was supplied. Otherwise the UI correctly shows known area only.
4. Inspect **What Oak has seen** for the saved occupancy map.
5. Check elapsed time, distance, goals, recoveries, collisions and human
   touches. `0` collisions and touches are the desired result.
6. Use **Coverage over time** to see whether mapping kept making progress or
   flattened before the run ended. Hover over the line to inspect the exact
   elapsed time and measured percentage or area at each sample.
7. Check warnings and the end reason. `COMPLETE` means the evidence finalized;
   it does not necessarily mean the whole room was mapped.
8. Download `run.json`, `scorecard.csv`, `timeline.csv` or `report.md` when the
   raw evidence is needed.

The current best fixed-seed simulation row is `frontier_seed_27000`: roughly
48% completeness, 2.7 m traveled and zero collisions. The 25% rows are useful
bag-and-replay proofs, not the best mapping result.

## How live runs appear

The ROS scoreboard recorder writes an atomic checkpoint about once per second.
If this server points at the same `results` directory, a new mission appears in
the archive automatically and its status changes while recording. No browser
restart is needed. Simulation and Oak use the same display; only the recorded
source and available metrics differ.

The server binds only to `127.0.0.1` by default. It exposes read-only `GET`
routes for summaries, timelines, maps and report downloads. It has no start,
stop, movement, WebRTC or robot-action endpoint. Mission controls remain a
later safety-reviewed feature.

## Troubleshooting

- **No runs found:** start the server with `--results-root` pointing at the
  worktree that contains the ignored results.
- **Address already in use:** another scoreboard server is already running.
  Reuse <http://127.0.0.1:3100>, stop its PowerShell window with `Ctrl+C`, or
  select another port with `--port 3101`.
- **A run says `RECORDING` after it ended:** that is a preserved checkpoint from
  an interrupted development trial, not a live robot connection.
- **Completeness says N/A:** the run did not include a ground-truth answer map;
  this is expected for most physical Oak runs.

## Tests

Run the host tests:

```powershell
node --test host_tools/scoreboard_dashboard/test/server.test.mjs host_tools/scoreboard_dashboard/test/run_sort.test.mjs
```
