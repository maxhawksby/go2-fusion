# Review only for merges into master

Date: 2026-10-22

## Context

Five people share one robot and one `master`. Reviewing every push would stall everyone.

## Decision

Your own branches are a sandbox. Merging into `master` needs one teammate's approval and green checks. Since 2026-10-26 that approval is only for changes to Oak's software (`src/`, `docker/`, `host_tools/`, the build check); wiki and team tooling merge once checks pass. Since 2026-11-15 the `review-gate` check enforces this from the changed files: robot changes need one teammate's approval, tooling and agent rules need one approval or Max's merge, and wiki pages and guides merge by themselves. Since 2026-11-16 the tier follows what a change can do to Oak, not only where the file sits: the scoreboard (`src/go2_scoreboard/`), the simulator (`src/go2_sim/`) and package tests cannot move Oak, so they count as tooling and Max may merge them as admin when no teammate is working. A change to their `package.xml` still needs a teammate, because it changes the Docker image. The size of a change does not lower its tier: on 2026-11-16 a one-line Nav2 setting helped Oak walk into a backpack (#127).

## Consequences

Fast personal work, one gate where it matters. A change can still merge unproven if the reviewer does not ask for evidence.
