# Review only for merges into master

Date: 2026-10-22

## Context

Five people share one robot and one `master`. Reviewing every push would stall everyone.

## Decision

Your own branches are a sandbox. Merging into `master` needs one teammate's approval and green checks. Since 2026-10-26 that approval is only for changes to Oak's software (`src/`, `docker/`, `host_tools/`, the build check); wiki and team tooling merge once checks pass. Since 2026-11-15 the `review-gate` check enforces this from the changed files: robot changes need one teammate's approval, tooling and agent rules need one approval or Max's merge, and wiki pages and guides merge by themselves.

## Consequences

Fast personal work, one gate where it matters. A change can still merge unproven if the reviewer does not ask for evidence.
