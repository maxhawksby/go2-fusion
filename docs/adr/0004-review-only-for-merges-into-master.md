# Review only for merges into master

Date: 2026-10-22

## Context

Five people share one robot and one `master`. Reviewing every push would stall everyone.

## Decision

Your own branches are a sandbox. Merging into `master` needs one teammate's approval and green checks. Since 2026-10-26 that approval is only for changes to Oak's software (`src/`, `docker/`, `host_tools/`, the build check); wiki and team tooling merge once checks pass.

## Consequences

Fast personal work, one gate where it matters. A change can still merge unproven if the reviewer does not ask for evidence.
