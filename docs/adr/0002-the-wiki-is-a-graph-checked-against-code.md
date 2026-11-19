# The wiki is a graph checked against the code

Date: 2026-10-22

## Context

Team knowledge was spread over many guides that drifted from the code.

## Decision

`README.md` is the center page, Oak. Hubs and one page per program live in `docs/wiki/`. A check fails when code changes without its page being reread.

## Consequences

Every code change carries a wiki step. Pages stay short and current, at the cost of a little work per pull request.
