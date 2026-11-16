# pstack model configuration

This project's sheet. In this repository it replaces the user's pstack-models.md; ignore that sheet's lines.

Per-role model overrides for pstack skills. Each pstack SKILL.md names its defaults in a Models section; the values here override those defaults. Delete a line to fall back to the skill default. A value of `inherit-parent` or `auto` runs that role on the parent session's model (the `Agent` call omits `model`); an alias entry in a panel list still counts toward that panel's fan-out. A model may carry a reasoning effort, as in `opus @xhigh` (levels: low, medium, high, xhigh, max); the role's subagent then runs at that level, each entry of a panel list on its own. `default effort` sets the level for a value without one; `session` uses the runtime's platform mapping to start from the parent session's effort. On Codex this requires active-turn metadata or an explicit effort choice. `session hook: off` stops the supported runtime's pstack routing hook from injecting the poteto-mode mandate; any other value, or no line, leaves it on.

The team is on mixed plans, so this sheet is light. Judgment roles run on whatever model your own session uses. Routine roles run on `sonnet`. Panels run two agents, not four.

feature, refactoring: sonnet
bug-fix: inherit-parent
perf-issue: inherit-parent
hillclimb: inherit-parent
judgment and prose: inherit-parent
strongest judgment: inherit-parent
how explorer: sonnet
how explainer: inherit-parent
why investigators: sonnet
why synthesizer: inherit-parent
reflect tooling: sonnet
reflect judgment, divergent, synthesizer: inherit-parent
arena runners: inherit-parent, sonnet
arena cross-judge pool: sonnet, opus
swarm workers: sonnet
architect runners: inherit-parent, sonnet
interrogate reviewers: inherit-parent, sonnet
thermos reviewers: inherit-parent
no-comments reviewer: sonnet

default effort: session
session hook: on
