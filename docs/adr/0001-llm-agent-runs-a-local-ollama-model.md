# LLM agent runs a local Ollama model

Date: 2026-10-22

## Context

The first plan (2026-10-08) was to rebuild Riley's LLM agent on Claude, because it needed close to a rewrite either way. Adib then built an agent that runs a free local Qwen model through Ollama.

## Decision

The agent runs a local Qwen model through Ollama. Simulation is the default and real moves reuse `/cmd_vel_agent`. The Claude rebuild was dropped and its plan deleted (git history keeps it).

## Consequences

No paid API key, billing account or extra Python libraries. Voice and object navigation are out of scope for the first step. Until `adib/feat/llm-agent` merges, the agent on `master` builds but does not run. See [LLM agent](../wiki/llm-agent.md).
