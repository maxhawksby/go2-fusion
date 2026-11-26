# LLM agent and VLM share one Ollama server

Date: 2026-11-26

Status: proposed (draft pull request, not yet agreed by the team)

## Context

Two parts of Oak call a Qwen model. The [LLM agent](../wiki/llm-agent.md) turns typed commands into robot steps; on `adib/feat/llm-agent` it runs `qwen3:4b-instruct-2507-q4_K_M` through Ollama at port 11434 ([decision 0001](0001-llm-agent-runs-a-local-ollama-model.md)). The [auto mapper](../wiki/auto-mapper.md)'s VLM ranks goals from camera pictures; on master it defaults to `Qwen/Qwen2.5-VL-7B-Instruct` on a vLLM-style server at port 8000. Two servers on two ports mean two setups, two SSH tunnels and two places for a model to be missing.

## Decision

Both use one Ollama server on port 11434. The auto mapper's VLM defaults to Qwen3-VL 8B Instruct (`qwen3-vl:8b-instruct`); the agent keeps its text model. Each model stays a setting (`VLM_URL`/`VLM_MODEL` for the VLM, `OLLAMA_BASE_URL`/`OLLAMA_MODEL` for the agent), so either can move to another server, including vLLM.

## Consequences

- One `ollama serve`, one tunnel. Ollama loads each model when first asked and unloads it after a while unused.
- Qwen3-VL needs a recent Ollama (0.20.3 refused it, 0.40.16 works).
- Two models on one GPU need room for both: TODO(Luis): measure the memory of `qwen3-vl:8b-instruct` and `qwen3:4b-instruct-2507-q4_K_M` loaded together.
- The VLM does not improve mapping yet: in the simulator it cost 7.1 points of coverage, from the camera and the wait rather than the model's picks ([evidence](../evidence/2026-11-26-qwen3-vl-ite-ablation.md)). This decision is about how the models are served, not about turning the VLM on; it stays off by default.
