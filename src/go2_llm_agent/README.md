# go2_llm_agent

Typed commands through local Ollama/Qwen, without a paid API key or extra Python
libraries. Simulation is the default; speech input and output are outside this feature.

## Build and run

Inside Docker, after following the [LLM agent setup](../../docs/wiki/llm-agent.md):

```bash
cbs go2_llm_agent
ros2 run go2_llm_agent agent --check
ros2 run go2_llm_agent agent
ros2 run go2_llm_agent agent --plan-only --command "walk forward for 3 seconds then turn left"
```

Live publishing requires `--mode ros` and a separately running driver and twist_mux.

## Test

Inside Docker (no robot or model needed):

```bash
python3 -m unittest discover -s /ros2_ws/src/go2_llm_agent/test -p test_agent.py -v
```

Full explanation: [LLM agent wiki page](../../docs/wiki/llm-agent.md).
