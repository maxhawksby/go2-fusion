# go2_llm_agent

Turns a typed or spoken plain-English command ("walk forward then sit") into robot actions with an LLM. On master this is Riley Francis's GPT-4o-mini version, whose pip deps are not in the Docker image, so it builds but does not run; the team is moving it to a local Ollama model on `adib/feat/llm-agent`.

## Run

Inside the container, with `OPENAI_API_KEY` set and `ROBOT_ENV=testing` (mock, the default) or `live`; never commit the key:

```bash
ros2 run go2_llm_agent agent            # interactive text prompt
ros2 run go2_llm_agent speech_bridge    # HTTP bridge on port 8000
curl -X POST http://localhost:8000/command -H "Content-Type: application/json" \
  -d '{"text":"walk forward for one second"}'
```

For voice, run [host_tools/speech_to_text](../../host_tools/speech_to_text/README.md) on the host.

Full explanation: [LLM agent wiki page](../../docs/wiki/llm-agent.md)
