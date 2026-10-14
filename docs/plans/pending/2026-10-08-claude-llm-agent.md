> Historical proposal: superseded for the typed local Ollama milestone on 2026-10-12. See [the agent README](../../../go2_llm_agent/README.md). Voice, vision and object-navigation features below are not part of that milestone.

---
created: 2026-10-08
status: pending
blocked_on: nothing - ready to build on 2026-10-09 after the team briefing; Max needs an ANTHROPIC_API_KEY for the live test
last_reconciled: 2026-10-08
---

# Claude LLM agent with voice commands

**Goal:** rebuild `go2_llm_agent` on Claude so a teammate can say "walk to the chair and sit" and Oak does it safely. Then
Oak can answer out loud and describe what its camera sees.

## Facts established (don't re-derive)

- Riley's agent (`go2_llm_agent/`, about 540 lines) uses GPT-4o-mini through LangChain and LangGraph. Its flow: the host mic
  tool (`host_tools/speech_to_text/vad_listen.py`, faster-whisper) POSTs `{"text"}` to FastAPI `:8000/command`, a planner
  LLM call writes every step up front, then one more LLM call per step picks a tool.
- Bugs confirmed by reading the code (not run):
  - It crashes with no API key, even in mock mode, because the client and `rclpy.init` are built at import time.
  - Live `look_for_object` always errors.
  - The plan is fixed before anything runs and never sees tool results.
  - Speed and turn direction get lost, so "turn right" turns left.
  - It loops forever when no tool is called.
  - It has no speed caps.
  - It has no stop, and the web handler blocks while the robot moves.
- The fusion already did these: the agent publishes on `cmd_vel_agent` (twist_mux priority 4, below Nav2 at 5, keyboard at 9
  and the gamepad at 10 while LB is held), and its sit sends `parameter="{}"`. Its pip deps are NOT in the image.
- Topics to reuse, with no new ones: `cmd_vel_agent`, `/webrtc_req` (Sport API: 1004 stand up, 1005 stand down,
  1006 recovery, 1009 sit, 1017 stretch), `/tts` (our `tts_node` speaks it), `/detected_objects_dict` (JSON from
  `object_detection`), `/navigate_to_object` (`object_navigation`), and `/camera/image_raw`.
- Claude, from the `claude-api` skill on 2026-10-08:
  - Native tool use replaces LangChain and LangGraph with a manual loop of about 40 lines.
  - Use `tool_choice: auto`; these models reject forced tool choice.
  - Price per command at about 3 round trips and ~6k in / 1k out tokens: Haiku 5.5 about $0.001, Sonnet 5.5 about $0.02,
    Opus 5.5 about $0.04. A camera frame adds about 500 tokens.
- Build check: from `docker/`, run `docker compose -p go2fusion run --rm unitree_ros bash -lc true`. All 14 packages
  build today.

## Steps

1. [ ] Branch `feat/claude-agent`. Swap the image's pip deps: add `anthropic` (pinned), plus `fastapi`, `uvicorn` and
   `python-dotenv`. Leave out LangChain, LangGraph and the pydantic pin. Add `env_file: .env` to both compose files,
   `docker/.env.example` with a blank `ANTHROPIC_API_KEY`, and `.env` in `.gitignore`.
2. [ ] `ros_io.py`: one node, `llm_agent`. It publishes to the topics above, and caches the latest object dict and camera
   frame.
3. [ ] `tools.py` with `strict: true` schemas, plus a printing `mock.py` backend that keeps Riley's fake scene.
   - Tools: `walk(duration_s|distance_m, speed_mps)`, `turn(degrees)` (signed), `sit`, `stretch`, `stand_up`,
     `stand_down`, `recovery_stand`, `stop`, `say(text)`, `look_for_object(label?)`, `go_to_object(name)`,
     `describe_scene(question)`.
   - Caps enforced in code: 0.4 m/s, 1.0 rad/s, 5 s per motion.
4. [ ] `agent.py`: the manual tool loop and system prompt.
   - Build the client lazily, and read the model from `AGENT_MODEL` (default `claude-haiku-5-5`, effort low).
   - Limits: at most 8 tool calls and 60 s per command; client `timeout=20`, `max_retries=2`.
   - On any error or refusal: send a zero twist and say "I couldn't do that".
   - With no key, a keyword parser drives the same tools, so it runs offline.
5. [ ] `server.py`:
   - FastAPI `POST /command` returns right away and runs the command in a worker thread with a cancel flag.
   - A new command cancels the old one.
   - "stop/halt/freeze" is matched locally, before any LLM call.
   - Also add a typing CLI.
6. [ ] Wire up the object tools.
   - `look_for_object` reads `/detected_objects_dict`.
   - `go_to_object` publishes `/navigate_to_object`, then polls until the object is under 0.6 m away, is lost, or 20 s
     pass.
   - Patch `object_navigation` (about 5 lines) so `"stop"` cancels.
   - `describe_scene` sends one 640 px JPEG frame.
7. [ ] Voice: keep Riley's host mic tool, and stop its 5 s POST timeout from reporting a false failure. Document the
   end-to-end run in `go2_llm_agent/README.md`.
8. [ ] Tests:
   - Offline: the keyword-mock suite, no key needed.
   - Docker: simulated twist_mux check (pattern: the gamepad test from 2026-10-08) proving the agent yields to keyboard
     and gamepad.
   - Docker: a live Claude call with Max's key.
9. [ ] FUSION.md row plus a PR. Robot test at ISL: on a stand first, then on the floor at low speed.

## Decisions for Max

- Default model: Haiku 5.5 (recommended for speed and cost) or Sonnet 5.5. Measure latency on day one either way.
- Whose API key and billing: Max's personal key, or a team or lab account.
- Who on the team should get commit rights to this branch.

## Reconciliation log
