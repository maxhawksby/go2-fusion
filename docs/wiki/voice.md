---
title: Voice
parent: brains
code: [host_tools/speech_to_text/**, src/speech_processor/**]
---

# Voice

Voice lets you talk to Oak and lets Oak talk back, which matters for our goal: guiding a blind person who listens through earbuds.

Part of: [Brains](brains.md)

## How it works

Voice has two halves.

**Speech in (your voice to text).** `vad_listen.py` runs on your own computer, outside Docker, because the container cannot reach your microphone reliably. It notices when you start and stop talking (*VAD*, voice activity detection), turns your speech into text with **Whisper** (a free speech-to-text model), and sends the text to the [LLM agent](llm-agent.md)'s small web server at `http://localhost:8000/command`. It is plain Python, not a ROS package, written by Riley Francis in [ROS2-LLM-Nav](https://github.com/RileyFrancis/ROS2-LLM-Nav). It uses the Whisper `base` model by default; set `WHISPER_MODEL = "tiny"` for lower latency. The model size, silence timeout and bridge address are constants at the top of `vad_listen.py`. The agent's server (`speech_bridge`) runs inside the container on port 8000; the Linux compose file uses host networking, so `localhost:8000` reaches it on Linux and WSL.

**Speech out (text to Oak's speaker).** The `tts_node` program in `speech_processor` listens on the `/tts` topic. When text arrives, it turns it into audio (*TTS*, text to speech), cuts the audio into small pieces, and sends them to Oak's speaker as commands on `/webrtc_req`. On `master` the voice comes from **ElevenLabs**, a paid online service, so it needs an `ELEVENLABS_API_KEY`.

**Piper, the free local voice.** On branch `jeth/feat/tts`, Jeth added **Piper**, a text-to-speech model that runs on your own computer, and made it the default. Noah's review branch `noah/tts-piper-review` (pull request #13) fixes how the audio is uploaded: it checks the audio file and sends it in pieces through the robot's "megaphone" audio mode (AudioHub API 4003), with tests.

Example: publishing `ros2 topic pub --once /tts std_msgs/String "data: 'Hello, I am Oak'"` makes Oak say that sentence, if `tts_node` is running with a working voice.

## Connects to

- [LLM agent](llm-agent.md): the receiver for speech in. Note that Adib's new Ollama agent disables the old `speech_bridge` server, so speech in has nothing to talk to on that branch yet.
- [Go2 driver](go2-driver.md): carries `/webrtc_req` audio commands to the robot.
- The Oak Control dashboard (Noah, branch `feat/desktop-app-prototype`, pull request #14) has its own push-to-talk button in the browser and can publish `/tts`. See [People](people.md).

| Topic or address | What travels on it |
| --- | --- |
| `/tts` | Text for Oak to speak |
| `/webrtc_req` | Audio pieces sent to Oak's speaker |
| `localhost:8000/command` | Text from the microphone tool to the agent (not a ROS topic) |

## How it breaks

- **Piper audio is choppy and drops the start of short clips** (Jeth's latest findings).
- **TTS was not tested in the 2026-10-11 lab session** because it needed an ElevenLabs key ([Test log](test-log.md)).
- **No key, no voice on `master`:** the launch files pass `ELEVENLABS_API_KEY` from the environment; if it is empty, nothing is spoken.
- **Microphone in WSL is flaky.** Run `vad_listen.py` from Windows Python if you can.
- **Docker Desktop on Windows** needs port 8000 published in `docker-compose.windows.yml` for the microphone tool to reach the agent.
- Riley tuned the audio pieces to 4 KB every 0.2 s and fixed the audio topic name ([Decisions](decisions.md), row 5). That still needs a robot test.

## Who's on it and what's next

- **Jeth**: Piper research. He played local speech through Oak's speaker using a separate `unitree_webrtc_connect` driver and the megaphone mode. *Next:* fix the packet loss, then connect TTS to the voice command pipeline.
- **Noah**: reviewed Piper and wrote the upload fix.
- TODO(Jeth): which branch becomes the one to merge, and how speech in will reach Adib's new agent.

## The code

- Speech in: https://github.com/maxhawksby/go2-fusion/tree/master/host_tools/speech_to_text
- Speech out: https://github.com/maxhawksby/go2-fusion/tree/master/src/speech_processor

Speech in, on your own computer (first run downloads a 150 MB Whisper model):

```shell
cd host_tools/speech_to_text
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python vad_listen.py             # speak a command; Ctrl+C stops it
```

Start the receiver first, inside the container: `ros2 run go2_llm_agent speech_bridge` (Riley's agent; see [LLM agent](llm-agent.md)).

Speech out, inside the container (with the robot connected and an ElevenLabs key set):

```shell
ros2 run speech_processor tts_node
```

<!-- sources: host_tools/speech_to_text/README.md and vad_listen.py; src/speech_processor/speech_processor/tts_node.py; src/go2_robot_sdk/launch/robot.launch.py (ELEVENLABS_API_KEY); src/go2_llm_agent/README.md; docs/FUSION.md row 5; docs/TEST_LOG.md 2026-10-11; origin/jeth/feat/tts (43c242d); origin/noah/tts-piper-review (60fbe57, PR #13); origin/feat/desktop-app-prototype host_tools/control_dashboard/README.md (PR #14); origin/adib/feat/llm-agent README; team updates (Jeth, Noah) week of 2026-10-18 -->
