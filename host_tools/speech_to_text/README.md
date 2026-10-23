# Speech to text (host tool)

`vad_listen.py` listens to your microphone, turns speech into text with Whisper, and sends it to the LLM agent's speech bridge at `http://localhost:8000/command`. It runs on your own computer, outside Docker (Windows Python is easiest for the microphone).

## Setup (once)

```shell
cd host_tools/speech_to_text
python -m venv .venv
# Windows: .venv\Scripts\activate    Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
```

The first run downloads the Whisper `base` model (about 150 MB).

## Run

1. Inside the container: `ros2 run go2_llm_agent speech_bridge` (port 8000; on Docker Desktop for Windows also publish port 8000 in `docker-compose.windows.yml`).
2. On the host: `python vad_listen.py`, then speak a command. Ctrl+C stops it.

Full explanation: [Voice wiki page](../../docs/wiki/voice.md)
