import asyncio
import collections
import io
import logging
import os
import sys
import wave

import av
import numpy as np
import requests
import webrtcvad
from faster_whisper import WhisperModel
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

logging.basicConfig(level=logging.FATAL)

# CONFIG
ROBOT_IP = os.environ.get("UNITREE_ROBOT_IP", "192.168.50.150")

RATE = 16000                   # what VAD + Whisper use (robot audio is resampled to this)
CHANNELS = 1

FRAME_MS = 30                  # must be 10, 20, or 30 for webrtcvad
FRAME_SAMPLES = int(RATE * FRAME_MS / 1000)
FRAME_BYTES = FRAME_SAMPLES * 2  # int16 = 2 bytes

VAD_MODE = 3                   # 0-3, higher = stricter
START_SPEECH_FRAMES = 3        # require this many voiced frames to start
END_SILENCE_MS = 900           # stop after this much silence
MAX_UTTERANCE_SEC = 8          # hard cap
PRE_ROLL_MS = 300              # keep some audio from before speech starts

POST_URL = "http://localhost:8000/command"

WHISPER_MODEL = "base"         # try "tiny" for lower latency
WHISPER_DEVICE = "cpu"
WHISPER_COMPUTE = "int8"


# SETUP
model = WhisperModel(
    WHISPER_MODEL,
    device=WHISPER_DEVICE,
    compute_type=WHISPER_COMPUTE,
)

pre_roll_frames = max(1, PRE_ROLL_MS // FRAME_MS)
end_silence_frames = max(1, END_SILENCE_MS // FRAME_MS)
max_frames = int((MAX_UTTERANCE_SEC * 1000) // FRAME_MS)

# Converts whatever the robot sends (48 kHz stereo, packed or planar) into
# 16 kHz mono signed 16-bit PCM. The resampler keeps its own state, so
# chunk boundaries between frames are handled correctly.
resampler = av.AudioResampler(format="s16", layout="mono", rate=RATE)

utterance_queue: "asyncio.Queue[bytes]" = None  # created in main()


def frame_to_pcm16k_mono(frame) -> bytes:
    out = resampler.resample(frame)
    if out is None:
        return b""
    if not isinstance(out, (list, tuple)):  # older PyAV returns a single frame
        out = [out]
    chunks = []
    for f in out:
        if f is None:
            continue
        chunks.append(f.to_ndarray().astype(np.int16).tobytes())
    return b"".join(chunks)


class UtteranceSegmenter:
    """Same VAD state machine as the mic version, fed by arbitrary-sized PCM chunks."""

    def __init__(self):
        self.vad = webrtcvad.Vad(VAD_MODE)
        self.buf = bytearray()
        self.pre_roll = collections.deque(maxlen=pre_roll_frames)
        self.recording = False
        self.speech_run = 0
        self.silence_run = 0
        self.utterance_frames: list[bytes] = []

    def feed(self, pcm_bytes: bytes) -> list[bytes]:
        utterances = []
        self.buf.extend(pcm_bytes)
        while len(self.buf) >= FRAME_BYTES:
            frame = bytes(self.buf[:FRAME_BYTES])
            del self.buf[:FRAME_BYTES]
            utt = self._process_frame(frame)
            if utt is not None:
                utterances.append(utt)
        return utterances

    def _process_frame(self, frame: bytes):
        voiced = self.vad.is_speech(frame, RATE)
        self.pre_roll.append(frame)

        if not self.recording:
            if voiced:
                self.speech_run += 1
            else:
                self.speech_run = 0

            if self.speech_run >= START_SPEECH_FRAMES:
                self.recording = True
                self.silence_run = 0
                self.utterance_frames = list(self.pre_roll)
                print("Speech detected...")
            return None

        # recording mode
        self.utterance_frames.append(frame)

        if voiced:
            self.silence_run = 0
        else:
            self.silence_run += 1

        too_long = len(self.utterance_frames) >= max_frames
        enough_silence = self.silence_run >= end_silence_frames

        if enough_silence or too_long:
            pcm_bytes = b"".join(self.utterance_frames)
            self.recording = False
            self.speech_run = 0
            self.silence_run = 0
            self.utterance_frames = []
            return pcm_bytes

        return None


segmenter = UtteranceSegmenter()


def pcm_to_wav_bytes(pcm_bytes: bytes) -> io.BytesIO:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # int16
        wf.setframerate(RATE)
        wf.writeframes(pcm_bytes)
    buf.seek(0)
    return buf


def transcribe_pcm(pcm_bytes: bytes) -> str:
    wav_buf = pcm_to_wav_bytes(pcm_bytes)
    segments, info = model.transcribe(
        wav_buf,
        beam_size=1,
        vad_filter=False,   # we already did VAD ourselves
    )
    text = " ".join(seg.text.strip() for seg in segments).strip()
    return text


def send_text(text: str):
    try:
        r = requests.post(
            POST_URL,
            json={"text": text},
            timeout=5,
        )
        print(f"[send] {r.status_code} {r.text}")
    except Exception as e:
        print(f"[send error] {e}")


# Called by the Unitree WebRTC driver for every incoming audio frame.
async def recv_audio_stream(frame):
    try:
        pcm = frame_to_pcm16k_mono(frame)
        if not pcm:
            return
        for utt in segmenter.feed(pcm):
            print("End of speech. Transcribing...")
            utterance_queue.put_nowait(utt)
    except Exception as e:
        print(f"[audio error] {e}", file=sys.stderr)


# Transcription runs in a worker thread so the blocking Whisper call
# never stalls the WebRTC event loop (which would drop the connection).
async def transcription_worker():
    loop = asyncio.get_running_loop()
    while True:
        pcm_bytes = await utterance_queue.get()
        try:
            text = await loop.run_in_executor(None, transcribe_pcm, pcm_bytes)
            print(f"Transcript: {text!r}")
            if text:
                await loop.run_in_executor(None, send_text, text)
        except Exception as e:
            print(f"[transcribe error] {e}", file=sys.stderr)
        print("\nListening again...\n")


async def main():
    global utterance_queue
    utterance_queue = asyncio.Queue()

    try:
        # Choose a connection method (uncomment the correct one)
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP)
        # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)

        await conn.connect()

        asyncio.create_task(transcription_worker())

        # Switch audio channel on and start receiving audio frames
        conn.audio.switchAudioChannel(True)
        conn.audio.add_track_callback(recv_audio_stream)

        print("Listening via Go2 microphone... speak to issue a command.")
        print("Press Ctrl+C to stop.\n")

        # Run until interrupted
        await asyncio.Event().wait()

    except ValueError as e:
        logging.error(f"Error in WebRTC connection: {e}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)
