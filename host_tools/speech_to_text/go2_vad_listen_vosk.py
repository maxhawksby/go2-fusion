import asyncio
import collections
import json
import logging
import os
import re
import sys
import time

import numpy as np
import webrtcvad
from scipy.ndimage import median_filter
from scipy.signal import butter, find_peaks, iirnotch, sosfilt, tf2sos, welch
from vosk import KaldiRecognizer, Model, SetLogLevel

# DEBUG MODE: pass any command-line argument (e.g. `python go2_voice_control_vosk.py debug`).
# It still listens to the robot's microphone, but never starts ROS or publishes any command.
# The audio is recorded to WAV files (raw, and after the filters), each detected utterance is
# saved as its own WAV, and the Vosk transcript plus how it would be interpreted is printed.
# Add the word "mic" (e.g. `python go2_voice_control_vosk.py debug mic`) to use this computer's
# microphone instead of the robot (no robot connection at all).
DEBUG = len(sys.argv) > 1
DEBUG_MIC = DEBUG and any(a.lower() == "mic" for a in sys.argv[1:])

if DEBUG:
    import wave
if DEBUG_MIC:
    import queue

    import sounddevice as sd  # pip install sounddevice
else:
    import av
    from unitree_webrtc_connect.webrtc_driver import (
        UnitreeWebRTCConnection,
        WebRTCConnectionMethod,
    )
if not DEBUG:
    import rclpy
    from go2_interfaces.msg import WebRtcReq

logging.basicConfig(level=logging.FATAL)

# CONFIG
ROBOT_IP = os.environ.get("UNITREE_ROBOT_IP", "192.168.50.150")

RATE = 16000                   # what VAD + Vosk use (robot audio is resampled to this)

FRAME_MS = 30                  # must be 10, 20, or 30 for webrtcvad
FRAME_SAMPLES = int(RATE * FRAME_MS / 1000)
FRAME_BYTES = FRAME_SAMPLES * 2  # int16 = 2 bytes

VAD_MODE = 3                   # 0-3, higher = stricter
START_SPEECH_FRAMES = 3        # start an utterance when this many of the last START_WINDOW_FRAMES
START_WINDOW_FRAMES = 5        # frames are voiced (tolerates the VAD flickering on/off)
END_SILENCE_MS = 450           # stop after this much silence (lower = faster, but may cut off slow speakers)
MAX_UTTERANCE_SEC = 5          # hard cap (commands are short)
PRE_ROLL_MS = 300              # keep some audio from before speech starts

# FAN / NOISE REJECTION
BANDPASS_LOW_HZ = 300          # cuts fan rumble/hum below this
BANDPASS_HIGH_HZ = 3800        # cuts hiss above this (speech intelligibility lives in between)
GATE_START_RATIO = 3.0         # to START an utterance, a frame must be this many times louder than the fan...
GATE_CONTINUE_RATIO = 1.5      # ...but once speech has started, quieter frames (down to this) still count,
                               # so the soft part of a phrase is not cut off. Raise both if the fan
                               # triggers it, lower them if your voice is cut short or missed.
MIN_SPEECH_MS = 240            # discard utterances where speech lasted less than this: clicks, bursts

# The fan is not broadband hiss: it is a set of steady tones (a ~216 Hz whine plus its harmonics at
# ~431, 647, 865, 1075 Hz...) that sit INSIDE the speech band, so the band-pass cannot remove them.
# Narrow notch filters can. They are found automatically while the robot is idle, and follow the
# fan as its speed drifts. Detected tones are printed as "[fan] notch filters at ...".
FAN_NOTCH_AUTO = True          # auto-detect and track the fan tones
FAN_NOTCH_HZ = []              # optional fixed notch frequencies, e.g. [216, 431, 647, 865, 1075]
FAN_NOTCH_BW_HZ = 40           # width of each notch (wider tolerates more drift, removes a bit more voice)
FAN_MAX_TONES = 8              # at most this many notches
FAN_TONE_MIN_DB = 12           # a spectral peak must stand this far above its surroundings to count as fan
FAN_MIN_HZ, FAN_MAX_HZ = 120, 2500
FAN_WINDOW_SEC = 1.0           # analyse the last second of idle audio...
FAN_REFRESH_SEC = 1.0          # ...this often
FAN_QUIET_FRAMES = 20          # only analyse after this many frames (~0.6 s) with no speech
FAN_RETUNE_HZ = 4              # a tone within this of an existing notch re-uses that notch
FAN_MAX_MISSES = 3             # drop a notch after the tone is missing from this many analyses

# Spectral noise reduction, applied after the notches and band-pass. The fan's whine is not just
# a few clean tones: it is a cluster (~375-550 Hz around the 431 Hz tone, plus peaks near 215,
# 845, 1075, 1290 Hz) that notches only partly remove. This learns the fan's full spectrum from
# the first DENOISE_LEARN_SEC of audio (keep quiet when the script starts) and keeps adapting
# while nobody is speaking, then turns down every frequency in proportion to how much of it is fan.
# Adds ~32 ms of delay.
DENOISE_ENABLED = True
DENOISE_REDUCTION_DB = 20      # max cut on fan-only frequencies (higher = quieter fan, but may sound
                               # "watery" and hurt recognition; try 15-25)
DENOISE_OVERSUB = 1.5          # treat the fan as this much louder than measured (removes more of it)
DENOISE_LEARN_SEC = 0.5        # initial fan profile from the first half second
DENOISE_STUCK_SEC = 4.0        # if "speech" lasts longer than this the fan itself changed: re-learn

# VOSK
# Download "vosk-model-small-en-us-0.15" from https://alphacephei.com/vosk/models, unzip it
# next to this script (or set VOSK_MODEL_PATH). If the folder is missing, Vosk tries to
# download VOSK_MODEL_NAME automatically (needs internet).
VOSK_MODEL_PATH = os.environ.get("VOSK_MODEL_PATH", "vosk-model-small-en-us-0.15")
VOSK_MODEL_NAME = "vosk-model-small-en-us-0.15"

# DEBUG MODE (see DEBUG above)
DEBUG_OUT_DIR = os.environ.get("DEBUG_OUT_DIR", "debug_recordings")
_mic = os.environ.get("MIC_DEVICE")  # optional sounddevice input device index or name
MIC_DEVICE = int(_mic) if _mic and _mic.isdigit() else (_mic or None)

# WAKE PHRASE
WAKE_PHRASE = "oak"        # say "hey oak sit" in one go, or "hey oak", pause, then "sit"
WAKE_WINDOW_SEC = 6.0          # after a bare wake phrase, a command is accepted for this long

# ROS2 publishing (equivalent to:
#   ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq
#     "{api_id: 1009, parameter: '{}', topic: 'rt/api/sport/request', id: 0, priority: 0}")
ROS_NODE_NAME = "go2_voice_commander"
ROS_TOPIC = "/webrtc_req"
SPORT_TOPIC = "rt/api/sport/request"


# GO2 COMMANDS
# command name -> (api_id, parameter JSON string, [spoken keywords / phrases])
# Keywords must be real spoken words (write numbers out: "dance one", not "dance 1"),
# because Vosk is restricted to exactly these words. When several keywords appear in
# one utterance, the longest matching phrase wins, so "stand down" -> StandDown rather
# than StandUp via "stand".
# Deliberately omitted: flips, jumps, pounce, handstand (unsafe to trigger by voice).
COMMANDS = {
    "Damp":          (1001, "{}", ["damp", "go limp", "motors off"]),
    "BalanceStand":  (1002, "{}", ["balance stand", "balance"]),
    "StopMove":      (1003, "{}", ["stop", "halt", "freeze"]),
    "StandUp":       (1004, "{}", ["stand up", "get up", "stand"]),
    "StandDown":     (1005, "{}", ["stand down", "lie down", "lay down", "get down"]),
    "RecoveryStand": (1006, "{}", ["recover", "recovery stand", "get back up"]),
    "Sit":           (1009, "{}", ["sit", "sit down"]),
    "RiseSit":       (1010, "{}", ["rise from sit", "rise sit", "rise up"]),
    "Hello":         (1016, "{}", ["hello", "wave", "say hello"]),
    "Stretch":       (1017, "{}", ["stretch"]),
    "Content":       (1020, "{}", ["content", "happy"]),
    "Wallow":        (1021, "{}", ["wallow", "roll over"]),
    "Dance1":        (1022, "{}", ["dance", "dance one"]),
    "Dance2":        (1023, "{}", ["dance two"]),
    "Pose":          (1028, '{"data": true}', ["pose", "strike a pose"]),
    "Scrape":        (1029, "{}", ["scrape", "scratch"]),
    "FrontFlip":     (1030, "{}", ["flip", "front flip"]),
    "WiggleHips":    (1033, "{}", ["wiggle", "wiggle hips", "shake it"]),
    "FingerHeart":   (1036, "{}", ["heart", "finger heart", "show love"]),
}

# Flattened (phrase, command name) list, longest phrase first.
_PHRASES = sorted(
    ((kw, name) for name, (_, _, kws) in COMMANDS.items() for kw in kws),
    key=lambda p: (-len(p[0]), p[0]),
)
_PHRASE_PATTERNS = [
    (re.compile(r"\b" + re.escape(kw) + r"\b"), kw, name) for kw, name in _PHRASES
]


def normalize_text(text: str) -> str:
    text = text.lower().replace("\u2019", "'")
    text = re.sub(r"[^a-z0-9'\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def match_keyword(norm: str):
    """Return (command name, keyword) for the longest keyword found in norm, or (None, None)."""
    for pattern, kw, name in _PHRASE_PATTERNS:
        if pattern.search(norm):
            return name, kw
    return None, None


# WAKE PHRASE LOGIC
_WAKE_TOKENS = WAKE_PHRASE.lower().split()
_armed_until = 0.0


def _find_wake(tokens: list):
    n = len(_WAKE_TOKENS)
    for i in range(len(tokens) - n + 1):
        if tokens[i:i + n] == _WAKE_TOKENS:
            return i
    return None


def process_text(text: str, now: float = None):
    """Decide what a recognized utterance means. Returns (command name or None, explanation)."""
    global _armed_until
    now = time.monotonic() if now is None else now
    tokens = [t for t in normalize_text(text).split() if t != "unk"]
    if not tokens:
        return None, "nothing recognized"

    wake_at = _find_wake(tokens)
    if wake_at is not None:
        rest = " ".join(tokens[wake_at + len(_WAKE_TOKENS):])
        if not rest:
            _armed_until = now + WAKE_WINDOW_SEC
            return None, f"wake phrase heard, say a command within {WAKE_WINDOW_SEC:.0f}s"
        name, kw = match_keyword(rest)
        if name:
            _armed_until = 0.0
            return name, f"wake phrase + '{kw}'"
        _armed_until = now + WAKE_WINDOW_SEC
        return None, f"wake phrase heard but '{rest}' is not a command"

    if now < _armed_until:
        name, kw = match_keyword(" ".join(tokens))
        if name:
            _armed_until = 0.0
            return name, f"'{kw}' (after wake phrase)"
        return None, f"still waiting for a command, heard '{' '.join(tokens)}'"

    return None, f"ignored, no wake phrase ('{' '.join(tokens)}')"


# VOSK SETUP
def build_grammar(vosk_model) -> str:
    """Restrict Vosk to the wake phrase and command keywords: it can only output these
    words (or [unk]), so anything else - fan noise, other speech - is forced onto the
    closest of them or discarded as [unk]."""
    find_word = getattr(vosk_model, "vosk_model_find_word", None)

    def missing(phrase: str) -> list:
        if find_word is None:
            return []
        return [w for w in phrase.split() if find_word(w) < 0]

    bad = missing(WAKE_PHRASE)
    if bad:
        raise SystemExit(
            f"Wake phrase word(s) {bad} are not in this Vosk model's vocabulary. "
            f"Change WAKE_PHRASE (e.g. to a word the model knows) and restart."
        )

    phrases = [WAKE_PHRASE]
    for name, (_, _, kws) in COMMANDS.items():
        for kw in kws:
            bad = missing(kw)
            if bad:
                print(f"[vosk] skipping keyword '{kw}' ({name}): not in vocabulary: {bad}")
                continue
            phrases.append(kw)
            phrases.append(f"{WAKE_PHRASE} {kw}")
    phrases.append("[unk]")
    return json.dumps(list(dict.fromkeys(phrases)))


SetLogLevel(-1)
if os.path.isdir(VOSK_MODEL_PATH):
    vosk_model = Model(model_path=VOSK_MODEL_PATH)
else:
    vosk_model = Model(model_name=VOSK_MODEL_NAME)
recognizer = KaldiRecognizer(vosk_model, RATE, build_grammar(vosk_model))


def recognize_pcm(pcm_bytes: bytes) -> str:
    """Run one utterance through Vosk and return the recognized text."""
    texts = []
    step = 3200  # 100 ms chunks, so any mid-utterance endpoint result is collected
    for i in range(0, len(pcm_bytes), step):
        if recognizer.AcceptWaveform(pcm_bytes[i:i + step]):
            texts.append(json.loads(recognizer.Result()).get("text", ""))
    texts.append(json.loads(recognizer.FinalResult()).get("text", ""))
    recognizer.Reset()
    return " ".join(t for t in texts if t).strip()


# Warm-up so the first real command is not slower than the rest.
recognize_pcm(b"\x00" * RATE * 2)


pre_roll_frames = max(1, PRE_ROLL_MS // FRAME_MS)
end_silence_frames = max(1, END_SILENCE_MS // FRAME_MS)
max_frames = int((MAX_UTTERANCE_SEC * 1000) // FRAME_MS)

# Converts whatever the robot sends (48 kHz stereo, packed or planar) into
# 16 kHz mono signed 16-bit PCM. The resampler keeps its own state, so
# chunk boundaries between frames are handled correctly.
resampler = None if DEBUG_MIC else av.AudioResampler(format="s16", layout="mono", rate=RATE)

utterance_queue: "asyncio.Queue[bytes]" = None  # created in main()
ros_node = None
ros_pub = None
debug_session = None  # DebugSession, only in debug mode
segmenter = None      # UtteranceSegmenter, created below


# Stateful band-pass filter (state carries across chunks, so no clicks at chunk edges).
_BP_SOS = butter(4, [BANDPASS_LOW_HZ, BANDPASS_HIGH_HZ], btype="band", fs=RATE, output="sos")
_bp_zi = np.zeros((_BP_SOS.shape[0], 2))


def bandpass_pcm(pcm: bytes) -> bytes:
    global _bp_zi
    x = np.frombuffer(pcm, dtype=np.int16).astype(np.float64)
    y, _bp_zi = sosfilt(_BP_SOS, x, zi=_bp_zi)
    return np.clip(y, -32768, 32767).astype(np.int16).tobytes()


def _excess_spectrum(x: np.ndarray, nperseg: int):
    """Power spectrum (dB) minus its local +-200 Hz median: tones stand out, broadband noise does not."""
    f, P = welch(x, RATE, window="hann", nperseg=nperseg, noverlap=nperseg * 3 // 4,
                 nfft=2 * nperseg, detrend="constant")
    db = 10.0 * np.log10(P + 1e-12)
    size = max(3, int(400.0 / (f[1] - f[0])) | 1)  # odd window
    return f, db - median_filter(db, size=size, mode="nearest")


def detect_fan_tones(x: np.ndarray) -> list:
    """Frequencies (Hz) of steady tones in a stretch of idle audio, strongest FAN_MAX_TONES."""
    n = len(x)
    if n < 4096:
        return []
    f, ex = _excess_spectrum(x, min(8192, 1 << int(np.log2(n * 0.55))))
    masked = np.where((f >= FAN_MIN_HZ) & (f <= FAN_MAX_HZ), ex, -100.0)
    peaks, _ = find_peaks(masked, height=FAN_TONE_MIN_DB, distance=max(1, int(30.0 / (f[1] - f[0]))))
    if len(peaks) == 0:
        return []
    # A fan tone is steady: it must also stand out in each half of the window.
    halves = []
    for part in (x[: n // 2], x[n // 2:]):
        halves.append(_excess_spectrum(part, min(2048, 1 << int(np.log2(len(part) * 0.55)))))
    found = []
    for pk in peaks:
        hz = float(f[pk])
        if all(exh[(fh >= hz - 8) & (fh <= hz + 8)].max() >= FAN_TONE_MIN_DB * 0.6 for fh, exh in halves):
            found.append((float(ex[pk]), hz))
    found.sort(reverse=True)
    return sorted(hz for _, hz in found[:FAN_MAX_TONES])


class FanNotch:
    """Bank of notch filters on the fan's tones, found (and re-tuned) automatically from idle audio."""

    def __init__(self):
        self.freqs: list = []
        self.misses: list = []
        self.sos = None
        self.zi = None
        self.buf = collections.deque()
        self.buf_samples = 0
        self.new_samples = 0
        if FAN_NOTCH_HZ:
            self._install([float(f) for f in FAN_NOTCH_HZ], [0] * len(FAN_NOTCH_HZ), [None] * len(FAN_NOTCH_HZ))

    def _install(self, freqs, misses, keep_from):
        """keep_from[i] = index of the old notch whose filter state notch i continues, or None."""
        old_zi = self.zi
        sections = [tf2sos(*iirnotch(fq, max(fq / FAN_NOTCH_BW_HZ, 1.0), RATE)) for fq in freqs]
        zi = np.zeros((len(freqs), 2))
        for i, j in enumerate(keep_from):
            if j is not None and old_zi is not None:
                zi[i] = old_zi[j]
        self.freqs, self.misses = list(freqs), list(misses)
        self.sos = np.vstack(sections) if sections else None
        self.zi = zi if sections else None

    def process(self, x: np.ndarray, idle: bool) -> np.ndarray:
        """Remove the fan tones from x (float64 samples); learns the tones from x when idle."""
        y = x
        if self.sos is not None:
            y, self.zi = sosfilt(self.sos, x, zi=self.zi)
        if FAN_NOTCH_AUTO:
            self._observe(x, idle)
        return y

    def _observe(self, x: np.ndarray, idle: bool):
        if not idle:  # speech or noise burst: start collecting again from scratch
            self.buf.clear()
            self.buf_samples = self.new_samples = 0
            return
        self.buf.append(x)
        self.buf_samples += len(x)
        self.new_samples += len(x)
        need = int(FAN_WINDOW_SEC * RATE)
        while len(self.buf) > 1 and self.buf_samples - len(self.buf[0]) >= need:
            self.buf_samples -= len(self.buf.popleft())
        if self.buf_samples >= need and self.new_samples >= int(FAN_REFRESH_SEC * RATE):
            self.new_samples = 0
            tones = detect_fan_tones(np.concatenate(self.buf)[-need:])
            if tones:
                self._update(tones)

    def _update(self, tones):
        freqs, misses, keep = [], [], []
        used = set()
        for t in tones:
            near = [j for j, fq in enumerate(self.freqs) if j not in used and abs(fq - t) <= FAN_RETUNE_HZ]
            if near:
                j = min(near, key=lambda j: abs(self.freqs[j] - t))
                used.add(j)
                freqs.append(self.freqs[j]); misses.append(0); keep.append(j)
            else:
                freqs.append(t); misses.append(0); keep.append(None)
        for j, fq in enumerate(self.freqs):  # tones not heard this time: keep for a few rounds
            if j not in used and self.misses[j] < FAN_MAX_MISSES:
                freqs.append(fq); misses.append(self.misses[j] + 1); keep.append(j)
        freqs, misses, keep = freqs[:FAN_MAX_TONES], misses[:FAN_MAX_TONES], keep[:FAN_MAX_TONES]
        changed = sorted(freqs) != sorted(self.freqs)
        self._install(freqs, misses, keep)
        if changed:
            print("[fan] notch filters at " + ", ".join(f"{fq:.0f}" for fq in sorted(freqs)) + " Hz")


fan_notch = FanNotch()


class FanDenoiser:
    """Streaming spectral noise reduction (Wiener filter with decision-directed SNR, which keeps
    the "musical noise" artifacts low). 512-point FFT, 50% overlap, sqrt-Hann windows: with no
    noise to remove it passes audio through unchanged, only delayed by one frame (32 ms)."""

    def __init__(self, n_fft: int = 512):
        self.n = n_fft
        self.hop = n_fft // 2
        self.win = np.sqrt(np.hanning(n_fft + 1)[:-1])
        self.floor = 10 ** (-DENOISE_REDUCTION_DB / 20)
        self.learn_frames = max(1, int(DENOISE_LEARN_SEC * RATE / self.hop))
        self.noise = None        # fan power spectrum
        self.n_learned = 0
        self.recent = collections.deque(maxlen=int(DENOISE_STUCK_SEC * RATE / self.hop))  # level vs fan
        self.prev_gain = None
        self.prev_gamma = None
        self.inbuf = np.zeros(0)
        self.frame = np.zeros(n_fft)
        self.ola = np.zeros(n_fft)

    def process(self, x: np.ndarray, may_learn: bool) -> np.ndarray:
        """x: float64 samples. Returns denoised samples (delayed; count can differ per call)."""
        self.inbuf = np.concatenate([self.inbuf, x])
        out = []
        while len(self.inbuf) >= self.hop:
            hop, self.inbuf = self.inbuf[: self.hop], self.inbuf[self.hop:]
            self.frame = np.concatenate([self.frame[self.hop:], hop])
            spec = np.fft.rfft(self.frame * self.win)
            power = np.abs(spec) ** 2
            self._learn(power, may_learn)
            gamma = power / (DENOISE_OVERSUB * self.noise + 1e-12)        # a-posteriori SNR
            ml = np.maximum(gamma - 1.0, 0.0)
            if self.prev_gain is None:
                xi = ml
            else:                                                       # a-priori SNR
                xi = 0.98 * self.prev_gain ** 2 * self.prev_gamma + 0.02 * ml
            gain = np.maximum(xi / (1.0 + xi), self.floor)
            self.prev_gain, self.prev_gamma = gain, gamma
            self.ola += np.fft.irfft(spec * gain, self.n) * self.win
            out.append(self.ola[: self.hop].copy())
            self.ola = np.concatenate([self.ola[self.hop:], np.zeros(self.hop)])
        return np.concatenate(out) if out else np.zeros(0)

    def _learn(self, power: np.ndarray, may_learn: bool):
        if self.noise is None:
            self.noise = power.copy()
            self.n_learned = 1
            return
        if self.n_learned < self.learn_frames:          # initial profile: plain average
            self.n_learned += 1
            self.noise += (power - self.noise) / self.n_learned
            return
        level = power.sum() / (self.noise.sum() + 1e-12)
        fan_only = level < 2.5
        self.recent.append(level)
        if len(self.recent) == self.recent.maxlen and np.median(self.recent) > 2.5:
            # Mostly louder than the fan for longer than any command: the fan itself got
            # louder (a command is short, so it cannot move the median). Re-learn, even
            # mid-"utterance".
            self.noise = 0.9 * self.noise + 0.1 * power
        elif fan_only and may_learn:                    # looks like fan only: adapt slowly
            self.noise = 0.95 * self.noise + 0.05 * power


fan_denoiser = FanDenoiser() if DENOISE_ENABLED else None


def apply_filters(pcm: bytes) -> bytes:
    """Fan-tone notches, band-pass, then spectral noise reduction. int16 PCM in, int16 PCM out."""
    global _bp_zi
    x = np.frombuffer(pcm, dtype=np.int16).astype(np.float64)
    idle = segmenter is not None and segmenter.quiet_frames >= FAN_QUIET_FRAMES
    x = fan_notch.process(x, idle)
    y, _bp_zi = sosfilt(_BP_SOS, x, zi=_bp_zi)
    if fan_denoiser is not None:
        y = fan_denoiser.process(y, may_learn=segmenter is None or not segmenter.recording)
    return np.clip(y, -32768, 32767).astype(np.int16).tobytes()


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
    pcm = b"".join(chunks)
    if not pcm:
        return b""
    filtered = apply_filters(pcm)
    if debug_session is not None:
        debug_session.write(pcm, filtered)
    return filtered


class UtteranceSegmenter:
    """Same VAD state machine as the mic version, fed by arbitrary-sized PCM chunks."""

    def __init__(self):
        self.vad = webrtcvad.Vad(VAD_MODE)
        self.buf = bytearray()
        self.pre_roll = collections.deque(maxlen=pre_roll_frames)
        self.recording = False
        self.silence_run = 0
        self.utterance_frames: list[bytes] = []
        self.noise_floor = None   # running estimate of fan RMS
        self.voiced_count = 0
        self.first_voiced = 0     # index in utterance_frames of the first voiced frame
        self.last_voiced = 0      # index just past the last voiced frame
        self.peak_ratio = 0.0     # loudest frame vs noise floor during the utterance
        self.voiced_hist = collections.deque(maxlen=START_WINDOW_FRAMES)
        self.ratio_hist = collections.deque(maxlen=START_WINDOW_FRAMES)
        self.dropped_count = 0
        self.quiet_frames = 0     # consecutive frames with no speech (used to learn the fan tones)

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
        samples = np.frombuffer(frame, dtype=np.int16).astype(np.float32)
        rms = float(np.sqrt(np.mean(samples * samples)))
        if self.noise_floor is None:
            self.noise_floor = max(rms, 1.0)

        # Speech = VAD says speech AND the frame is clearly louder than the fan.
        # Starting needs a louder frame than continuing (hysteresis), so a phrase that
        # begins strongly and trails off quietly is not cut in the middle.
        ratio = rms / self.noise_floor
        gate = GATE_CONTINUE_RATIO if self.recording else GATE_START_RATIO
        voiced = ratio > gate and self.vad.is_speech(frame, RATE)
        self.quiet_frames = 0 if (self.recording or voiced) else self.quiet_frames + 1
        self.pre_roll.append(frame)

        if not self.recording:
            # Track the noise floor only while idle: falls fast, rises slowly.
            if rms < self.noise_floor:
                self.noise_floor = 0.9 * self.noise_floor + 0.1 * rms
            else:
                self.noise_floor = 0.995 * self.noise_floor + 0.005 * rms
            self.noise_floor = max(self.noise_floor, 1.0)

            self.ratio_hist.append(ratio)
            self.voiced_hist.append(voiced)

            if sum(self.voiced_hist) >= START_SPEECH_FRAMES:
                self.recording = True
                self.silence_run = 0
                self.utterance_frames = list(self.pre_roll)
                self.voiced_count = sum(self.voiced_hist)
                # the history covers the last len(voiced_hist) frames of utterance_frames
                offset = len(self.utterance_frames) - len(self.voiced_hist)
                self.first_voiced = offset + list(self.voiced_hist).index(True)
                self.last_voiced = offset + len(self.voiced_hist) - list(reversed(self.voiced_hist)).index(True)
                self.peak_ratio = max(self.ratio_hist)
                self.voiced_hist.clear()
                print("Speech detected...")
            return None

        # recording mode
        self.utterance_frames.append(frame)
        self.peak_ratio = max(self.peak_ratio, ratio)

        if voiced:
            self.silence_run = 0
            self.voiced_count += 1
            self.last_voiced = len(self.utterance_frames)
        else:
            self.silence_run += 1

        too_long = len(self.utterance_frames) >= max_frames
        enough_silence = self.silence_run >= end_silence_frames

        if enough_silence or too_long:
            pcm_bytes = b"".join(self.utterance_frames)
            span_ms = (self.last_voiced - self.first_voiced) * FRAME_MS
            stats = (
                f"speech lasted {span_ms} ms, {self.voiced_count} voiced frames, "
                f"peak {self.peak_ratio:.1f}x the noise floor"
            )
            self.recording = False
            self.silence_run = 0
            self.voiced_count = 0
            self.utterance_frames = []
            self.voiced_hist.clear()
            self.ratio_hist.clear()
            if span_ms < MIN_SPEECH_MS:
                self.dropped_count += 1
                print(f"[filter] too short, ignored: {stats} (minimum {MIN_SPEECH_MS} ms)")
                if DEBUG and debug_session is not None:
                    path = os.path.join(
                        DEBUG_OUT_DIR, f"{debug_session.name}_dropped_{self.dropped_count:03d}.wav"
                    )
                    write_wav(path, pcm_bytes)
                    print(f"[debug] saved the ignored clip: {path}")
                return None
            if DEBUG:
                print(f"[debug] utterance: {stats}")
            return pcm_bytes

        return None


segmenter = UtteranceSegmenter()


def publish_command(name: str):
    """Publish one WebRtcReq for the named command (same as `ros2 topic pub --once`)."""
    api_id, parameter, _ = COMMANDS[name]
    msg = WebRtcReq()
    msg.id = 0
    msg.topic = SPORT_TOPIC
    msg.api_id = api_id
    msg.parameter = parameter
    msg.priority = 0
    ros_pub.publish(msg)
    print(f"[ros2] published {name} (api_id={api_id}) to {ROS_TOPIC}")


def handle_transcript(text: str):
    command, why = process_text(text)
    if command is None:
        print(f"[wake] {why}")
        return
    print(f"[command] {command} [{why}]")
    if DEBUG:
        return  # debug mode never publishes
    try:
        publish_command(command)
    except Exception as e:
        print(f"[publish error] {e}", file=sys.stderr)


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


# Recognition runs in a worker thread so it never stalls the WebRTC event loop
# (which would drop the connection).
async def transcription_worker():
    loop = asyncio.get_running_loop()
    count = 0
    while True:
        pcm_bytes = await utterance_queue.get()
        if DEBUG:
            count += 1
            try:
                await loop.run_in_executor(
                    None, handle_debug_utterance, pcm_bytes, count, debug_session.name
                )
            except Exception as e:
                print(f"[debug error] {e}", file=sys.stderr)
            continue
        try:
            t0 = time.perf_counter()
            text = await loop.run_in_executor(None, recognize_pcm, pcm_bytes)
            print(f"Heard: {text!r} (recognition took {time.perf_counter() - t0:.2f}s)")
            if text:
                await loop.run_in_executor(None, handle_transcript, text)
        except Exception as e:
            print(f"[recognize error] {e}", file=sys.stderr)
        print("\nListening again...\n")


async def main():
    global utterance_queue, ros_node, ros_pub
    utterance_queue = asyncio.Queue()

    if DEBUG:
        start_debug_session()
        print("DEBUG MODE: using the robot's microphone only. No ROS, no commands will be sent.")
        print(f"Saving recordings to '{DEBUG_OUT_DIR}/' (session {debug_session.name}).")
    else:
        # Create the publisher up front so ROS discovery finishes before the first command.
        rclpy.init()
        ros_node = rclpy.create_node(ROS_NODE_NAME)
        ros_pub = ros_node.create_publisher(WebRtcReq, ROS_TOPIC, 10)

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

        print(f"Listening via Go2 microphone... say '{WAKE_PHRASE}' followed by a command (e.g. '{WAKE_PHRASE} sit').")
        print("Press Ctrl+C to stop.\n")

        # Run until interrupted
        await asyncio.Event().wait()

    except ValueError as e:
        logging.error(f"Error in WebRTC connection: {e}")


class DebugSession:
    """Continuous recordings of the whole debug session: raw, and after the filters."""

    def __init__(self):
        os.makedirs(DEBUG_OUT_DIR, exist_ok=True)
        self.name = time.strftime("%Y%m%d_%H%M%S")
        self.raw_path = os.path.join(DEBUG_OUT_DIR, f"{self.name}_session_raw.wav")
        self.filtered_path = os.path.join(DEBUG_OUT_DIR, f"{self.name}_session_filtered.wav")
        self._raw = self._open(self.raw_path)
        self._filtered = self._open(self.filtered_path)

    @staticmethod
    def _open(path: str):
        wf = wave.open(path, "wb")
        wf.setnchannels(1)
        wf.setsampwidth(2)  # int16
        wf.setframerate(RATE)
        return wf

    def write(self, raw_pcm: bytes, filtered_pcm: bytes):
        self._raw.writeframes(raw_pcm)
        self._filtered.writeframes(filtered_pcm)

    def close(self):
        for wf in (self._raw, self._filtered):
            try:
                wf.close()
            except Exception:
                pass


def start_debug_session():
    global debug_session
    if debug_session is None:
        debug_session = DebugSession()


def close_debug_session():
    if debug_session is not None:
        debug_session.close()
        print(f"[debug] session recordings saved:\n  {debug_session.raw_path}\n  {debug_session.filtered_path}")


def write_wav(path: str, pcm_bytes: bytes):
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # int16
        wf.setframerate(RATE)
        wf.writeframes(pcm_bytes)


def handle_debug_utterance(pcm_bytes: bytes, n: int, session: str):
    """Save the filtered utterance as a WAV and show what Vosk heard and how it is interpreted."""
    path = os.path.join(DEBUG_OUT_DIR, f"{session}_{n:03d}.wav")
    write_wav(path, pcm_bytes)
    seconds = len(pcm_bytes) / 2 / RATE

    t0 = time.perf_counter()
    text = recognize_pcm(pcm_bytes)
    took = time.perf_counter() - t0
    command, why = process_text(text)

    print(f"[debug] saved {path} ({seconds:.2f}s of filtered audio, noise floor ~{segmenter.noise_floor:.0f})")
    print(f"[debug] Vosk transcript: {text!r} (recognition took {took:.2f}s)")
    if command:
        api_id = COMMANDS[command][0]
        print(f"[debug] interpreted: {command} (api_id={api_id}) [{why}] -> would publish, but debug mode sends nothing")
    else:
        print(f"[debug] interpreted: no command [{why}]")
    print()


def debug_main():
    """Microphone -> same band-pass filter, gate and VAD -> WAV file + Vosk transcript. No robot."""
    start_debug_session()
    session = debug_session.name
    audio_q: "queue.Queue[bytes]" = queue.Queue()

    def on_audio(indata, frames, time_info, status):
        if status:
            print(f"[mic] {status}", file=sys.stderr)
        audio_q.put(bytes(indata))

    print("DEBUG MODE (mic): not connecting to the robot. Using this computer's microphone.")
    print(f"Saving recordings to '{DEBUG_OUT_DIR}/' (session {session}). Say '{WAKE_PHRASE}' + a command.")
    print("Press Ctrl+C to stop.\n")

    count = 0
    with sd.RawInputStream(
        samplerate=RATE,
        channels=1,
        dtype="int16",
        blocksize=FRAME_SAMPLES,
        device=MIC_DEVICE,
        callback=on_audio,
    ):
        while True:
            try:
                chunk = audio_q.get(timeout=0.5)
            except queue.Empty:
                continue
            filtered = apply_filters(chunk)
            debug_session.write(chunk, filtered)
            for utt in segmenter.feed(filtered):
                count += 1
                try:
                    handle_debug_utterance(utt, count, session)
                except Exception as e:
                    print(f"[debug error] {e}", file=sys.stderr)


def shutdown_ros():
    if DEBUG:
        return  # ROS was never started
    try:
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    try:
        if DEBUG_MIC:
            debug_main()
        else:
            asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        close_debug_session()
        shutdown_ros()
    sys.exit(0)
