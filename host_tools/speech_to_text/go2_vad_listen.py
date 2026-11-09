import asyncio
import collections
import logging
import os
import re
import sys
import time
from difflib import SequenceMatcher

import av
import numpy as np
import rclpy
import webrtcvad
from scipy.ndimage import median_filter
from scipy.signal import butter, find_peaks, iirnotch, sosfilt, tf2sos, welch
from faster_whisper import WhisperModel
from go2_interfaces.msg import WebRtcReq
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
START_SPEECH_FRAMES = 3        # start an utterance when this many of the last START_WINDOW_FRAMES
START_WINDOW_FRAMES = 5        # frames are voiced (tolerates the VAD flickering on/off)
END_SILENCE_MS = 350           # stop after this much silence (lower = faster, but may cut off slow speakers)
MAX_UTTERANCE_SEC = 3          # hard cap: commands are 1-3 words, so never wait longer than this
PRE_ROLL_MS = 300              # keep some audio from before speech starts

# FAN / NOISE REJECTION
BANDPASS_LOW_HZ = 300          # cuts fan rumble/hum below this
BANDPASS_HIGH_HZ = 3800        # cuts hiss above this (speech intelligibility lives in between)
GATE_START_RATIO = 2.5         # to START an utterance, a frame must be this many times louder than the fan...
GATE_CONTINUE_RATIO = 1.3      # ...but once speech has started, quieter frames (down to this) still count
MIN_SPEECH_MS = 240            # discard utterances where speech lasted less than this: clicks, bursts
NOISE_FLOOR_SEC = 3.0          # the fan level is the MEDIAN loudness of the last few seconds of idle audio
                               # (tracking the quietest moments instead made the fan's normal pulsing
                               # look like speech and kept "utterances" open until the 3 s cap)
END_DROP_RATIO = 0.1           # speech has ended once the level falls below this fraction of the
                               # utterance's loudest moment (20 dB down), even if the fan is above the
                               # continue gate: you don't wait for the cap after a loud command
FAN_CLIP_PEAK = 4.0            # an utterance that hits the cap without ever getting this loud was the fan:
                               # raise the fan level estimate from it so it doesn't keep happening
MAX_QUEUE_AGE_SEC = 4.0        # utterances waiting longer than this for Whisper are dropped (stale)

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
# Whisper copes with noise well but not with the "watery" sound of heavy noise reduction, so by
# default the noise-reduced audio is only used to DETECT speech (start/end, far fewer false
# triggers), while Whisper hears the same moment with only the fan tones notched out.
WHISPER_HEARS_DENOISED = False

MAX_WORDS = 6                  # ignore transcripts longer than this (fan "hallucinations" tend to be long)
MIN_AVG_LOGPROB = -1.5         # drop Whisper segments it is not confident about
MAX_NO_SPEECH_PROB = 0.6       # ...or that it thinks are not speech (used together with logprob < -1.0)

WHISPER_MODEL = "tiny.en"      # English-only tiny model: much faster than "base"
WHISPER_DEVICE = "auto"        # "auto" = NVIDIA GPU if one is usable (much faster), else CPU
WHISPER_THREADS = os.cpu_count() or 4
WHISPER_MAX_TOKENS = 16        # commands are a few words; stops slow, long "hallucinated" transcripts

# Biases Whisper toward the command vocabulary (improves accuracy of the small model).
WHISPER_PROMPT = "Robot dog commands: sit, stand up, stand down, lie down, hello, stretch, dance, stop."

# ROS2 publishing (equivalent to:
#   ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq
#     "{api_id: 1009, parameter: '{}', topic: 'rt/api/sport/request', id: 0, priority: 0}")
ROS_NODE_NAME = "go2_voice_commander"
ROS_TOPIC = "/webrtc_req"
SPORT_TOPIC = "rt/api/sport/request"


# GO2 COMMANDS
# command name -> (api_id, parameter JSON string, [spoken keywords / phrases])
# Keywords are matched as whole words/phrases (case-insensitive). When several
# keywords appear in one utterance, the longest matching phrase wins, so
# "stand down" -> StandDown rather than StandUp via "stand".
# Deliberately omitted: flips, jumps, pounce, handstand (unsafe to trigger by
# voice, since Whisper can mis-hear).
COMMANDS = {
    "Damp":          (1001, "{}", ["damp", "go limp", "relax motors", "motors off"]),
    "BalanceStand":  (1002, "{}", ["balance stand", "balance"]),
    "StopMove":      (1003, "{}", ["stop", "halt", "freeze"]),
    "StandUp":       (1004, "{}", ["stand up", "get up", "stand"]),
    "StandDown":     (1005, "{}", ["stand down", "lie down", "lay down", "get down"]),
    "RecoveryStand": (1006, "{}", ["recover", "recovery stand", "get back up"]),
    "Sit":           (1009, "{}", ["sit", "sit down"]),
    "RiseSit":       (1010, "{}", ["rise from sit", "rise sit", "rise up"]),
    "Hello":         (1016, "{}", ["hello", "hi", "wave", "say hello"]),
    "Stretch":       (1017, "{}", ["stretch"]),
    "Content":       (1020, "{}", ["content", "happy"]),
    "Wallow":        (1021, "{}", ["wallow", "roll over", "roll"]),
    "Dance1":        (1022, "{}", ["dance", "dance one", "dance 1"]),
    "Dance2":        (1023, "{}", ["dance two", "dance 2"]),
    "Pose":          (1028, '{"data": true}', ["pose", "strike a pose"]),
    "Scrape":        (1029, "{}", ["scrape", "scratch"]),
    "WiggleHips":    (1033, "{}", ["wiggle", "wiggle hips", "shake it"]),
    "FingerHeart":   (1036, "{}", ["heart", "finger heart", "show love"]),
}

# FUZZY (CLOSEST-SOUNDING) MATCHING
# If no keyword matches exactly, pick the command whose keyword sounds closest to what
# Whisper heard (e.g. "sat"/"set" -> Sit, "stan up" -> StandUp).
FUZZY_ENABLED = True
FUZZY_THRESHOLD = 0.78         # 0-1 similarity needed (higher = stricter, fewer false triggers)
FUZZY_MARGIN = 0.05            # best command must beat the runner-up by this much, else ignored
FUZZY_MIN_KEYWORD_LEN = 3      # very short keywords ("hi") are only matched exactly
FUZZY_EXACT_ONLY = {"Damp"}    # commands that must never be triggered by a fuzzy match

NEGATIONS = {"not", "no", "never", "dont", "don't", "cant", "can't", "wont", "won't"}

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


_PHONETIC_RULES = (
    ("ph", "f"), ("ck", "k"), ("kn", "n"), ("wr", "r"), ("gh", ""), ("wh", "w"),
    ("ce", "se"), ("ci", "si"), ("cy", "sy"), ("c", "k"), ("q", "k"), ("x", "ks"), ("z", "s"),
)


def phonetic_key(word: str) -> str:
    """Rough sound-alike key: spelling variants and vowel changes collapse together."""
    w = re.sub(r"[^a-z]", "", word.lower())
    if not w:
        return ""
    for a, b in _PHONETIC_RULES:
        w = w.replace(a, b)
    if not w:
        return ""
    w = w[0] + re.sub(r"[aeiouyhw]", "", w[1:])  # keep first letter, drop later vowels
    return re.sub(r"(.)\1+", r"\1", w)          # collapse doubled letters


def similarity(words_a: list, words_b: list) -> float:
    """0-1 score blending spelling similarity and sound similarity."""
    raw_a, raw_b = " ".join(words_a), " ".join(words_b)
    key_a = " ".join(k for k in (phonetic_key(w) for w in words_a) if k)
    key_b = " ".join(k for k in (phonetic_key(w) for w in words_b) if k)
    raw = SequenceMatcher(None, raw_a, raw_b).ratio()
    phon = SequenceMatcher(None, key_a, key_b).ratio() if key_a and key_b else 0.0
    return 0.5 * raw + 0.5 * phon


def _negated(tokens: list, start: int) -> bool:
    return any(w in NEGATIONS for w in tokens[max(0, start - 2):start])


def exact_match(norm: str):
    for pattern, kw, name in _PHRASE_PATTERNS:
        for m in pattern.finditer(norm):
            prev_words = norm[: m.start()].split()[-2:]
            if any(w in NEGATIONS for w in prev_words):
                continue  # e.g. "don't sit"
            return name, f"exact '{kw}'"
    return None, None


def fuzzy_match(norm: str):
    tokens = norm.split()
    best = {}  # command name -> (score, keyword, heard)
    for name, (_, _, kws) in COMMANDS.items():
        if name in FUZZY_EXACT_ONLY:
            continue
        for kw in kws:
            if len(kw) < FUZZY_MIN_KEYWORD_LEN or any(ch.isdigit() for ch in kw):
                continue
            kw_words = kw.split()
            k = len(kw_words)
            tolerance = max(1, int(0.3 * len(kw)))  # heard text must be a similar length
            for i in range(len(tokens) - k + 1):
                if _negated(tokens, i):
                    continue
                window = tokens[i:i + k]
                if abs(len(" ".join(window)) - len(kw)) > tolerance:
                    continue
                score = similarity(window, kw_words)
                if score > best.get(name, (0.0,))[0]:
                    best[name] = (score, kw, " ".join(window))
    ranked = sorted(best.items(), key=lambda kv: -kv[1][0])
    if not ranked or ranked[0][1][0] < FUZZY_THRESHOLD:
        return None, None
    if len(ranked) > 1 and ranked[0][1][0] - ranked[1][1][0] < FUZZY_MARGIN:
        return None, f"ambiguous between {ranked[0][0]} and {ranked[1][0]}"
    name, (score, kw, heard) = ranked[0]
    return name, f"fuzzy heard '{heard}' ~ '{kw}' ({score:.2f})"


def find_command(text: str):
    """Return (command name or None, explanation)."""
    norm = normalize_text(text)
    if not norm:
        return None, None
    name, why = exact_match(norm)
    if name is None and FUZZY_ENABLED:
        name, why = fuzzy_match(norm)
    return name, why


def match_command(text: str):
    """Return the command name matched in the transcript, or None."""
    return find_command(text)[0]


# SETUP
def _load_whisper():
    devices = ["cuda", "cpu"] if WHISPER_DEVICE == "auto" else [WHISPER_DEVICE]
    for dev in devices:
        try:
            m = WhisperModel(
                WHISPER_MODEL,
                device=dev,
                compute_type="float16" if dev == "cuda" else "int8",
                cpu_threads=WHISPER_THREADS,
            )
            print(f"[whisper] '{WHISPER_MODEL}' running on {dev.upper()}")
            return m
        except Exception as e:
            if dev == devices[-1]:
                raise
            print(f"[whisper] {dev} not available ({e.__class__.__name__}), trying the next option")


model = _load_whisper()

pre_roll_frames = max(1, PRE_ROLL_MS // FRAME_MS)
end_silence_frames = max(1, END_SILENCE_MS // FRAME_MS)
max_frames = int((MAX_UTTERANCE_SEC * 1000) // FRAME_MS)

# Converts whatever the robot sends (48 kHz stereo, packed or planar) into
# 16 kHz mono signed 16-bit PCM. The resampler keeps its own state, so
# chunk boundaries between frames are handled correctly.
resampler = av.AudioResampler(format="s16", layout="mono", rate=RATE)

utterance_queue: "asyncio.Queue" = None  # created in main()
ros_node = None
ros_pub = None
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


class _Delay:
    """Delays a stream by a fixed number of samples (keeps the notched audio Whisper hears in
    step with the noise-reduced audio used for detection, which is 256 samples late)."""

    def __init__(self, samples: int):
        self.buf = np.zeros(samples)

    def take(self, x: np.ndarray, n_out: int) -> np.ndarray:
        self.buf = np.concatenate([self.buf, x])
        out, self.buf = self.buf[:n_out], self.buf[n_out:]
        return out


_whisper_delay = _Delay(fan_denoiser.hop if fan_denoiser is not None else 0)


def _to_pcm(y: np.ndarray) -> bytes:
    return np.clip(y, -32768, 32767).astype(np.int16).tobytes()


def apply_filters(pcm: bytes):
    """Returns (detect_pcm, whisper_pcm), same length and time-aligned:
    detect = notches + band-pass + noise reduction (decides where speech starts and ends),
    whisper = notches + band-pass only (what Whisper transcribes, unless WHISPER_HEARS_DENOISED)."""
    global _bp_zi
    x = np.frombuffer(pcm, dtype=np.int16).astype(np.float64)
    idle = segmenter is not None and segmenter.quiet_frames >= FAN_QUIET_FRAMES
    x = fan_notch.process(x, idle)
    y, _bp_zi = sosfilt(_BP_SOS, x, zi=_bp_zi)
    if fan_denoiser is None:
        p = _to_pcm(y)
        return p, p
    clean = fan_denoiser.process(y, may_learn=segmenter is None or not segmenter.recording)
    plain = _whisper_delay.take(y, len(clean))
    clean_pcm = _to_pcm(clean)
    return clean_pcm, (clean_pcm if WHISPER_HEARS_DENOISED else _to_pcm(plain))


def frame_to_pcm16k_mono(frame):
    out = resampler.resample(frame)
    if out is None:
        return b"", b""
    if not isinstance(out, (list, tuple)):  # older PyAV returns a single frame
        out = [out]
    chunks = []
    for f in out:
        if f is None:
            continue
        chunks.append(f.to_ndarray().astype(np.int16).tobytes())
    pcm = b"".join(chunks)
    if not pcm:
        return b"", b""
    return apply_filters(pcm)


class Utterance:
    """One detected utterance plus timestamps for the latency printout."""

    def __init__(self, pcm: bytes, speech_end: float, closed: float):
        self.pcm = pcm
        self.speech_end = speech_end  # when you stopped talking
        self.closed = closed          # when the end-of-speech silence wait finished


class UtteranceSegmenter:
    """Finds utterances. Decides on the noise-reduced audio, but keeps the matching stretch of
    the audio Whisper should hear. Start needs a louder frame than continuing (hysteresis)."""

    def __init__(self):
        self.vad = webrtcvad.Vad(VAD_MODE)
        self.buf = bytearray()
        self.wbuf = bytearray()
        self.pre_roll = collections.deque(maxlen=pre_roll_frames)
        self.recording = False
        self.silence_run = 0
        self.utterance_frames: list[bytes] = []
        self.noise_floor = None   # running estimate of the fan level (after filtering)
        self.voiced_hist = collections.deque(maxlen=START_WINDOW_FRAMES)
        self.first_voiced = 0
        self.last_voiced = 0
        self.last_voiced_time = 0.0
        self.quiet_frames = 0     # consecutive frames with no speech (used to learn the fan tones)
        self.idle_rms = collections.deque(maxlen=max(10, int(NOISE_FLOOR_SEC * 1000 / FRAME_MS)))
        self.utt_rms: list = []   # detect-stream loudness of each frame of the current utterance
        self.peak_ratio = 0.0

    def feed(self, detect_pcm: bytes, whisper_pcm: bytes) -> list:
        utterances = []
        self.buf.extend(detect_pcm)
        self.wbuf.extend(whisper_pcm)
        while len(self.buf) >= FRAME_BYTES and len(self.wbuf) >= FRAME_BYTES:
            frame = bytes(self.buf[:FRAME_BYTES])
            wframe = bytes(self.wbuf[:FRAME_BYTES])
            del self.buf[:FRAME_BYTES]
            del self.wbuf[:FRAME_BYTES]
            utt = self._process_frame(frame, wframe)
            if utt is not None:
                utterances.append(utt)
        return utterances

    def _process_frame(self, frame: bytes, wframe: bytes):
        samples = np.frombuffer(frame, dtype=np.int16).astype(np.float32)
        rms = float(np.sqrt(np.mean(samples * samples)))
        if self.noise_floor is None:
            self.noise_floor = max(rms, 1.0)

        ratio = rms / self.noise_floor
        if self.recording:
            gate = max(GATE_CONTINUE_RATIO, END_DROP_RATIO * self.peak_ratio)
        else:
            gate = GATE_START_RATIO
        voiced = ratio > gate and self.vad.is_speech(frame, RATE)
        self.quiet_frames = 0 if (self.recording or voiced) else self.quiet_frames + 1
        self.pre_roll.append(wframe)
        now = time.monotonic()

        if not self.recording:
            # Fan level = median loudness of recent idle audio (robust to the fan's pulsing).
            self.idle_rms.append(rms)
            self.noise_floor = max(float(np.median(self.idle_rms)), 1.0)

            self.voiced_hist.append(voiced)
            if sum(self.voiced_hist) >= START_SPEECH_FRAMES:
                self.recording = True
                self.silence_run = 0
                self.peak_ratio = ratio
                self.utt_rms = [rms]
                self.utterance_frames = list(self.pre_roll)
                offset = len(self.utterance_frames) - len(self.voiced_hist)
                self.first_voiced = offset + list(self.voiced_hist).index(True)
                self.last_voiced = offset + len(self.voiced_hist) - list(reversed(self.voiced_hist)).index(True)
                self.last_voiced_time = now
                self.voiced_hist.clear()
                print("Speech detected...")
            return None

        # recording mode
        self.utterance_frames.append(wframe)
        self.utt_rms.append(rms)
        if voiced:
            self.peak_ratio = max(self.peak_ratio, ratio)
            self.silence_run = 0
            self.last_voiced = len(self.utterance_frames)
            self.last_voiced_time = now
        else:
            self.silence_run += 1

        too_long = len(self.utterance_frames) >= max_frames
        enough_silence = self.silence_run >= end_silence_frames
        if not (enough_silence or too_long):
            return None

        # Keep only up to ~150 ms after the last speech: the trailing silence wait is
        # not needed by Whisper and would only make it slower.
        keep = min(len(self.utterance_frames), self.last_voiced + max(1, 150 // FRAME_MS))
        pcm_bytes = b"".join(self.utterance_frames[:keep])
        span_ms = (self.last_voiced - self.first_voiced) * FRAME_MS
        self.recording = False
        self.silence_run = 0
        self.utterance_frames = []
        self.voiced_hist.clear()
        if too_long and self.peak_ratio < FAN_CLIP_PEAK:
            # Hit the cap without ever being clearly louder than the fan: it WAS the fan.
            # Count it as background so the fan level estimate catches up, and skip Whisper.
            self.idle_rms.extend(self.utt_rms)
            self.noise_floor = max(float(np.median(self.idle_rms)), 1.0)
            print(f"[filter] ignored: fan noise held the detector open (peak only {self.peak_ratio:.1f}x)")
            return None
        if span_ms < MIN_SPEECH_MS:
            print(f"[filter] too short, ignored (speech lasted {span_ms} ms)")
            return None
        return Utterance(pcm_bytes, self.last_voiced_time, now)


segmenter = UtteranceSegmenter()


def _supported_extra_args() -> dict:
    """max_new_tokens only exists in newer faster-whisper versions."""
    import inspect
    try:
        params = inspect.signature(model.transcribe).parameters
    except (TypeError, ValueError):
        return {}
    return {"max_new_tokens": WHISPER_MAX_TOKENS} if "max_new_tokens" in params else {}


_EXTRA_ARGS = _supported_extra_args()


def transcribe_pcm(pcm_bytes: bytes) -> str:
    # Pass raw float32 samples directly (skips WAV encode/decode).
    audio = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32768.0
    segments, info = model.transcribe(
        audio,
        language="en",                    # skip language detection
        beam_size=1,
        best_of=1,
        temperature=0.0,                  # no fallback re-decodes
        without_timestamps=True,
        condition_on_previous_text=False,
        initial_prompt=WHISPER_PROMPT,
        vad_filter=False,                 # we already did VAD ourselves
        **_EXTRA_ARGS,
    )
    kept = []
    for seg in segments:
        if seg.no_speech_prob > MAX_NO_SPEECH_PROB and seg.avg_logprob < -1.0:
            continue  # Whisper thinks this is not speech
        if seg.avg_logprob < MIN_AVG_LOGPROB:
            continue  # low confidence, likely noise mis-heard as words
        kept.append(seg.text.strip())
    return " ".join(kept).strip()


# Warm-up: the first transcription is much slower (lazy init), so do it now.
transcribe_pcm(np.zeros(RATE, dtype=np.int16).tobytes())


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


DRY_RUN = len(sys.argv) > 1  # any CLI argument = show the match, don't publish


def handle_transcript(text: str):
    """Returns the command name if one was sent (or would be, in dry-run mode)."""
    if len(text.split()) > MAX_WORDS:
        print(f"[command] ignored, too many words ({len(text.split())})")
        return None
    command, why = find_command(text)
    if command is None:
        print(f"[command] no keyword matched{f' ({why})' if why else ''}")
        return None
    print(f"[command] matched: {command} [{why}]")
    if DRY_RUN:
        print(f"[dry run] would send: {command}")
        return command
    try:
        publish_command(command)
    except Exception as e:
        print(f"[publish error] {e}", file=sys.stderr)
        return None
    return command


# Called by the Unitree WebRTC driver for every incoming audio frame.
async def recv_audio_stream(frame):
    try:
        detect_pcm, whisper_pcm = frame_to_pcm16k_mono(frame)
        if not detect_pcm:
            return
        for utt in segmenter.feed(detect_pcm, whisper_pcm):
            print("End of speech. Transcribing...")
            utterance_queue.put_nowait(utt)
    except Exception as e:
        print(f"[audio error] {e}", file=sys.stderr)


# Transcription runs in a worker thread so the blocking Whisper call
# never stalls the WebRTC event loop (which would drop the connection).
async def transcription_worker():
    loop = asyncio.get_running_loop()
    pending: list = []  # oldest ... newest
    while True:
        if not pending:
            pending.append(await utterance_queue.get())
        while not utterance_queue.empty():
            pending.append(utterance_queue.get_nowait())
        # Never work through a backlog: drop anything that has waited too long, and do the
        # newest first (it is the one you most likely just said).
        now = time.monotonic()
        fresh = [u for u in pending if now - u.closed <= MAX_QUEUE_AGE_SEC]
        if len(fresh) < len(pending):
            print(f"[queue] dropped {len(pending) - len(fresh)} stale utterance(s)")
        pending = fresh
        if not pending:
            continue
        utt = pending.pop()
        command = None
        try:
            t_start = time.monotonic()
            text = await loop.run_in_executor(None, transcribe_pcm, utt.pcm)
            t_text = time.monotonic()
            print(f"Transcript: {text!r}")
            command = handle_transcript(text) if text else None
            t_done = time.monotonic()
            # Where the time went, from the moment you stopped talking:
            print(
                f"[timing] {t_done - utt.speech_end:.2f}s total = "
                f"{utt.closed - utt.speech_end:.2f}s end-of-speech wait + "
                f"{t_start - utt.closed:.2f}s queued + "
                f"{t_text - t_start:.2f}s Whisper ({len(utt.pcm) / 2 / RATE:.1f}s of audio) + "
                f"{t_done - t_text:.2f}s matching/publish"
                + ("" if command else "  (no command)")
            )
        except Exception as e:
            print(f"[transcribe error] {e}", file=sys.stderr)
        if command and pending:
            # A newer command was just carried out: older waiting audio is out of date.
            print(f"[queue] dropped {len(pending)} older utterance(s) (a newer command was sent)")
            pending.clear()
        print("\nListening again...\n")


async def main():
    global utterance_queue, ros_node, ros_pub
    utterance_queue = asyncio.Queue()

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

        print("Listening via Go2 microphone... say a command (e.g. 'sit', 'stand up', 'hello').")
        print("Press Ctrl+C to stop.\n")

        # Run until interrupted
        await asyncio.Event().wait()

    except ValueError as e:
        logging.error(f"Error in WebRTC connection: {e}")


def shutdown_ros():
    try:
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        shutdown_ros()
    sys.exit(0)
