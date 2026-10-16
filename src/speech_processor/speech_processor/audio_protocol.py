"""Pure helpers for validating and chunking Go2 AudioHub WAV uploads."""

import base64
import io
import wave
from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class WavInfo:
    channels: int
    sample_width: int
    sample_rate: int
    frame_count: int

    @property
    def duration(self) -> float:
        return self.frame_count / self.sample_rate


def inspect_wav(wav_data: bytes) -> WavInfo:
    """Return WAV metadata and reject empty or malformed audio."""
    try:
        with wave.open(io.BytesIO(wav_data), "rb") as wav_file:
            info = WavInfo(
                channels=wav_file.getnchannels(),
                sample_width=wav_file.getsampwidth(),
                sample_rate=wav_file.getframerate(),
                frame_count=wav_file.getnframes(),
            )
    except (EOFError, wave.Error) as exc:
        raise ValueError("TTS provider returned an invalid WAV file") from exc

    if info.frame_count <= 0 or info.sample_rate <= 0:
        raise ValueError("TTS provider returned an empty WAV file")
    return info


def build_megaphone_blocks(wav_data: bytes, chunk_size: int) -> List[Dict[str, object]]:
    """Encode a WAV using the fields required by AudioHub API 4003."""
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")

    encoded = base64.b64encode(wav_data).decode("ascii")
    chunks = [encoded[i:i + chunk_size] for i in range(0, len(encoded), chunk_size)]
    total = len(chunks)
    return [
        {
            "current_block_size": len(chunk),
            "block_content": chunk,
            "current_block_index": index,
            "total_block_number": total,
        }
        for index, chunk in enumerate(chunks, start=1)
    ]
