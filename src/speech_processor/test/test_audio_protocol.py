import base64
import io
import unittest
import wave

from speech_processor.audio_protocol import build_megaphone_blocks, inspect_wav


def make_wav(frames: bytes = b"\x01\x00" * 100) -> bytes:
    output = io.BytesIO()
    with wave.open(output, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(22050)
        wav_file.writeframes(frames)
    return output.getvalue()


class AudioProtocolTest(unittest.TestCase):
    def test_inspect_wav_reports_generated_audio(self):
        info = inspect_wav(make_wav())
        self.assertEqual(info.channels, 1)
        self.assertEqual(info.sample_width, 2)
        self.assertEqual(info.sample_rate, 22050)
        self.assertEqual(info.frame_count, 100)
        self.assertGreater(info.duration, 0)

    def test_inspect_wav_rejects_empty_audio(self):
        with self.assertRaisesRegex(ValueError, "empty WAV"):
            inspect_wav(make_wav(b""))

    def test_megaphone_blocks_have_required_metadata_and_round_trip(self):
        wav_data = make_wav()
        blocks = build_megaphone_blocks(wav_data, chunk_size=17)

        self.assertGreater(len(blocks), 1)
        for index, block in enumerate(blocks, start=1):
            self.assertEqual(block["current_block_index"], index)
            self.assertEqual(block["total_block_number"], len(blocks))
            self.assertEqual(block["current_block_size"], len(block["block_content"]))

        encoded = "".join(block["block_content"] for block in blocks)
        self.assertEqual(base64.b64decode(encoded), wav_data)


if __name__ == "__main__":
    unittest.main()
