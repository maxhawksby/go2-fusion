#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Enhanced TTS Node

Improved Text-to-Speech functionality with better architecture,
caching, and multiple provider support.
"""

import io
import json
import os
import time
import hashlib
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
from enum import Enum
import threading

from pydub import AudioSegment
from pydub.playback import play
import rclpy
from rclpy.node import Node
import requests
from std_msgs.msg import String
from go2_interfaces.msg import WebRtcReq
from .audio_protocol import build_megaphone_blocks, inspect_wav

import wave
from piper import PiperVoice, SynthesisConfig
from pathlib import Path

class AudioFormat(Enum):
    """Supported audio formats"""
    MP3 = "mp3"
    WAV = "wav"
    OGG = "ogg"


class TTSProvider(Enum):
    """Supported TTS providers"""
    ELEVENLABS = "elevenlabs"
    GOOGLE = "google"
    AMAZON = "amazon"
    OPENAI = "openai"
    PIPER = "piper"


@dataclass
class TTSConfig:
    """Configuration for TTS functionality"""
    api_key: str
    provider: TTSProvider = TTSProvider.PIPER
    voice_name: str = "en_US-lessac-medium.onnx"
    local_playback: bool = False
    use_cache: bool = True
    cache_dir: str = "tts_cache"
    chunk_size: int = 4096
    audio_quality: str = "standard"  # standard, high
    language: str = "en"
    
    # ElevenLabs specific settings
    stability: float = 0.5
    similarity_boost: float = 0.5
    model_id: str = "eleven_turbo_v2_5"

    # Piper specific settings
    volume: float = 1.0
    length_scale: float = 1.0  # speed
    noise_scale: float = 1.0  # more audio variation
    noise_w_scale: float = 1.0  # more speaking variation
    normalize_audio: bool = True  # use raw audio from voice


class AudioCache:
    """Thread-safe audio cache management"""

    # Without a cap this directory grows without bound over the node's
    # lifetime - the only way to clear it was a ROS service that isn't even
    # wired up (see _setup_communication).
    MAX_CACHE_FILES = 200

    def __init__(self, cache_dir: str, enabled: bool = True):
        self.cache_dir = cache_dir
        self.enabled = enabled
        self._lock = threading.Lock()

        if self.enabled:
            os.makedirs(self.cache_dir, exist_ok=True)
    
    def get_cache_path(self, text: str, voice_name: str, provider: str) -> str:
        """Generate cache file path"""
        cache_key = f"{text}_{voice_name}_{provider}"
        text_hash = hashlib.md5(cache_key.encode()).hexdigest()
        extension = "wav" if provider == TTSProvider.PIPER.value else "mp3"
        return os.path.join(self.cache_dir, f"{text_hash}.{extension}")
    
    def get(self, text: str, voice_name: str, provider: str) -> Optional[bytes]:
        """Get cached audio data"""
        if not self.enabled:
            return None
            
        with self._lock:
            cache_path = self.get_cache_path(text, voice_name, provider)
            if os.path.exists(cache_path):
                with open(cache_path, "rb") as f:
                    return f.read()
        return None
    
    def put(self, text: str, voice_name: str, provider: str, audio_data: bytes) -> bool:
        """Cache audio data"""
        if not self.enabled or not audio_data:
            return False
            
        with self._lock:
            try:
                cache_path = self.get_cache_path(text, voice_name, provider)
                with open(cache_path, "wb") as f:
                    f.write(audio_data)
                self._evict_oldest_if_over_limit()
                return True
            except Exception:
                return False

    def _evict_oldest_if_over_limit(self) -> None:
        """Remove the oldest cached files once over MAX_CACHE_FILES. Caller
        must already hold self._lock."""
        try:
            entries = [
                os.path.join(self.cache_dir, f) for f in os.listdir(self.cache_dir)
            ]
            entries = [f for f in entries if os.path.isfile(f)]
            overflow = len(entries) - self.MAX_CACHE_FILES
            if overflow <= 0:
                return
            entries.sort(key=os.path.getmtime)
            for path in entries[:overflow]:
                os.unlink(path)
        except Exception:
            pass
    
    def clear(self) -> bool:
        """Clear all cached files"""
        if not self.enabled:
            return True
            
        with self._lock:
            try:
                for filename in os.listdir(self.cache_dir):
                    file_path = os.path.join(self.cache_dir, filename)
                    if os.path.isfile(file_path):
                        os.unlink(file_path)
                return True
            except Exception:
                return False
    
    def get_cache_stats(self) -> Dict[str, Any]:
        """Get cache statistics"""
        if not self.enabled:
            return {"enabled": False}
            
        with self._lock:
            try:
                files = os.listdir(self.cache_dir)
                total_size = sum(
                    os.path.getsize(os.path.join(self.cache_dir, f)) 
                    for f in files if os.path.isfile(os.path.join(self.cache_dir, f))
                )
                return {
                    "enabled": True,
                    "file_count": len(files),
                    "total_size_mb": round(total_size / (1024 * 1024), 2),
                    "cache_dir": self.cache_dir
                }
            except Exception:
                return {"enabled": True, "error": "Unable to read cache stats"}


class TTSProvider_ElevenLabs:
    """ElevenLabs TTS provider implementation"""
    
    def __init__(self, config: TTSConfig):
        self.config = config
        self.base_url = "https://api.elevenlabs.io/v1"
    
    def synthesize(self, text: str) -> Optional[bytes]:
        """Generate speech using ElevenLabs API"""
        url = f"{self.base_url}/text-to-speech/{self.config.voice_name}"
        
        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.config.api_key,
        }
        
        data = {
            "text": text,
            "model_id": self.config.model_id,
            "voice_settings": {
                "stability": self.config.stability,
                "similarity_boost": self.config.similarity_boost
            },
        }
        
        try:
            response = requests.post(url, json=data, headers=headers, timeout=30)
            response.raise_for_status()
            return response.content
        except requests.exceptions.RequestException:
            return None
    
    def get_voices(self) -> List[Dict[str, Any]]:
        """Get available voices"""
        url = f"{self.base_url}/voices"
        headers = {"xi-api-key": self.config.api_key}
        
        try:
            response = requests.get(url, headers=headers, timeout=10)
            response.raise_for_status()
            return response.json().get("voices", [])
        except requests.exceptions.RequestException:
            return []

class TTSProvider_Piper:
    """Piper TTS provider implementation"""
    
    def __init__(self, config: TTSConfig):
        self.config = config
        model_path = Path(config.voice_name)
        if not model_path.is_absolute():
            model_path = Path(os.getenv("PIPER_VOICE_DIR", "/ros2_ws")) / model_path
        if not model_path.is_file():
            raise FileNotFoundError(f"Piper voice model not found: {model_path}")
        self.voice = PiperVoice.load(model_path)
    
    def synthesize(self, text: str) -> Optional[bytes]:
        """Generate speech using Piper"""

        syn_config = SynthesisConfig(
            volume=self.config.volume,
            length_scale=self.config.length_scale,
            noise_scale=self.config.noise_scale,
            noise_w_scale=self.config.noise_w_scale,
            normalize_audio=self.config.normalize_audio,
        )

        bytes_buffer = io.BytesIO()

        with wave.open(bytes_buffer, "wb") as wav_file:
            self.voice.synthesize_wav(text, wav_file, syn_config=syn_config)

        return bytes_buffer.getvalue()

class AudioProcessor:
    """Audio processing utilities"""
    
    @staticmethod
    def convert_to_wav(audio_data: bytes, input_format: AudioFormat = AudioFormat.MP3) -> Optional[bytes]:
        """Convert audio data to WAV format"""
        try:
            if input_format == AudioFormat.MP3:
                audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            elif input_format == AudioFormat.OGG:
                audio = AudioSegment.from_ogg(io.BytesIO(audio_data))
            else:
                return audio_data  # Already WAV
            
            wav_io = io.BytesIO()
            audio.export(wav_io, format="wav")
            return wav_io.getvalue()
        except Exception:
            return None
    
    @staticmethod
    def get_duration(audio_data: bytes, format: AudioFormat = AudioFormat.WAV) -> float:
        """Get audio duration in seconds"""
        try:
            if format == AudioFormat.WAV:
                audio = AudioSegment.from_wav(io.BytesIO(audio_data))
            elif format == AudioFormat.MP3:
                audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            else:
                audio = AudioSegment.from_file(io.BytesIO(audio_data))
            
            return len(audio) / 1000.0  # Convert ms to seconds
        except Exception:
            return 0.0
    
    @staticmethod
    def split_into_chunks(data: bytes, chunk_size: int) -> List[bytes]:
        """Split data into chunks"""
        return [data[i:i + chunk_size] for i in range(0, len(data), chunk_size)]


class EnhancedTTSNode(Node):
    """Enhanced TTS Node with improved architecture"""
    
    def __init__(self):
        super().__init__("tts_node")
        
        # Declare parameters
        self._declare_parameters()
        
        # Load configuration
        self.config = self._load_configuration()
        
        # Initialize components
        self.cache = AudioCache(self.config.cache_dir, self.config.use_cache)
        self.audio_processor = AudioProcessor()
        self.playback_lock = threading.Lock()
        
        # Initialize TTS provider
        self.tts_provider = self._create_tts_provider()
        
        if not self.tts_provider:
            self.get_logger().error("Failed to initialize TTS provider!")
            return
        
        # Setup subscriptions and publishers
        self._setup_communication()
        
        # RTC topic for the audio-hub sub-protocol (start/chunk/end
        # commands). Matches go2_robot_sdk's
        # domain.constants.webrtc_topics.RTC_TOPIC["AUDIO_HUB_REQ"]. This
        # was previously 1003 (an api_id-looking int, not a topic string) -
        # WebRtcReq.topic is a `string` field, so every call to
        # _send_audio_command raised an AssertionError on the generated
        # message setter, silently caught by tts_callback's broad except -
        # meaning every non-local-playback TTS request failed outright.
        self.RTC_TOPIC = {"AUDIO_HUB_REQ": "rt/api/audiohub/request"}
        
        # Log initialization
        self._log_initialization()
    
    def _declare_parameters(self) -> None:
        """Declare all node parameters"""
        self.declare_parameter("api_key", "")
        self.declare_parameter("provider", "piper")
        self.declare_parameter("voice_name", "en_US-lessac-medium.onnx")
        self.declare_parameter("local_playback", False)
        self.declare_parameter("use_cache", False)
        self.declare_parameter("cache_dir", "tts_cache")
        self.declare_parameter("chunk_size", 4096)
        self.declare_parameter("audio_quality", "standard")
        self.declare_parameter("language", "en")
        self.declare_parameter("stability", 0.5)
        self.declare_parameter("similarity_boost", 0.5)
        self.declare_parameter("model_id", "eleven_turbo_v2_5")
    
    def _load_configuration(self) -> TTSConfig:
        """Load configuration from parameters"""
        provider_str = self.get_parameter("provider").get_parameter_value().string_value
        try:
            provider = TTSProvider(provider_str)
        except ValueError:
            provider = TTSProvider.PIPER
        
        return TTSConfig(
            api_key=self.get_parameter("api_key").get_parameter_value().string_value,
            provider=provider,
            voice_name=self.get_parameter("voice_name").get_parameter_value().string_value,
            local_playback=self.get_parameter("local_playback").get_parameter_value().bool_value,
            use_cache=self.get_parameter("use_cache").get_parameter_value().bool_value,
            cache_dir=self.get_parameter("cache_dir").get_parameter_value().string_value,
            chunk_size=self.get_parameter("chunk_size").get_parameter_value().integer_value,
            audio_quality=self.get_parameter("audio_quality").get_parameter_value().string_value,
            language=self.get_parameter("language").get_parameter_value().string_value,
            stability=self.get_parameter("stability").get_parameter_value().double_value,
            similarity_boost=self.get_parameter("similarity_boost").get_parameter_value().double_value,
            model_id=self.get_parameter("model_id").get_parameter_value().string_value,
        )
    
    def _create_tts_provider(self):
        """Create TTS provider based on configuration"""
        if self.config.provider == TTSProvider.ELEVENLABS:
            if not self.config.api_key:
                self.get_logger().error("ElevenLabs API key not provided!")
                return None
            return TTSProvider_ElevenLabs(self.config)
        elif self.config.provider == TTSProvider.PIPER:
            return TTSProvider_Piper(self.config)
        else:
            self.get_logger().error(f"Unsupported TTS provider: {self.config.provider}")
            return None
    
    def _setup_communication(self) -> None:
        """Setup ROS2 communication"""
        self.subscription = self.create_subscription(
            String, "/tts", self.tts_callback, 10
        )
        
        self.audio_pub = self.create_publisher(WebRtcReq, "/webrtc_req", 10)
        
        # Service for cache management
        # self.cache_service = self.create_service(
        #     Empty, "clear_tts_cache", self.clear_cache_callback
        # )
    
    MAX_TEXT_LENGTH = 1000

    def tts_callback(self, msg: String) -> None:
        """Handle incoming TTS requests.

        Dispatches to a background thread: synthesis (a blocking HTTP call)
        and robot playback (a sequence of blocking time.sleep() calls
        totaling tens of seconds to minutes) used to run directly in this
        subscription callback, on the default single-threaded executor -
        blocking every other callback on this node for the duration of a
        single TTS request.
        """
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("Received empty TTS request")
            return
        if len(text) > self.MAX_TEXT_LENGTH:
            self.get_logger().warn(
                f"TTS request text is {len(text)} chars, exceeds "
                f"{self.MAX_TEXT_LENGTH}-char limit; truncating."
            )
            text = text[:self.MAX_TEXT_LENGTH]

        threading.Thread(
            target=self._process_tts_request, args=(text,), daemon=True
        ).start()

    def _process_tts_request(self, text: str) -> None:
        """Synthesize (or fetch from cache) and play back one TTS request.
        Runs on a background thread - see tts_callback."""
        try:
            self.get_logger().info(f'🎤 TTS Request: "{text}" (voice: {self.config.voice_name})')
            
            # Check cache first
            cache_hit = False
            audio_data = self.cache.get(text, self.config.voice_name, self.config.provider.value)
            
            if audio_data:
                self.get_logger().info("💾 Cache hit - using cached audio")
                cache_hit = True
            else:
                # Generate new speech
                self.get_logger().info("🔊 Generating new speech...")
                audio_data = self.tts_provider.synthesize(text)
                
                if audio_data:
                    # Cache the result
                    if self.cache.put(text, self.config.voice_name, self.config.provider.value, audio_data):
                        self.get_logger().info("💾 Audio cached successfully")
                else:
                    self.get_logger().error("❌ Failed to generate speech")
                    return
            
            # Process and play audio
            if self.config.local_playback:
                self._play_locally(audio_data)
            else:
                # AudioHub has one megaphone upload session. Serializing here
                # prevents concurrent /tts callbacks from interleaving chunks.
                with self.playback_lock:
                    self._play_on_robot(audio_data)
            
            # Log success
            status = "cached" if cache_hit else "generated"
            self.get_logger().info(f"✅ TTS completed successfully ({status})")
            
        except Exception as e:
            self.get_logger().error(f"❌ TTS processing error: {str(e)}")
    
    def _play_locally(self, audio_data: bytes) -> None:
        """Play audio locally"""
        try:
            audio = AudioSegment.from_mp3(io.BytesIO(audio_data))
            play(audio)
            self.get_logger().info("🔊 Local playback completed")
        except Exception as e:
            try:
                audio = AudioSegment.from_wav(io.BytesIO(audio_data))
                play(audio)
                self.get_logger().info("🔊 Local playback completed")
            except Exception as e:
                self.get_logger().error(f"❌ Local playback error: {str(e)}")
    
                
    def _play_on_robot(self, audio_data: bytes) -> None:
        """Send audio to robot for playback, accepting MP3 or WAV audio."""
        try:
            # Detect WAV format using its file header
            if (
                len(audio_data) >= 12
                and audio_data[:4] == b"RIFF"
                and audio_data[8:12] == b"WAVE"
            ):
                # Already WAV; skip conversion
                wav_data = audio_data
                self.get_logger().info("Audio is already WAV; skipping conversion")
            else:
                # Assume MP3 and convert to WAV
                wav_data = self.audio_processor.convert_to_wav(
                    audio_data, AudioFormat.MP3
                )

            if not wav_data:
                self.get_logger().error("❌ Failed to convert audio to WAV")
                return

            wav_info = inspect_wav(wav_data)
            duration = wav_info.duration
            blocks = build_megaphone_blocks(wav_data, self.config.chunk_size)
            total_chunks = len(blocks)

            self.get_logger().info(
                f"📤 Sending WAV to robot: {len(wav_data)} bytes, "
                f"{wav_info.sample_rate} Hz, {total_chunks} chunks, "
                f"{duration:.1f}s duration"
            )

            # Send start command
            self._send_audio_command(4001, "")
            time.sleep(0.1)

            # Send audio chunks
            for audio_block in blocks:
                chunk_idx = audio_block["current_block_index"]
                self._send_audio_command(4003, json.dumps(audio_block))

                if chunk_idx % 10 == 0:
                    self.get_logger().info(
                        f"📤 Sent {chunk_idx}/{total_chunks} chunks"
                    )

                time.sleep(0.2)  # Prevent flooding

            # Wait for playback to complete
            self.get_logger().info(
                f"⏳ Waiting for playback completion ({duration:.1f}s)..."
            )
            time.sleep(duration + 1.0)

            # Send end command
            self._send_audio_command(4002, "")

            self.get_logger().info("🎵 Robot playback completed")

        except Exception as e:
            self.get_logger().error(f"❌ Robot playback error: {str(e)}")

    def _send_audio_command(self, api_id: int, parameter: str) -> None:
        """Send audio command to robot"""
        req = WebRtcReq()
        req.api_id = api_id
        req.priority = 0
        req.parameter = parameter
        req.topic = self.RTC_TOPIC["AUDIO_HUB_REQ"]
        self.audio_pub.publish(req)
    
    def _log_initialization(self) -> None:
        """Log initialization details"""
        cache_stats = self.cache.get_cache_stats()
        
        self.get_logger().info("🎤 Enhanced TTS Node Initialized")
        self.get_logger().info(f"   Provider: {self.config.provider.value}")
        self.get_logger().info(f"   Voice: {self.config.voice_name}")
        self.get_logger().info(f"   Playback: {'Local' if self.config.local_playback else 'Robot'}")
        self.get_logger().info(f"   Language: {self.config.language}")
        self.get_logger().info(f"   Quality: {self.config.audio_quality}")
        
        if cache_stats["enabled"]:
            self.get_logger().info(f"   Cache: Enabled ({cache_stats.get('file_count', 0)} files, "
                                 f"{cache_stats.get('total_size_mb', 0)}MB)")
        else:
            self.get_logger().info("   Cache: Disabled")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = EnhancedTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"❌ TTS Node error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main() 
