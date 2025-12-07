# Chapter 3: Language Planning with Whisper & Large Language Models

**Module**: 4 | **Week**: 12 | **Difficulty**: Intermediate | **Estimated Time**: 8–10 hours

---

## Learning Objectives

After completing this chapter, you will be able to:

1. **Integrate Whisper speech-to-text** API to convert voice commands to text
2. **Design effective LLM prompts** for robotics task decomposition using chain-of-thought reasoning
3. **Parse LLM outputs** into executable skill sequences and detect ambiguities
4. **Implement task decomposition** that converts natural language to structured action plans
5. **Design and implement replanning logic** to recover from task failures and adapt to dynamic environments
6. **Explain behavior trees** as hierarchical task representations and implement basic behavior tree engines

---

## Key Concepts

- **Speech Recognition** (Whisper): Converting audio to text with >95% accuracy across languages and accents
- **Prompt Engineering**: Crafting LLM inputs to reliably produce structured, robotics-compatible outputs
- **Chain-of-Thought Reasoning**: Breaking complex tasks into step-by-step reasoning before execution
- **Task Decomposition**: Converting high-level goals into low-level executable skills
- **Behavior Trees**: Hierarchical, modular representations of robot behaviors
- **Replanning**: Detecting failures and generating alternative action sequences
- **Tool Use**: Teaching LLMs to call functions (e.g., moveto, pick, place) as part of reasoning

---

## Part 1: Speech Recognition with Whisper

### Whisper Overview

Whisper is OpenAI's speech-to-text model that achieves robust, multi-lingual speech recognition. Key features:

- **Accuracy**: >95% on clear speech, ~80% on noisy audio (significantly better than before ~2024)
- **Languages**: Trained on 99 languages (multilingual capability)
- **Model Sizes**: tiny (39M) → base (74M) → small (244M) → medium (769M) → large (1.5B)
- **Latency**:
  - Tiny/base: ~100ms per 30 seconds of audio (real-time capable)
  - Large: ~500ms per 30 seconds (sufficient for planning, not control-loop speed)
- **Cost**: API pricing ~$0.02 per minute of audio (free if self-hosted)

### Whisper Architecture

```
Audio Input (16kHz PCM, mono)
    ↓
[Mel Spectrogram Encoder]  → 512-dim features per 20ms
    ↓
[Transformer Encoder]  → Processes temporal context
    ↓
[BOS token] → Start decoding
    ↓
[Transformer Decoder]  → Generates text tokens autoregressively
    ↓
Text Output ("pick up the red cup")
```

Whisper's key innovation: **Trained on weakly-supervised data from the web** (video captions), making it robust to diverse audio conditions, accents, and background noise.

### Integration with ROS 2

```python
# File: ros2_whisper_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Audio  # Audio message type
from std_msgs.msg import String
import numpy as np
from openai import OpenAI
import threading

class WhisperNode(Node):
    """
    ROS 2 node that:
    1. Subscribes to audio topic
    2. Buffers audio (e.g., 5 seconds)
    3. Sends to Whisper API
    4. Publishes transcribed text
    """

    def __init__(self):
        super().__init__('whisper_node')

        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

        # Subscribers
        self.audio_subscription = self.create_subscription(
            String,  # In practice, you'd use a custom Audio message
            '/microphone/audio_bytes',
            self.audio_callback,
            10
        )

        # Publishers
        self.transcription_publisher = self.create_publisher(
            String, '/whisper/transcription', 10
        )

        # Audio buffer
        self.audio_buffer = np.array([], dtype=np.int16)
        self.sample_rate = 16000  # 16 kHz
        self.buffer_duration = 5.0  # 5 seconds
        self.buffer_size = int(self.sample_rate * self.buffer_duration)

        self.get_logger().info("Whisper node initialized")

    def audio_callback(self, msg):
        """Buffer incoming audio."""
        # Parse audio from message (format depends on message type)
        # For simplicity, assume msg.data is base64-encoded PCM
        import base64
        audio_bytes = base64.b64decode(msg.data)
        audio_chunk = np.frombuffer(audio_bytes, dtype=np.int16)

        # Append to buffer
        self.audio_buffer = np.append(self.audio_buffer, audio_chunk)

        # If buffer is full, process
        if len(self.audio_buffer) >= self.buffer_size:
            self._process_audio_buffer()

    def _process_audio_buffer(self):
        """Send buffered audio to Whisper API."""
        # Convert to audio format (WAV file in memory)
        import io
        import wave

        buffer_to_process = self.audio_buffer[:self.buffer_size]
        self.audio_buffer = self.audio_buffer[self.buffer_size:]  # Clear buffer

        # Create WAV file
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(buffer_to_process.tobytes())

        wav_buffer.seek(0)

        # Send to Whisper API
        try:
            response = self.client.audio.transcriptions.create(
                model="whisper-1",
                file=wav_buffer,
                language="en"
            )

            transcript = response.text
            self.get_logger().info(f"Transcribed: {transcript}")

            # Publish
            msg = String()
            msg.data = transcript
            self.transcription_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Whisper API error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Handling Noisy Input

Whisper is robust, but can still misrecognize. Strategies:

1. **Confidence Thresholding**: Only accept high-confidence transcriptions
2. **Fallback Mechanisms**: If uncertain, ask user for confirmation
3. **Context**: Use previous commands to correct likely mistakes

```python
def transcribe_with_fallback(audio_bytes, confidence_threshold=0.8):
    """
    Transcribe with fallback to user confirmation.
    """
    # Get Whisper transcription
    transcript = whisper_api.transcribe(audio_bytes)

    # If confidence is low, ask user
    if transcript.confidence < confidence_threshold:
        return f"Did you say: {transcript.text}? (yes/no)"
    else:
        return transcript.text
```

---

## Part 2: Large Language Models for Task Decomposition

### LLMs for Robotics

Large Language Models (GPT-4, Claude, etc.) excel at task reasoning. For robotics:

**Strengths**:
- Natural language understanding (parsing varied commands)
- Reasoning (breaking complex tasks into steps)
- Generalization (handling novel scenarios)
- Flexibility (no pre-programmed task library needed)

**Limitations**:
- Latency (seconds per response, too slow for control loops)
- Cost (API calls)
- Hallucination (may propose infeasible actions)
- Context length (can't maintain very long task histories)

**Best Practice**: Use LLMs for **planning and high-level reasoning**, not for real-time control.

### Prompting Strategies

Effective prompts for robot task decomposition include:

1. **System Prompt**: Define the robot's capabilities and constraints
2. **Context**: Describe the scene (from perception module)
3. **Task**: The user's command
4. **Output Format**: Structured JSON with step-by-step actions

**Example System Prompt**:
```
You are a task planner for a humanoid robot. The robot has the following capabilities:

Skills:
- MoveTo(location): Navigate to a 2D position
- Pick(object, gripper_force): Grasp an object
- Place(location, release_speed): Release object at location
- Search(object_name): Find object in environment
- Open(object): Open doors, drawers, containers
- Close(object): Close doors, drawers

Constraints:
- Robot can only carry one object at a time
- Gripper force must be: soft (0.5), moderate (1.0), or firm (1.5)
- Cannot move through walls or locked doors
- Max reach distance: 1.0m from base

When given a user command, decompose it into a sequence of skills.
Return JSON format: {"steps": [{"skill": "...", "params": {...}}, ...]}
```

**Example Prompt Session**:
```
System: [System prompt above]

User: "Pick up the red cup from the table and place it on the shelf"

Context: Scene has: red cup at (0.5, 0.3), table at (0.5, 0.0), shelf at (0.8, 1.0)

Task: Decompose the command into robot skills.