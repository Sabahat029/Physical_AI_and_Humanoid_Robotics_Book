# Chapter 10: Voice-to-Action Integration

**Date**: December 16, 2025
**Module**: Vision-Language-Action (VLA) - Human-Robot Interaction
**Chapter**: 10 of 12
**Estimated Reading Time**: 150 minutes
**Prerequisites**: Module 1-3 knowledge, understanding of natural language processing, real-time system concepts, human-robot interaction principles

## Learning Objectives

By the end of this chapter, you will be able to:

1. Integrate speech recognition systems with robot control using modern approaches like Whisper
2. Process audio input and map voice commands to specific robot actions
3. Design robust voice command processing pipelines with confidence scoring
4. Handle real-time speech processing with low latency requirements
5. Implement multi-modal interaction combining voice, vision, and action
6. Create error handling and user feedback mechanisms for voice interfaces

---

## 10.1 Introduction to Voice-to-Action in Physical AI

### Voice Commands as Natural Interaction Modality

Voice-to-action integration represents a crucial capability for Physical AI systems, enabling natural human-robot interaction through spoken language. Unlike traditional command-line interfaces or application-based controls, voice commands allow humans to interact with robots using their most natural communication modality. For Physical AI systems, this means translating linguistic instructions into embodied actions in the physical world.

The challenge of voice-to-action integration extends beyond simple speech recognition. It requires understanding the context of spoken commands, mapping abstract language to concrete robotic actions, and executing these actions while maintaining the natural flow of human conversation. This is particularly complex for Physical AI systems that must interpret commands in the context of their physical environment and current state.

Voice-to-action systems must handle the inherent ambiguity of natural language. When a human says "Pick up the red cup," the system must determine:
- What constitutes "the red cup" among potentially multiple objects
- The spatial relationships needed for "picking up"
- The current state of the robot and its available actions
- The environmental constraints that might affect execution

### Components of Voice-to-Action Systems

A complete voice-to-action system typically includes several key components:

**Speech Recognition**: Converting audio signals to text with appropriate confidence measures

**Natural Language Understanding (NLU)**: Parsing the text to extract intent, entities, and action specifications

**Context Resolution**: Using environmental and situational context to disambiguate commands

**Action Planning**: Mapping high-level commands to specific robot behaviors

**Execution and Feedback**: Carrying out actions and providing appropriate feedback

Each component must operate efficiently to maintain natural interaction timing, typically requiring responses within 1-2 seconds for acceptable user experience.

### Challenges in Physical AI Voice Integration

Voice integration in Physical AI systems faces several unique challenges:

**Environmental Noise**: Physical robot environments often have significant background noise that can interfere with speech recognition

**Distance and Directionality**: Microphones may be distant from speakers, and robot orientation may affect signal quality

**Real-time Requirements**: Physical interactions often require immediate response, limiting processing time

**Ambiguity Resolution**: Natural language commands may require environmental context for proper interpretation

**Safety Considerations**: Voice commands must be processed carefully to prevent unsafe robot behaviors

### Architecture for Voice-to-Action Integration

The architecture for voice-to-action integration typically follows a pipeline pattern with feedback mechanisms for improved performance:

```
Audio Input → Preprocessing → Speech Recognition → NLU → Context Resolution → Action Planning → Robot Execution
```

With feedback loops for:
- Execution results informing future command interpretation
- Environmental state updates affecting command resolution
- User feedback for system improvement

---

## 10.2 Speech Recognition and Audio Processing

### Modern Speech Recognition Approaches

Modern speech recognition has been revolutionized by deep learning approaches, particularly transformer-based models. OpenAI's Whisper represents one of the most effective open-source options for voice-to-text conversion, offering multilingual support and robust performance across various acoustic conditions.

Whisper's architecture combines an encoder-decoder transformer with attention mechanisms, trained on large-scale multilingual and multitask learning objectives. This results in robust performance across different speakers, languages, and acoustic conditions, making it particularly suitable for robotic applications.

```python
import whisper
import torch
import numpy as np
import pyaudio
import wave
import threading
from queue import Queue
import time

class SpeechRecognizer:
    def __init__(self, model_size="small", device="cuda" if torch.cuda.is_available() else "cpu"):
        """
        Initialize speech recognition system with Whisper model
        """
        self.model_size = model_size
        self.device = device
        
        # Load Whisper model
        self.model = whisper.load_model(model_size, device=device)
        
        # Audio parameters
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        
        # Audio input setup
        self.audio = pyaudio.PyAudio()
        self.stream = None
        
        # Processing queue for real-time operation
        self.audio_queue = Queue()
        self.result_queue = Queue()
        
        # Recognition parameters
        self.energy_threshold = 1000  # Minimum audio energy to consider as speech
        self.silence_duration = 1.0   # Seconds of silence to trigger recognition
        
        print(f"Speech recognizer initialized with model: {model_size}, device: {device}")
    
    def start_listening(self):
        """Start audio input and recognition in background thread"""
        self.stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        
        # Start background thread for audio processing
        self.listening_thread = threading.Thread(target=self._audio_input_loop, daemon=True)
        self.listening_thread.start()
        
        print("Started listening for voice commands...")
    
    def stop_listening(self):
        """Stop audio input and recognition"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
    
    def _audio_input_loop(self):
        """Continuous audio input and processing loop"""
        audio_buffer = []
        silent_chunks = 0
        total_chunks = 0
        silence_threshold = int(self.silence_duration * self.sample_rate / self.chunk_size)
        
        while True:
            try:
                # Read audio data
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                audio_array = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                
                # Calculate energy for voice detection
                energy = np.sum(audio_array ** 2) / len(audio_array)
                
                if energy > self.energy_threshold:
                    # Voice detected, add to buffer
                    audio_buffer.extend(audio_array)
                    silent_chunks = 0
                    total_chunks += 1
                else:
                    # Silence detected
                    if len(audio_buffer) > 0:
                        silent_chunks += 1
                    
                    # If enough silence and we have collected some audio
                    if silent_chunks >= silence_threshold and len(audio_buffer) > 0:
                        # Process the collected audio
                        audio_segment = np.array(audio_buffer)
                        
                        # Add to processing queue
                        self.audio_queue.put(audio_segment.copy())
                        
                        # Clear buffer for next segment
                        audio_buffer = []
                        
                        # Reset counters
                        silent_chunks = 0
                        total_chunks = 0
                    elif len(audio_buffer) > 0:  # Still in speech, continue collecting
                        audio_buffer.extend(audio_array)
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                print(f"Error in audio input loop: {e}")
                break
    
    def process_audio_segment(self, audio_segment):
        """Process a single audio segment using Whisper"""
        try:
            # Pad audio to minimum required length for Whisper
            if len(audio_segment) < 32000:  # Whisper minimum is ~1 second at 16kHz
                padding = np.zeros(32000 - len(audio_segment))
                audio_segment = np.concatenate([audio_segment, padding])
            
            # Run Whisper transcription
            result = self.model.transcribe(
                audio_segment,
                language='en',
                temperature=0.0,  # Deterministic results
                compression_ratio_threshold=2.4,  # Filter out low-quality transcriptions
                logprob_threshold=-1.0,  # Filter out low-probability transcriptions
                no_speech_threshold=0.6  # Filter out non-speech audio
            )
            
            if result and result['text'].strip():
                return {
                    'text': result['text'].strip(),
                    'confidence': self._calculate_confidence(result),
                    'timestamp': time.time(),
                    'language': result.get('language', 'unknown')
                }
        
        except Exception as e:
            print(f"Error processing audio segment: {e}")
        
        return None
    
    def _calculate_confidence(self, result):
        """Calculate confidence score from Whisper results"""
        # Use the average log probability as confidence indicator
        if 'avg_logprob' in result:
            # Convert log probability to confidence (higher is better, but negative)
            # So we use 1 - abs(avg_logprob) normalized to positive range
            avg_logprob = result['avg_logprob']
            # Logprob is negative, so more negative = less confident
            # Normalize to 0-1 range where 1 is most confident
            confidence = max(0.0, min(1.0, 1.0 - abs(avg_logprob + 1.0)))
            return confidence
        
        return 0.5  # Default confidence if not available
    
    def get_recognized_command(self, timeout=None):
        """Get the next recognized voice command"""
        try:
            return self.result_queue.get(timeout=timeout)
        except:
            return None

# Example usage
recognizer = SpeechRecognizer()
recognizer.start_listening()

# In a separate thread or process, you would continuously process commands:
def command_processor():
    while True:
        if not recognizer.audio_queue.empty():
            audio_segment = recognizer.audio_queue.get()
            result = recognizer.process_audio_segment(audio_segment)
            if result:
                print(f"Recognized: {result['text']} (confidence: {result['confidence']:.2f})")
                # Add to result queue for higher-level processing
                recognizer.result_queue.put(result)
        time.sleep(0.1)

# Start command processor
processor_thread = threading.Thread(target=command_processor, daemon=True)
processor_thread.start()
```

### Audio Preprocessing for Robot Environments

Robotic environments often present challenging acoustic conditions that require specialized preprocessing. The robot's own motors, fans, and mechanical sounds can interfere with speech recognition. Additionally, ambient noise from the operating environment needs to be addressed.

```python
import webrtcvad
import collections
import scipy.signal
import librosa

class AudioPreprocessor:
    def __init__(self):
        # Voice Activity Detection
        self.vad = webrtcvad.Vad(2)  # Sensitivity level 2 (0-3)
        
        # Parameters for VAD
        self.sample_rate = 16000
        self.frame_duration = 30  # ms
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)
        
        # Noise reduction parameters
        self.fft_size = 2048
        self.hop_length = 512
        
        # For noise estimation
        self.noise_buffer_size = 50
        self.noise_buffer = collections.deque(maxlen=self.noise_buffer_size)
        
    def preprocess_audio(self, audio_data):
        """
        Apply preprocessing to improve speech recognition in noisy robot environments
        """
        # Step 1: Noise reduction using spectral subtraction
        enhanced_audio = self._spectral_subtraction(audio_data)
        
        # Step 2: Voice activity detection
        voice_active = self._detect_voice_activity(enhanced_audio)
        
        if not voice_active:
            return None  # No voice detected
        
        # Step 3: Audio normalization
        normalized_audio = self._normalize_audio(enhanced_audio)
        
        return normalized_audio
    
    def _spectral_subtraction(self, audio_data):
        """
        Apply spectral subtraction for noise reduction
        """
        # Compute STFT
        stft = librosa.stft(audio_data, n_fft=self.fft_size, hop_length=self.hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)
        
        # Estimate noise spectrum from initial frames (assumed to be noise)
        if len(self.noise_buffer) < self.noise_buffer_size:
            # If noise estimation is not ready, return original
            if len(audio_data) > 2048:  # Only add if we have enough samples
                # Estimate current noise from low-energy frames
                noise_estimate = np.mean(magnitude[:, :10], axis=1) if magnitude.shape[1] > 10 else np.mean(magnitude, axis=1)
                self.noise_buffer.append(noise_estimate)
        
        # If we have noise estimate, apply spectral subtraction
        if len(self.noise_buffer) > 0:
            avg_noise = np.mean(list(self.noise_buffer), axis=0)
            
            # Apply spectral subtraction
            enhanced_magnitude = np.maximum(magnitude - avg_noise[:, np.newaxis], 0.3 * magnitude)
            
            # Reconstruct signal
            enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
            enhanced_audio = librosa.istft(enhanced_stft, hop_length=self.hop_length, length=len(audio_data))
            
            return enhanced_audio
        
        return audio_data  # Return original if no noise model available
    
    def _detect_voice_activity(self, audio_data):
        """
        Detect if voice activity is present using WebRTC VAD
        """
        # Convert to the format expected by WebRTC VAD
        audio_int16 = (audio_data * 32767).astype(np.int16)
        audio_bytes = audio_int16.tobytes()
        
        # Split into frames
        frames = self._frame_generator(audio_bytes)
        
        voice_count = 0
        total_frames = 0
        
        for frame in frames:
            if self.vad.is_speech(frame, self.sample_rate):
                voice_count += 1
            total_frames += 1
        
        # Consider voice active if more than 30% of frames contain voice
        return voice_count / max(1, total_frames) > 0.3
    
    def _frame_generator(self, audio_bytes):
        """Generate audio frames for VAD processing"""
        frame_size = self.frame_size * 2  # 2 bytes per sample for int16
        for i in range(0, len(audio_bytes) - frame_size, frame_size):
            yield audio_bytes[i:i + frame_size]
    
    def _normalize_audio(self, audio_data):
        """Normalize audio to standard level"""
        # Normalize to -20 dB RMS
        rms = np.sqrt(np.mean(audio_data ** 2))
        target_rms = 10 ** (-20 / 20)  # -20 dB
        
        if rms > 0:
            gain = target_rms / rms
            return audio_data * gain
        else:
            return audio_data
```

### Real-time Processing Considerations

Real-time voice processing for robotics requires careful attention to latency and computational efficiency. The system must process audio in real-time while maintaining high accuracy.

```python
class RealTimeVoiceProcessor:
    def __init__(self, speech_recognizer, audio_preprocessor):
        self.recognizer = speech_recognizer
        self.preprocessor = audio_preprocessor
        
        # Processing pipeline
        self.processing_queue = collections.deque(maxlen=10)
        self.result_cache = {}
        
        # Performance metrics
        self.processing_times = collections.deque(maxlen=50)
        self.accuracy_history = collections.deque(maxlen=50)
        
    def process_realtime_audio(self, audio_chunk):
        """
        Process audio chunk in real-time with performance optimization
        """
        start_time = time.time()
        
        # Preprocess audio
        preprocessed = self.preprocessor.preprocess_audio(audio_chunk)
        if preprocessed is None:
            return None
        
        # Add to processing queue
        self.processing_queue.append({
            'audio': preprocessed,
            'timestamp': time.time()
        })
        
        # Process when queue has enough data
        if len(self.processing_queue) >= 3:  # Process in batches for efficiency
            batch_result = self._process_batch()
            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)
            
            return batch_result
        
        return None
    
    def _process_batch(self):
        """Process multiple audio segments efficiently"""
        batch_audio = np.concatenate([item['audio'] for item in self.processing_queue])
        self.processing_queue.clear()
        
        # Process the combined audio
        result = self.recognizer.process_audio_segment(batch_audio)
        
        return result
    
    def get_performance_metrics(self):
        """Get real-time performance metrics"""
        if not self.processing_times:
            return None
            
        avg_processing_time = sum(self.processing_times) / len(self.processing_times)
        
        return {
            'avg_processing_time': avg_processing_time,
            'processing_rate': 1.0 / avg_processing_time if avg_processing_time > 0 else float('inf'),
            'queue_depth': len(self.processing_queue),
            'sample_rate': self.recognizer.sample_rate
        }
```

---

## 10.3 Natural Language Understanding for Robot Commands

### Command Structure and Intent Recognition

Natural language understanding for robotics requires specialized processing to map natural language commands to executable robot actions. Unlike general-purpose language understanding, robot command understanding must account for the physical nature of robot capabilities and environmental context.

```python
import re
import spacy
from typing import Dict, List, Tuple, Optional
import inflect

class RobotCommandParser:
    def __init__(self):
        # Load spaCy English model (you may need to install it: python -m spacy download en_core_web_sm)
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            print("spaCy English model not found. Please install with: python -m spacy download en_core_web_sm")
            self.nlp = None
        
        self.inflect_engine = inflect.engine()
        
        # Define robot action vocabulary
        self.action_verbs = {
            'move': ['go', 'move', 'navigate', 'walk', 'drive', 'travel'],
            'grasp': ['pick up', 'take', 'grasp', 'grab', 'lift', 'collect'],
            'place': ['put', 'place', 'set', 'drop', 'release', 'position'],
            'bring': ['bring', 'fetch', 'carry', 'transport', 'deliver'],
            'follow': ['follow', 'accompany', 'accompany', 'go after'],
            'stop': ['stop', 'halt', 'wait', 'pause'],
            'look_at': ['look at', 'see', 'observe', 'examine', 'view'],
            'point_to': ['point to', 'point at', 'indicate', 'show'],
        }
        
        # Object types and categories
        self.object_categories = {
            'container': ['cup', 'bottle', 'box', 'bowl', 'glass', 'mug'],
            'food': ['apple', 'banana', 'snack', 'cookie', 'fruit', 'sandwich'],
            'tool': ['pen', 'book', 'phone', 'keys', 'tablet', 'remote'],
            'furniture': ['table', 'chair', 'couch', 'desk', 'shelf', 'bed'],
        }
        
        # Spatial relationships and directions
        self.spatial_relations = [
            'near', 'next to', 'beside', 'on', 'in', 'at', 'by', 'to', 'from',
            'left', 'right', 'front', 'back', 'above', 'below', 'in front of', 
            'behind', 'across from', 'around'
        ]
        
        # Location references
        self.location_keywords = [
            'kitchen', 'living room', 'bedroom', 'office', 'bathroom', 
            'dining room', 'hallway', 'door', 'window', 'cabinet'
        ]

    def parse_command(self, text: str) -> Optional[Dict]:
        """
        Parse natural language command to extract action, objects, and spatial relationships
        """
        if not self.nlp:
            return None
            
        doc = self.nlp(text.lower())
        
        # Extract primary action
        action = self._extract_action(doc)
        if not action:
            return None
        
        # Extract objects to manipulate
        objects = self._extract_objects(doc)
        
        # Extract spatial information
        spatial_info = self._extract_spatial_info(doc)
        
        # Extract destination/locations
        locations = self._extract_locations(doc)
        
        # Extract modifiers (quantifiers, colors, sizes)
        modifiers = self._extract_modifiers(doc)
        
        command_structure = {
            'action': action,
            'objects': objects,
            'spatial_info': spatial_info,
            'locations': locations,
            'modifiers': modifiers,
            'original_text': text,
            'confidence': self._calculate_parse_confidence(doc)
        }
        
        return command_structure
    
    def _extract_action(self, doc) -> Optional[str]:
        """Extract the primary action verb from the command"""
        for token in doc:
            # Check if token matches any action verb pattern
            current_span = 1
            while current_span <= 3 and len(doc) > token.i + current_span - 1:
                span_text = ' '.join([t.text for t in doc[token.i:token.i + current_span]])
                
                for action_type, verbs in self.action_verbs.items():
                    if span_text in verbs or token.lemma_ in verbs:
                        return action_type
                
                current_span += 1
            
            # Also check single token if span didn't match
            if token.pos_ == 'VERB' or token.lemma_ in [v for verbs in self.action_verbs.values() for v in verbs]:
                for action_type, verbs in self.action_verbs.items():
                    if token.lemma_ in verbs:
                        return action_type
        
        return None
    
    def _extract_objects(self, doc) -> List[Dict]:
        """Extract objects mentioned in the command"""
        objects = []
        
        # Look for noun phrases and entities
        for chunk in doc.noun_chunks:
            # Check if this chunk is describing an object
            if self._is_physical_object(chunk.root):
                obj_info = {
                    'text': chunk.text,
                    'lemma': chunk.root.lemma_,
                    'category': self._categorize_object(chunk.root.lemma_),
                    'modifiers': []
                }
                
                # Look for adjectives that modify the object
                for token in chunk:
                    if token.pos_ == 'ADJ':
                        obj_info['modifiers'].append(token.text)
                
                objects.append(obj_info)
        
        return objects
    
    def _extract_spatial_info(self, doc) -> List[Dict]:
        """Extract spatial relationships and positioning information"""
        spatial_info = []
        
        # Look for prepositional phrases indicating spatial relationships
        for token in doc:
            if token.pos_ == 'ADP' and token.text in self.spatial_relations:
                # Extract what the preposition relates to and what it refers to
                related_to = token.head.text if token.head else None
                reference_point = None
                
                # Look for the object of the preposition
                for child in token.children:
                    if child.pos_ in ['NOUN', 'PROPN', 'PRON']:
                        reference_point = child.text
                        break
                
                spatial_info.append({
                    'relation': token.text,
                    'related_to': related_to,
                    'reference_point': reference_point
                })
        
        return spatial_info
    
    def _extract_locations(self, doc) -> List[str]:
        """Extract location information from the command"""
        locations = []
        
        for token in doc:
            if any(location in token.text for location in self.location_keywords):
                locations.append(token.text)
        
        # Also check noun chunks that might contain location info
        for chunk in doc.noun_chunks:
            if any(location in chunk.text.lower() for location in self.location_keywords):
                locations.append(chunk.text)
        
        return list(set(locations))  # Remove duplicates
    
    def _extract_modifiers(self, doc) -> List[Dict]:
        """Extract quantity and quality modifiers"""
        modifiers = []
        
        for token in doc:
            if token.pos_ in ['NUM', 'ADJ', 'DET']:
                modifier_info = {
                    'type': token.pos_,
                    'text': token.text,
                    'value': self._parse_number(token.text) if token.pos_ == 'NUM' else token.text
                }
                modifiers.append(modifier_info)
        
        return modifiers
    
    def _is_physical_object(self, token) -> bool:
        """Determine if a token likely refers to a physical object"""
        # Check if it's in known object categories
        if token.lemma_ in [obj for category in self.object_categories.values() for obj in category]:
            return True
        
        # Check if it's a common physical object (nouns)
        if token.pos_ in ['NOUN', 'PROPN'] and token.pos_ != 'SCONJ':
            return True
        
        return False
    
    def _categorize_object(self, obj_text) -> str:
        """Categorize an object based on its name"""
        for category, objects in self.object_categories.items():
            if obj_text in objects:
                return category
        
        return 'other'
    
    def _parse_number(self, num_text) -> int:
        """Parse number text to integer value"""
        try:
            return int(num_text)
        except ValueError:
            # Handle written numbers
            try:
                return self.inflect_engine.number_to_words(num_text)
            except:
                return 0  # Default to 0 if can't parse
    
    def _calculate_parse_confidence(self, doc) -> float:
        """Calculate confidence in the parse based on various factors"""
        confidence = 0.5  # Base confidence
        
        # Increase confidence if we found a clear action
        if self._extract_action(doc):
            confidence += 0.3
        
        # Increase confidence based on number of objects found
        objects = self._extract_objects(doc)
        confidence += min(0.2, len(objects) * 0.05)
        
        # Consider sentence length (too short might be ambiguous, too long might be complex)
        if 3 <= len(doc) <= 15:
            confidence += 0.1
        
        return min(1.0, confidence)

# Example usage
parser = RobotCommandParser()

# Test command parsing
test_commands = [
    "Pick up the red cup from the table",
    "Go to the kitchen and bring me a bottle of water",
    "Point to the blue book on the shelf",
    "Place the box next to the chair"
]

for cmd in test_commands:
    result = parser.parse_command(cmd)
    if result:
        print(f"Command: {cmd}")
        print(f"Parsed: {result}")
        print("-" * 50)
```

### Semantic Role Labeling for Action Understanding

For more sophisticated command understanding, we can implement semantic role labeling to identify the roles of different entities in robot commands.

```python
class SemanticRoleLabeler:
    def __init__(self):
        # Define semantic roles relevant to robotics
        self.semantic_roles = {
            'Agent': 'The entity performing the action (usually the robot)',
            'Patient': 'The entity being acted upon (object to manipulate)',
            'Theme': 'The entity being moved or affected',
            'Source': 'The starting location of action',
            'Goal': 'The destination or target of action',
            'Instrument': 'The tool used to perform action',
            'Location': 'The place where action occurs',
            'Direction': 'The direction of movement',
            'Time': 'When the action should occur'
        }
    
    def label_command_roles(self, command_structure: Dict) -> Dict:
        """
        Assign semantic roles to elements in a parsed command
        """
        roles = {
            'Agent': 'robot',  # Robot is usually the agent
            'Patient': [],
            'Theme': [],
            'Source': [],
            'Goal': [],
            'Instrument': [],
            'Location': [],
            'Direction': [],
            'Time': []
        }
        
        # Assign roles based on command structure
        if command_structure['objects']:
            roles['Patient'] = command_structure['objects']
            roles['Theme'] = command_structure['objects']
        
        if command_structure['locations']:
            # Determine if locations are source or goal based on action type
            if command_structure['action'] in ['move', 'go', 'navigate']:
                roles['Goal'] = command_structure['locations']
            elif command_structure['action'] in ['grasp', 'take']:
                roles['Source'] = command_structure['locations']
        
        # Extract directional information from spatial info
        for spatial in command_structure['spatial_info']:
            if spatial['relation'] in ['left', 'right', 'front', 'back', 'above', 'below']:
                roles['Direction'].append(spatial['relation'])
        
        # Combine with original structure
        command_structure['semantic_roles'] = roles
        return command_structure

# Integration example
role_labeler = SemanticRoleLabeler()
```

---

## 10.4 Voice Command Mapping to Robot Actions

### Action Mapping Architecture

The process of mapping voice commands to robot actions requires a sophisticated architecture that can handle various command types and translate them into executable robot behaviors. This mapping must consider the robot's current state, capabilities, and environmental constraints.

```python
import asyncio
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Any, Callable
import numpy as np

class RobotActionType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    COMMUNICATION = "communication"
    WAIT = "wait"

@dataclass
class RobotCommand:
    action_type: RobotActionType
    parameters: Dict[str, Any]
    priority: int = 5  # 1-10, higher is more urgent
    timeout: float = 30.0  # seconds
    feedback_required: bool = True

class ActionMapper:
    def __init__(self, robot_interface):
        self.robot_interface = robot_interface
        
        # Action mapping rules
        self.action_mapping = {
            'move': self._map_navigation,
            'go': self._map_navigation, 
            'navigate': self._map_navigation,
            'grasp': self._map_manipulation,
            'pick up': self._map_manipulation,
            'take': self._map_manipulation,
            'place': self._map_placement,
            'put': self._map_placement,
            'look_at': self._map_perception,
            'examine': self._map_perception,
            'point_to': self._map_communication,
            'stop': self._map_wait,
            'wait': self._map_wait
        }
        
        # Capability constraints
        self.capabilities = robot_interface.get_capabilities()
    
    def map_command(self, parsed_command: Dict) -> List[RobotCommand]:
        """
        Map a parsed command to executable robot commands
        """
        action_type = parsed_command['action']
        objects = parsed_command['objects']
        spatial_info = parsed_command['spatial_info']
        locations = parsed_command['locations']
        modifiers = parsed_command['modifiers']
        
        if action_type in self.action_mapping:
            # Call the appropriate mapping function
            robot_commands = self.action_mapping[action_type](
                objects, spatial_info, locations, modifiers, parsed_command
            )
            return robot_commands
        else:
            raise ValueError(f"Unknown action type: {action_type}")
    
    def _map_navigation(self, objects, spatial_info, locations, modifiers, original_cmd) -> List[RobotCommand]:
        """Map navigation-related commands to robot actions"""
        commands = []
        
        # Determine destination
        destination = None
        if locations:
            destination = self._resolve_location(locations[0])
        elif spatial_info:
            # Use spatial information to determine navigation target
            destination = self._interpret_spatial_navigation(spatial_info)
        
        if destination:
            commands.append(RobotCommand(
                action_type=RobotActionType.NAVIGATION,
                parameters={'destination': destination, 'speed': 0.5},
                priority=7,
                timeout=60.0
            ))
        
        return commands
    
    def _map_manipulation(self, objects, spatial_info, locations, modifiers, original_cmd) -> List[RobotCommand]:
        """Map manipulation-related commands to robot actions"""
        commands = []
        
        if not objects:
            print("No object specified for manipulation command")
            return commands
        
        # Find the target object
        target_object = objects[0]  # Use first object as primary target
        
        # Get object location (may need perception to locate it)
        commands.append(RobotCommand(
            action_type=RobotActionType.PERCEPTION,
            parameters={'search_for': target_object['text'], 'search_type': 'object'},
            priority=9,
            timeout=10.0
        ))
        
        # Grasp action
        commands.append(RobotCommand(
            action_type=RobotActionType.MANIPULATION,
            parameters={'action': 'grasp', 'object': target_object, 'location': 'auto'},
            priority=8,
            timeout=15.0
        ))
        
        return commands
    
    def _map_placement(self, objects, spatial_info, locations, modifiers, original_cmd) -> List[RobotCommand]:
        """Map placement-related commands to robot actions"""
        commands = []
        
        # Usually involves placing a held object
        if locations:
            place_location = self._resolve_location(locations[0])
            commands.append(RobotCommand(
                action_type=RobotActionType.MANIPULATION,
                parameters={'action': 'place', 'location': place_location},
                priority=8,
                timeout=15.0
            ))
        
        return commands
    
    def _map_perception(self, objects, spatial_info, locations, modifiers, original_cmd) -> List[RobotCommand]:
        """Map perception-related commands to robot actions"""
        commands = []
        
        if objects:
            target = objects[0]['text']
            commands.append(RobotCommand(
                action_type=RobotActionType.PERCEPTION,
                parameters={'action': 'look_at', 'target': target},
                priority=6,
                timeout=10.0
            ))
        elif spatial_info:
            direction = spatial_info[0].get('reference_point', 'forward')
            commands.append(RobotCommand(
                action_type=RobotActionType.PERCEPTION,
                parameters={'action': 'look_direction', 'direction': direction},
                priority=6,
                timeout=10.0
            ))
        
        return commands
    
    def _map_communication(self, objects, spatial_info, locations, modifiers, original_cmd) -> List[RobotCommand]:
        """Map communication-related commands (pointing, gestures)"""
        commands = []
        
        if objects:
            target = objects[0]['text']
            commands.append(RobotCommand(
                action_type=RobotActionType.COMMUNICATION,
                parameters={'action': 'point_to', 'target': target},
                priority=5,
                timeout=5.0
            ))
        
        return commands
    
    def _map_wait(self, objects, spatial_info, locations, modifiers, original_cmd) -> List[RobotCommand]:
        """Map wait-related commands"""
        commands = []
        
        # Determine wait duration from modifiers if available
        duration = 5.0  # default
        for mod in modifiers:
            if mod['type'] == 'NUM':
                duration = float(mod['value'])
                break
        
        commands.append(RobotCommand(
            action_type=RobotActionType.WAIT,
            parameters={'duration': duration},
            priority=3,
            timeout=duration + 2.0
        ))
        
        return commands
    
    def _resolve_location(self, location_text: str) -> Dict:
        """Resolve a location reference to coordinates or map location"""
        # In a real implementation, this would query a map or spatial memory
        # For now, using a simple example resolution
        location_map = {
            'kitchen': {'x': 5.0, 'y': 3.0, 'z': 0.0},
            'living room': {'x': 2.0, 'y': 1.0, 'z': 0.0},
            'bedroom': {'x': 8.0, 'y': 1.0, 'z': 0.0},
            'office': {'x': 4.0, 'y': 6.0, 'z': 0.0},
        }
        
        return location_map.get(location_text.lower(), {'x': 0.0, 'y': 0.0, 'z': 0.0})
    
    def _interpret_spatial_navigation(self, spatial_info: List[Dict]) -> Dict:
        """Interpret spatial relationships for navigation"""
        # This would implement spatial reasoning for navigation
        # Simplified example
        if spatial_info:
            rel_info = spatial_info[0]
            if rel_info['relation'] in ['left', 'right', 'front', 'back']:
                # Move in the relative direction from current position
                current_pos = self.robot_interface.get_current_position()
                if rel_info['relation'] == 'left':
                    return {'x': current_pos['x'] - 1.0, 'y': current_pos['y'], 'z': current_pos['z']}
                elif rel_info['relation'] == 'right':
                    return {'x': current_pos['x'] + 1.0, 'y': current_pos['y'], 'z': current_pos['z']}
                # ... other directions
        
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}

class RobotActionExecutor:
    def __init__(self, robot_interface, action_mapper):
        self.robot_interface = robot_interface
        self.action_mapper = action_mapper
        self.command_queue = asyncio.Queue()
        self.action_history = []
    
    async def execute_command(self, parsed_command: Dict) -> Dict:
        """
        Execute a fully parsed voice command
        """
        try:
            # Map the command to robot actions
            robot_commands = self.action_mapper.map_command(parsed_command)
            
            # Execute each command in sequence
            results = []
            for cmd in robot_commands:
                result = await self._execute_single_command(cmd)
                results.append(result)
                
                # Check for failure - stop execution if needed
                if not result.get('success', True):
                    break
            
            execution_result = {
                'success': all(r.get('success', False) for r in results),
                'command': parsed_command,
                'actions': results,
                'timestamp': time.time()
            }
            
            self.action_history.append(execution_result)
            return execution_result
            
        except Exception as e:
            error_result = {
                'success': False,
                'error': str(e),
                'command': parsed_command,
                'actions': [],
                'timestamp': time.time()
            }
            self.action_history.append(error_result)
            return error_result
    
    async def _execute_single_command(self, robot_cmd: RobotCommand) -> Dict:
        """Execute a single robot command"""
        start_time = time.time()
        
        try:
            if robot_cmd.action_type == RobotActionType.NAVIGATION:
                result = await self._execute_navigation(robot_cmd)
            elif robot_cmd.action_type == RobotActionType.MANIPULATION:
                result = await self._execute_manipulation(robot_cmd)
            elif robot_cmd.action_type == RobotActionType.PERCEPTION:
                result = await self._execute_perception(robot_cmd)
            elif robot_cmd.action_type == RobotActionType.COMMUNICATION:
                result = await self._execute_communication(robot_cmd)
            elif robot_cmd.action_type == RobotActionType.WAIT:
                result = await self._execute_wait(robot_cmd)
            else:
                result = {'success': False, 'error': 'Unknown action type'}
        
        except asyncio.TimeoutError:
            result = {'success': False, 'error': 'Command timed out'}
        except Exception as e:
            result = {'success': False, 'error': f'Execution error: {str(e)}'}
        
        # Add execution time
        result['execution_time'] = time.time() - start_time
        result['command'] = robot_cmd
        
        return result
    
    async def _execute_navigation(self, robot_cmd: RobotCommand) -> Dict:
        """Execute navigation command"""
        destination = robot_cmd.parameters['destination']
        speed = robot_cmd.parameters.get('speed', 0.5)
        
        # In real implementation, this would call navigation stack
        # For simulation:
        print(f"Navigating to destination: {destination}")
        
        # Simulate navigation completion
        await asyncio.sleep(1.0)  # Simulate navigation time
        
        return {'success': True, 'destination': destination, 'path_length': 2.0}
    
    async def _execute_manipulation(self, robot_cmd: RobotCommand) -> Dict:
        """Execute manipulation command"""
        action = robot_cmd.parameters['action']
        obj = robot_cmd.parameters.get('object', 'unknown object')
        
        print(f"Performing manipulation: {action} {obj}")
        
        # Simulate manipulation completion
        await asyncio.sleep(2.0)  # Simulate manipulation time
        
        return {'success': True, 'action': action, 'object': obj}
    
    async def _execute_perception(self, robot_cmd: RobotCommand) -> Dict:
        """Execute perception command"""
        action = robot_cmd.parameters.get('action', 'look')
        target = robot_cmd.parameters.get('target', 'environment')
        
        print(f"Performing perception: {action} at {target}")
        
        # Simulate perception completion
        await asyncio.sleep(0.5)  # Simulate perception time
        
        # Simulate perception results
        perception_data = {
            'objects_detected': 3,
            'target_found': True,
            'confidence': 0.85
        }
        
        return {'success': True, 'action': action, 'target': target, 'data': perception_data}
    
    async def _execute_communication(self, robot_cmd: RobotCommand) -> Dict:
        """Execute communication command"""
        action = robot_cmd.parameters['action']
        target = robot_cmd.parameters.get('target', 'unknown')
        
        print(f"Performing communication: {action} at {target}")
        
        # Simulate communication completion
        await asyncio.sleep(0.3)
        
        return {'success': True, 'action': action, 'target': target}
    
    async def _execute_wait(self, robot_cmd: RobotCommand) -> Dict:
        """Execute wait command"""
        duration = robot_cmd.parameters['duration']
        
        print(f"Waiting for {duration} seconds")
        
        await asyncio.sleep(duration)
        
        return {'success': True, 'duration': duration}

# Example integration
class VoiceCommandSystem:
    def __init__(self, robot_interface):
        self.speech_recognizer = SpeechRecognizer()
        self.audio_preprocessor = AudioPreprocessor()
        self.command_parser = RobotCommandParser()
        self.role_labeler = SemanticRoleLabeler()
        self.action_mapper = ActionMapper(robot_interface)
        self.action_executor = RobotActionExecutor(robot_interface, self.action_mapper)
        
        # Real-time processing components
        self.realtime_processor = RealTimeVoiceProcessor(
            self.speech_recognizer, 
            self.audio_preprocessor
        )
    
    async def process_voice_command(self, audio_chunk=None) -> Dict:
        """Process a voice command from audio to action execution"""
        if audio_chunk is not None:
            # Process real-time audio
            processed_audio = self.realtime_processor.process_realtime_audio(audio_chunk)
        else:
            # Wait for command from microphone
            print("Listening for voice command...")
            self.speech_recognizer.start_listening()
            
            # In a real implementation, you would have a callback mechanism
            # For now, simulate with a recognized command
            recognized_text = "Go to the kitchen and pick up the red cup"
            parsed = self.command_parser.parse_command(recognized_text)
            
            if parsed:
                # Apply semantic role labeling
                labeled_command = self.role_labeler.label_command_roles(parsed)
                
                # Execute the command
                result = await self.action_executor.execute_command(labeled_command)
                
                return result
            else:
                return {'success': False, 'error': 'Could not parse command'}
```

### Context-Aware Command Resolution

Physical AI systems need to resolve ambiguous commands using environmental context and current robot state.

```python
class ContextResolver:
    def __init__(self):
        self.robot_state = {}
        self.environment_map = {}
        self.object_memory = {}
        self.spatial_memory = {}
    
    def resolve_ambiguous_command(self, parsed_command: Dict, context: Dict) -> Dict:
        """
        Resolve ambiguous elements in a command using context
        """
        resolved_command = parsed_command.copy()
        
        # Resolve ambiguous objects using environment context
        for i, obj in enumerate(parsed_command['objects']):
            resolved_obj = self._resolve_object_reference(obj, context)
            resolved_command['objects'][i] = resolved_obj
        
        # Resolve ambiguous locations using spatial context
        for i, location in enumerate(parsed_command['locations']):
            resolved_location = self._resolve_location_reference(location, context)
            resolved_command['locations'][i] = resolved_location
        
        # Resolve spatial relationships using current robot state
        for i, spatial in enumerate(parsed_command['spatial_info']):
            resolved_spatial = self._resolve_spatial_reference(spatial, context)
            resolved_command['spatial_info'][i] = resolved_spatial
        
        return resolved_command
    
    def _resolve_object_reference(self, obj: Dict, context: Dict) -> Dict:
        """
        Resolve object reference using environment context
        """
        # Get all objects currently perceived
        perceived_objects = context.get('perceived_objects', [])
        
        # Look for objects matching the description
        candidates = []
        for p_obj in perceived_objects:
            if self._match_object_description(obj, p_obj):
                candidates.append(p_obj)
        
        if candidates:
            # Use the closest or most recently seen object
            closest = min(candidates, key=lambda x: x.get('distance', float('inf')))
            obj['resolved_object'] = closest
            obj['confidence'] = 0.9  # High confidence in resolution
        else:
            obj['confidence'] = 0.3  # Low confidence, object not found
            obj['resolved_object'] = None
        
        return obj
    
    def _resolve_location_reference(self, location: str, context: Dict) -> str:
        """
        Resolve location reference using spatial memory
        """
        # Check if location is in known spatial memory
        known_locations = context.get('known_locations', {})
        
        if location in known_locations:
            return known_locations[location]
        
        # Fallback to map-based resolution
        # This would involve spatial reasoning to match natural language
        # to specific coordinates or map locations
        return location  # Return original if cannot resolve
    
    def _resolve_spatial_reference(self, spatial: Dict, context: Dict) -> Dict:
        """
        Resolve spatial reference using robot state and environment
        """
        # Get robot's current pose
        robot_pose = context.get('robot_pose', {'x': 0, 'y': 0, 'theta': 0})
        
        # Calculate actual coordinates based on relative reference
        if spatial['relation'] in ['left', 'right', 'front', 'back']:
            angle_offset = self._get_direction_offset(spatial['relation'])
            target_angle = robot_pose['theta'] + angle_offset
            target_x = robot_pose['x'] + np.cos(target_angle) * 1.0  # 1m offset
            target_y = robot_pose['y'] + np.sin(target_angle) * 1.0
            
            spatial['resolved_coordinates'] = {'x': target_x, 'y': target_y}
        
        return spatial
    
    def _match_object_description(self, target_desc: Dict, candidate_obj: Dict) -> bool:
        """
        Match an object description to a candidate object
        """
        # Check if object name matches
        if target_desc['lemma'] == candidate_obj.get('name', '').lower():
            return True
        
        # Check category match
        if target_desc.get('category') == candidate_obj.get('category'):
            # Check color or other modifiers
            target_color = self._extract_color(target_desc.get('modifiers', []))
            candidate_color = candidate_obj.get('color')
            
            if target_color and candidate_color:
                return target_color.lower() == candidate_color.lower()
            else:
                return True  # Category match without color conflict
        
        return False
    
    def _extract_color(self, modifiers: List[str]) -> str:
        """Extract color from object modifiers"""
        color_words = ['red', 'blue', 'green', 'yellow', 'black', 'white', 'orange', 'purple', 'pink', 'gray']
        for mod in modifiers:
            if mod.lower() in color_words:
                return mod.lower()
        return None
    
    def _get_direction_offset(self, direction: str) -> float:
        """Get angle offset for relative directions"""
        offsets = {
            'front': 0.0,
            'ahead': 0.0,
            'forward': 0.0,
            'front_left': np.pi/4,
            'front_right': -np.pi/4,
            'left': np.pi/2,
            'right': -np.pi/2,
            'back_left': 3*np.pi/4,
            'back_right': -3*np.pi/4,
            'back': np.pi
        }
        return offsets.get(direction, 0.0)
```

---

## 10.5 Robustness and Error Handling

### Handling Recognition Errors

Voice-to-action systems must gracefully handle recognition errors and provide appropriate feedback to users. Error handling strategies include confidence-based filtering, error correction, and user clarification requests.

```python
class RobustVoiceProcessor:
    def __init__(self):
        self.confidence_threshold = 0.7
        self.recent_errors = []
        self.user_preference_model = {}
        
    def handle_recognition_result(self, recognition_result: Dict) -> Dict:
        """
        Handle speech recognition result with robustness checks
        """
        if not recognition_result:
            return self._handle_empty_recognition()
        
        confidence = recognition_result.get('confidence', 0.0)
        text = recognition_result.get('text', '')
        
        if confidence < self.confidence_threshold:
            return self._handle_low_confidence(text, confidence)
        
        # Check for common recognition errors
        corrected_text = self._correct_common_errors(text)
        
        return {
            'success': True,
            'text': corrected_text,
            'confidence': confidence,
            'original_text': text
        }
    
    def _handle_empty_recognition(self) -> Dict:
        """Handle case where no speech was recognized"""
        return {
            'success': False,
            'error': 'no_speech_detected',
            'suggested_action': 'try_again',
            'confidence': 0.0
        }
    
    def _handle_low_confidence(self, text: str, confidence: float) -> Dict:
        """Handle low-confidence recognition results"""
        # Check if text makes semantic sense
        semantic_validity = self._check_semantic_validity(text)
        
        if semantic_validity > 0.3:  # Even low confidence, but semantically valid
            return {
                'success': True,
                'text': text,
                'confidence': confidence,
                'warning': 'low_confidence',
                'semantic_validity': semantic_validity
            }
        else:
            return {
                'success': False,
                'error': 'low_confidence_and_semantic_invalid',
                'original_text': text,
                'confidence': confidence,
                'suggested_action': 'clarify_or_repeat'
            }
    
    def _correct_common_errors(self, text: str) -> str:
        """
        Correct common speech recognition errors
        """
        # Common error patterns and corrections
        corrections = {
            # Numbers that sound similar
            r'\bfor\b': 'four',
            r'\btoo\b': 'two',
            r'\bwon\b': 'one',  # "won" vs "one"
            
            # Words that sound similar
            r'\bhallway\b': 'hally',  # If "hallway" is misrecognized as "hally" in some accents
            r'\bcomputer\b': 'compter',  # Common misrecognition
        }
        
        corrected_text = text
        for pattern, correction in corrections.items():
            corrected_text = re.sub(pattern, correction, corrected_text, flags=re.IGNORECASE)
        
        return corrected_text
    
    def _check_semantic_validity(self, text: str) -> float:
        """
        Check if recognized text has semantic validity for robot commands
        """
        # Check if text contains known robot commands
        robot_command_keywords = [
            'move', 'go', 'navigate', 'pick', 'take', 'grasp', 'place', 'put',
            'look', 'see', 'examine', 'point', 'stop', 'wait', 'bring', 'fetch'
        ]
        
        # Count command-related words
        words = text.lower().split()
        command_words = [word for word in words if any(cmd in word for cmd in robot_command_keywords)]
        
        if len(command_words) >= 1:
            return min(1.0, len(command_words) / len(words) * 2)  # Boost for command density
        
        # Check for object references
        object_indicators = ['the', 'a', 'an', 'my', 'your', 'this', 'that', 'these', 'those']
        object_words = [word for word in words if word in object_indicators]
        
        return min(0.5, len(object_words) / len(words)) if object_words else 0.0
    
    def request_clarification(self, command: Dict, available_options: List[str] = None) -> Dict:
        """
        Request user clarification for ambiguous commands
        """
        if available_options:
            clarification_text = f"I heard '{command.get('original_text', '')}' but I'm not sure what you mean. Did you mean: {', '.join(available_options)}?"
        else:
            clarification_text = f"I heard '{command.get('original_text', '')}' but I'm not sure how to help with that. Could you rephrase?"
        
        return {
            'action': 'request_clarification',
            'message': clarification_text,
            'original_command': command,
            'options': available_options
        }
    
    def learn_from_errors(self, original_recognition: Dict, user_correction: str):
        """
        Learn from user corrections to improve future recognition
        """
        # This would implement learning from user corrections
        # In a real system, this would update acoustic or language models
        pass
```

### Multi-Modal Error Recovery

Physical AI systems can leverage multiple modalities to recover from voice recognition errors by combining visual, spatial, and contextual information.

```python
class MultiModalErrorRecovery:
    def __init__(self):
        self.perception_system = None
        self.context_resolver = ContextResolver()
    
    def handle_command_error(self, original_command: Dict, error_info: Dict, context: Dict) -> Dict:
        """
        Handle command execution errors using multi-modal recovery
        """
        error_type = error_info.get('error_type', 'unknown')
        
        if error_type == 'object_not_found':
            return self._recover_object_not_found(original_command, context)
        elif error_type == 'location_ambiguous':
            return self._recover_location_ambiguous(original_command, context)
        elif error_type == 'action_unsupported':
            return self._recover_action_unsupported(original_command, context)
        else:
            return self._general_recovery(original_command, error_info, context)
    
    def _recover_object_not_found(self, command: Dict, context: Dict) -> Dict:
        """
        Recover when target object is not found using perception
        """
        # Request active perception to locate possible objects
        requested_object = command.get('objects', [{}])[0] if command.get('objects') else {}
        
        if requested_object:
            search_params = {
                'object_type': requested_object.get('lemma', ''),
                'search_area': context.get('current_location', 'around_robot'),
                'attributes': requested_object.get('modifiers', [])
            }
            
            # Request perception system to search for objects
            perception_request = {
                'action': 'search_for_object',
                'parameters': search_params,
                'timeout': 10.0
            }
            
            return {
                'recovery_action': 'perception_search',
                'request': perception_request,
                'original_command': command
            }
    
    def _recover_location_ambiguous(self, command: Dict, context: Dict) -> Dict:
        """
        Recover when location reference is ambiguous using spatial reasoning
        """
        # Use spatial context to disambiguate location
        locations = command.get('locations', [])
        if not locations:
            return {'recovery_action': 'ask_for_clarification', 'message': 'Could you specify where exactly?'}
        
        # Get all possible locations that match the description
        possible_locations = self._get_possible_locations(locations[0], context)
        
        if len(possible_locations) == 1:
            # Disambiguated successfully
            command['locations'] = [possible_locations[0]]
            return {
                'recovery_action': 'retry_command',
                'resolved_command': command
            }
        elif len(possible_locations) > 1:
            # Multiple possibilities, ask user to choose
            return {
                'recovery_action': 'request_choice',
                'options': possible_locations,
                'message': f"I found multiple locations that match '{locations[0]}'. Which one did you mean?"
            }
        else:
            # No matches found
            return {
                'recovery_action': 'ask_for_alternative',
                'message': f"I don't know a location called '{locations[0]}'. Could you describe it differently?"
            }
    
    def _recover_action_unsupported(self, command: Dict, context: Dict) -> Dict:
        """
        Recover when requested action is not supported
        """
        requested_action = command.get('action')
        
        # Provide alternatives based on robot capabilities
        alternatives = self._get_action_alternatives(requested_action, context)
        
        if alternatives:
            return {
                'recovery_action': 'suggest_alternatives',
                'alternatives': alternatives,
                'message': f"I can't {requested_action}, but I can {', '.join(alternatives)}. Would you like me to do one of those instead?"
            }
        else:
            return {
                'recovery_action': 'inform_limitation',
                'message': f"Sorry, I'm not able to {requested_action}. Is there something else I can help with?"
            }
    
    def _general_recovery(self, command: Dict, error_info: Dict, context: Dict) -> Dict:
        """
        General error recovery approach
        """
        # Try to understand intent and suggest alternatives
        suggested_repair = self._suggest_command_repair(command, error_info, context)
        
        if suggested_repair:
            return {
                'recovery_action': 'suggest_repair',
                'suggested_command': suggested_repair,
                'message': f"Did you mean: '{suggested_repair}'?"
            }
        else:
            return {
                'recovery_action': 'request_rephrase',
                'message': "I'm not sure how to help with that. Could you rephrase the command?"
            }
    
    def _get_possible_locations(self, location_desc: str, context: Dict) -> List[str]:
        """Get possible locations matching a description"""
        # This would integrate with spatial memory system
        # For demo purposes, returning some options
        if 'room' in location_desc.lower():
            return ['kitchen', 'living room', 'bedroom', 'office']
        elif 'table' in location_desc.lower():
            return ['kitchen table', 'dining table', 'desk']
        else:
            return []
    
    def _get_action_alternatives(self, requested_action: str, context: Dict) -> List[str]:
        """Get alternative actions for unsupported action"""
        # This would check robot capabilities and suggest alternatives
        capability_map = {
            'fly': ['move', 'navigate'],
            'jump': ['step_over', 'climb'],
            'invisibility': ['hide', 'wait'],
            'teleport': ['navigate', 'go']
        }
        
        alternatives = capability_map.get(requested_action, [])
        
        # Filter by actually available capabilities
        available_caps = context.get('robot_capabilities', [])
        return [alt for alt in alternatives if alt in available_caps]
    
    def _suggest_command_repair(self, command: Dict, error_info: Dict, context: Dict) -> str:
        """Suggest a repair for the command"""
        # Use context and error info to suggest a repair
        # This would implement more sophisticated NLP-based repair
        original_text = command.get('original_text', '')
        
        # Simple rule-based repairs
        repairs = {
            r'bring me the (.+) that (.+)': r'get the \1 that \2',
            r'go and (.+)': r'go to \1',
            r'pick up (.+) and move to (.+)': r'take \1 to \2'
        }
        
        for pattern, replacement in repairs.items():
            import re
            if re.search(pattern, original_text, re.IGNORECASE):
                repaired = re.sub(pattern, replacement, original_text, re.IGNORECASE)
                return repaired
        
        return None
```

---

## 10.6 Integration with Higher-Level Planning Systems

### Voice Command Integration Architecture

The integration between voice command systems and higher-level planning systems requires careful coordination to ensure that voice commands are properly understood, planned, and executed in the context of overall robot goals and capabilities.

```python
class VoiceIntegrationManager:
    def __init__(self, voice_system, planning_system, execution_system):
        self.voice_system = voice_system
        self.planning_system = planning_system
        self.execution_system = execution_system
        
        self.command_history = []
        self.pending_commands = {}
        self.user_intents = {}
        
    def process_voice_command_as_task(self, command_text: str, user_id: str = "default") -> Dict:
        """
        Process voice command as a high-level task in the planning system
        """
        # Parse the voice command
        parsed_command = self.voice_system.command_parser.parse_command(command_text)
        if not parsed_command:
            return {'success': False, 'error': 'Could not parse command'}
        
        # Apply semantic role labeling
        labeled_command = self.voice_system.role_labeler.label_command_roles(parsed_command)
        
        # Resolve any ambiguities using context
        context = self._get_current_context()
        resolved_command = self.voice_system.context_resolver.resolve_ambiguous_command(
            labeled_command, context
        )
        
        # Convert to planning task
        planning_task = self._convert_to_planning_task(resolved_command, user_id)
        
        # Submit to planning system
        plan_result = self.planning_system.create_plan(planning_task)
        
        if plan_result['success']:
            # Execute the plan
            execution_result = self.execution_system.execute_plan(plan_result['plan'])
            
            # Update command history
            command_record = {
                'command_text': command_text,
                'parsed_command': resolved_command,
                'planning_task': planning_task,
                'plan': plan_result['plan'],
                'execution_result': execution_result,
                'timestamp': time.time(),
                'user_id': user_id
            }
            self.command_history.append(command_record)
            
            return {
                'success': True,
                'planning_result': plan_result,
                'execution_result': execution_result,
                'task_id': plan_result.get('task_id')
            }
        else:
            return {
                'success': False,
                'planning_error': plan_result.get('error', 'Planning failed'),
                'command_text': command_text
            }
    
    def _convert_to_planning_task(self, resolved_command: Dict, user_id: str) -> Dict:
        """
        Convert voice command to planning system task format
        """
        action = resolved_command['action']
        
        # Map voice command action to planning task
        task_mapping = {
            'move': 'navigation_task',
            'grasp': 'manipulation_task', 
            'place': 'placement_task',
            'bring': 'delivery_task',
            'look_at': 'perception_task'
        }
        
        task_type = task_mapping.get(action, 'general_task')
        
        task = {
            'task_type': task_type,
            'action': action,
            'objects': resolved_command.get('objects', []),
            'locations': resolved_command.get('locations', []),
            'spatial_info': resolved_command.get('spatial_info', []),
            'user_id': user_id,
            'priority': self._determine_priority(resolved_command),
            'request_time': time.time()
        }
        
        # Add task-specific parameters
        if action in ['grasp', 'take']:
            task['target_object'] = resolved_command['objects'][0] if resolved_command['objects'] else None
        elif action in ['move', 'go', 'navigate']:
            task['destination'] = resolved_command['locations'][0] if resolved_command['locations'] else None
        elif action in ['bring', 'fetch']:
            task['delivery_target'] = user_id  # Deliver to the user
            task['pickup_object'] = resolved_command['objects'][0] if resolved_command['objects'] else None
        
        return task
    
    def _determine_priority(self, command: Dict) -> int:
        """Determine priority of voice command based on various factors"""
        priority = 5  # Default medium priority
        
        # Increase priority for safety-related commands
        safety_keywords = ['stop', 'emergency', 'danger', 'help']
        command_lower = command.get('original_text', '').lower()
        
        if any(keyword in command_lower for keyword in safety_keywords):
            priority = 10  # Highest priority
        
        # Adjust based on spatial relationships (immediate vicinity)
        spatial_info = command.get('spatial_info', [])
        if any(s['relation'] in ['here', 'this', 'near', 'close'] for s in spatial_info):
            priority += 1
        
        return min(10, max(1, priority))  # Clamp to 1-10 range
    
    def _get_current_context(self) -> Dict:
        """Get current context for command resolution"""
        return {
            'robot_pose': self.execution_system.get_robot_state()['pose'],
            'perceived_objects': self.execution_system.get_perceived_objects(),
            'known_locations': self.execution_system.get_known_locations(),
            'robot_capabilities': self.execution_system.get_capabilities(),
            'current_plan': self.planning_system.get_active_plan(),
            'task_queue': self.planning_system.get_task_queue()
        }
    
    def handle_conversation_context(self, command_text: str, user_id: str = "default") -> Dict:
        """
        Handle voice commands in the context of ongoing conversation
        """
        # Check if this continues a previous command
        if user_id in self.user_intents and self.user_intents[user_id]['status'] == 'awaiting_clarification':
            # Handle as clarification to previous intent
            previous_intent = self.user_intents[user_id]
            response = self._handle_continuation(command_text, previous_intent)
            return response
        
        # Otherwise process as new command
        return self.process_voice_command_as_task(command_text, user_id)
    
    def _handle_continuation(self, response_text: str, previous_intent: Dict) -> Dict:
        """Handle response that continues a previous intent"""
        if previous_intent['expected_response_type'] == 'yes_no':
            if self._is_affirmative(response_text):
                # Continue with original command
                return self.process_voice_command_as_task(previous_intent['original_command'], previous_intent['user_id'])
            else:
                # Cancel or modify command
                return {'success': True, 'message': 'Command cancelled', 'cancelled': True}
        elif previous_intent['expected_response_type'] == 'choice':
            # Parse the choice from response_text
            choice = self._extract_choice(response_text, previous_intent['options'])
            if choice:
                # Execute command with chosen option
                return self._execute_with_choice(choice, previous_intent)
            else:
                return {'success': False, 'error': 'Could not understand your choice'}
    
    def _is_affirmative(self, text: str) -> bool:
        """Check if text is an affirmative response"""
        affirmative = ['yes', 'yep', 'sure', 'ok', 'okay', 'go ahead', 'please']
        return any(word in text.lower() for word in affirmative)
    
    def _extract_choice(self, response: str, options: List[str]) -> str:
        """Extract choice from user response"""
        # Simple approach: check if any option is in the response
        response_lower = response.lower()
        for option in options:
            if option.lower() in response_lower:
                return option
        return None
    
    def _execute_with_choice(self, choice: str, previous_intent: Dict) -> Dict:
        """Execute command with user's choice"""
        # In practice, this would modify the original command based on the choice
        # For now, return a success message
        return {
            'success': True,
            'message': f'Executing with choice: {choice}',
            'choice': choice,
            'original_intent': previous_intent
        }

# Example usage in a complete system
class CompleteVoiceToActionSystem:
    def __init__(self):
        # Initialize all components
        self.robot_interface = MockRobotInterface()  # Would be real robot interface
        self.voice_system = VoiceCommandSystem(self.robot_interface)
        self.planning_system = MockPlanningSystem()  # Would be real planning system
        self.execution_system = MockExecutionSystem()  # Would be real execution system
        
        # Integration manager
        self.integration_manager = VoiceIntegrationManager(
            self.voice_system,
            self.planning_system, 
            self.execution_system
        )
        
        # Error recovery
        self.error_recovery = MultiModalErrorRecovery()
        
        print("Complete Voice-to-Action system initialized")
    
    async def process_command_with_recovery(self, command_text: str, user_id: str = "default") -> Dict:
        """
        Process command with full error recovery capabilities
        """
        try:
            # First attempt
            result = self.integration_manager.handle_conversation_context(command_text, user_id)
            
            if result.get('success'):
                return result
            else:
                # Handle specific errors with recovery
                error_type = result.get('error', 'general_failure')
                context = self.integration_manager._get_current_context()
                
                recovery_result = self.error_recovery.handle_command_error(
                    {'original_text': command_text},
                    {'error_type': error_type},
                    context
                )
                
                return {
                    'original_result': result,
                    'recovery_attempt': recovery_result,
                    'needs_user_input': recovery_result.get('recovery_action') in ['request_clarification', 'request_choice']
                }
        
        except Exception as e:
            return {
                'success': False,
                'system_error': str(e),
                'command': command_text
            }

# Mock classes for demonstration
class MockRobotInterface:
    def get_capabilities(self):
        return ['navigation', 'manipulation', 'perception']
    
    def get_current_position(self):
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}

class MockPlanningSystem:
    def create_plan(self, task):
        return {'success': True, 'plan': [{'action': 'demo_action', 'params': {}}], 'task_id': 'task_123'}
    
    def get_active_plan(self):
        return None
    
    def get_task_queue(self):
        return []

class MockExecutionSystem:
    def execute_plan(self, plan):
        return {'success': True, 'actions_completed': 1}
    
    def get_robot_state(self):
        return {'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0}}
    
    def get_perceived_objects(self):
        return [{'name': 'red cup', 'category': 'container', 'color': 'red', 'distance': 1.5}]
    
    def get_known_locations(self):
        return {'kitchen': {'x': 5.0, 'y': 3.0}, 'living room': {'x': 2.0, 'y': 1.0}}
    
    def get_capabilities(self):
        return ['navigation', 'manipulation', 'perception']
```

---

## Summary

This chapter has provided a comprehensive exploration of voice-to-action integration for Physical AI and humanoid robotics systems:

1. **Introduction to voice-to-action**: Understanding the role of speech interfaces in natural human-robot interaction
2. **Speech recognition and audio processing**: Implementing modern systems with Whisper and audio preprocessing for robot environments
3. **Natural language understanding**: Parsing natural language commands and extracting semantic meaning
4. **Action mapping**: Converting voice commands to executable robot behaviors with proper context resolution
5. **Error handling and robustness**: Implementing multi-modal error recovery and user interaction management
6. **System integration**: Connecting voice systems with planning and execution frameworks for seamless operation

The key insight from this chapter is that successful voice-to-action integration requires more than just speech recognition; it needs sophisticated natural language understanding, context-aware resolution of ambiguities, and tight integration with robot planning and execution systems. The system must handle the inherent ambiguity of natural language while maintaining the precision required for physical robot control.

## Key Terms

- **Voice-to-Action**: Systems that convert spoken language commands to robotic actions
- **Automatic Speech Recognition (ASR)**: Technology that converts speech to text
- **Natural Language Understanding (NLU)**: Processing text to extract meaning and intent
- **Semantic Role Labeling**: Identifying the roles of different entities in actions
- **Confidence Scoring**: Measuring reliability of speech recognition and command parsing
- **Multi-Modal Integration**: Combining speech with visual and spatial information
- **Context Resolution**: Using environmental context to disambiguate commands
- **Error Recovery**: Strategies for handling recognition and execution failures
- **Voice Activity Detection (VAD)**: Identifying when speech is present in audio
- **Acoustic Model**: Component that maps audio features to linguistic units
- **Language Model**: Component that captures linguistic probability of word sequences

## Further Reading

- Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." arXiv preprint.
- Jurafsky, D., & Martin, J. H. (2020). "Speech and Language Processing" (3rd ed.). Pearson.
- Chen, G., et al. (2022). "Transformer-Based Acoustic Modeling for Speech Recognition." IEEE Transactions on Audio, Speech, and Language Processing.
- Young, S., et al. (2013). "The Dialog State Tracking Challenge." SIGDIAL Conference.
- Kollar, T., et al. (2010). "Toward Understanding Natural Language Spatial Commands for Human-Robot Interaction." IEEE International Conference on Robotics and Automation.

---

**Chapter 11 Preview**: In the next chapter, we will explore LLM Cognitive Planning for Physical AI systems, examining how large language models can serve as high-level cognitive planners that decompose complex tasks into sequences of executable actions for embodied robots.