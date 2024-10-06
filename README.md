# Audio Streamer ROS2 Package

## Overview
`audio_streamer` is a ROS2 package that enables real-time audio streaming. It includes two main components:
1. **Audio Stream Publisher**: Captures audio from a specified hardware device and publishes it as a ROS2 topic.
2. **Audio Stream Player**: Subscribes to the audio stream topic and plays the received audio data on the default speaker.

This package uses the ['sounddevice'](https://github.com/spatialaudio/python-sounddevice) python library to manage audio input and output streams, with support for hardware-specific audio devices and configurations such as sample rates and channels. It also includes a custom message package, `audio_msgs`, for defining the audio data.

(Tested on ROS2 Humble)

## Features
- **Audio Capture**: Captures audio from a microphone or other input device.
- **Real-Time Audio Streaming**: Publishes the captured audio as a ROS2 topic 
- **Audio Playback**: Subscribes to the audio stream and plays it back in real-time using the default system audio output (e.g., speakers).
- **Custom Audio Message**: The package includes `audio_msgs`, a custom message for handling audio data in ROS2.


## Typical Installation

1. **Clone the Repository**
   ```bash
   mkdir -p ros2_ws/src
   cd ~/ros2_ws/src
   https://github.com/atarbabgei/audio_streamer.git
   ```

2. **Install Dependencies**
   Install the required Python dependencies using `pip`:
   ```bash
   pip3 install numpy sounddevice
   ```

3. **Build the Package**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Nodes

### 1. Audio Stream Publisher

**Node Name**: `audio_stream_publisher`

This node captures audio from the specified hardware input device (e.g., `hw:1,0`) and publishes it to the `audio_stream` topic in real-time.

#### Parameters:
- `input_hw_id`: Hardware ID for the input device (default: `hw:1,0`).
- `sample_rate`: Sample rate for audio recording (default: 48000 Hz).
- `channels`: Number of audio channels (default: 1).

#### Published Topic:
- `audio_stream` (type: `audio_msgs/Audio`)

#### How to Run:
```bash
ros2 run audio_streamer audio_stream_publisher
```

### 2. Audio Stream Player

**Node Name**: `audio_stream_player`

This node subscribes to the `audio_stream` topic and plays the audio on the default system output device (e.g., speakers).

#### Parameters:
- `output_device`: System output device for playback (default: `pulse`).

#### Subscribed Topic:
- `audio_stream` (type: `audio_msgs/Audio`)

#### How to Run:
```bash
ros2 run audio_streamer audio_stream_player
```

## Usage

1. **Run the Audio Stream Publisher**:
   Captures audio from the specified input device and publishes it to the `audio_stream` topic.
   ```bash
   ros2 run audio_streamer audio_stream_publisher
   ```

2. **Run the Audio Stream Player**:
   Subscribes to the `audio_stream` topic and plays the audio on the default system output.
   ```bash
   ros2 run audio_streamer audio_stream_player
   ```

## Example

1. Start the publisher:
   ```bash
   ros2 run audio_streamer audio_stream_publisher
   ```

2. Start the player (in another terminal):
   ```bash
   ros2 run audio_streamer audio_stream_player
   ```

Now, any audio captured by the publisher node will be streamed and played on your system’s default output device in real-time.

## Dependencies

- `rclpy`: ROS2 Python client library
- `sounddevice`: Python library for audio input/output
- `numpy`: For handling audio data efficiently
- `audio_msgs`: Custom ROS2 message types for audio streaming

## License
MIT