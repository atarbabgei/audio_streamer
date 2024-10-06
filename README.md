
# Audio Streamer ROS2 Package

## Overview
`audio_streamer` is a ROS2 package that enables real-time audio streaming. It includes three main components:
1. **Audio Stream Publisher**: Captures audio from a specified hardware device and publishes it as a ROS2 topic.
2. **Audio Stream Player**: Subscribes to the audio stream topic and plays the received audio data on the default speaker.
3. **Audio Stream Recorder**: Subscribes to the audio stream topic and records the received audio data to a `.wav` file.

This package uses the ['sounddevice'](https://github.com/spatialaudio/python-sounddevice) Python library to manage audio input and output streams, with support for hardware-specific audio devices and configurations such as sample rates and channels. It also includes a custom message package, `audio_msgs`, for defining the audio data.

(Tested on ROS2 Humble)

## Features
- **Audio Capture**: Captures audio from a microphone or other input device.
- **Real-Time Audio Streaming**: Publishes the captured audio as a ROS2 topic.
- **Audio Playback**: Subscribes to the audio stream and plays it back in real-time using the default system audio output (e.g., speakers).
- **Audio Recording**: Records audio from the ROS2 audio stream into a `.wav` file.
- **Custom Audio Message**: The package includes `audio_msgs`, a custom message for handling audio data in ROS2.

## Installation

1. **Clone the Repository**
   ```bash
   mkdir -p ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/atarbabgei/audio_streamer.git
   ```

2. **Install Dependencies**
   Install the required Python dependencies using `pip`:
   ```bash
   pip3 install numpy sounddevice scipy
   ```

3. **Build the Package**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
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

3. Start the recorder (in another terminal):
   ```bash
   ros2 run audio_streamer audio_stream_recorder
   ```

This will record the audio stream to a `.wav` file.

## License
MIT