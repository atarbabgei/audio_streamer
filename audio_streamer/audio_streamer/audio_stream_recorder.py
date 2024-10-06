import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import numpy as np
import scipy.io.wavfile as wav
from datetime import datetime

class AudioStreamRecorder(Node):
    def __init__(self):
        super().__init__('audio_stream_recorder')

        # Declare parameters with default values
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('file_prefix', 'audio_recording')

        # Get parameters
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.file_prefix = self.get_parameter('file_prefix').get_parameter_value().string_value

        # Audio buffer to store incoming data
        self.audio_buffer = []

        # Subscribe to the audio_stream topic
        self.subscription = self.create_subscription(
            Audio,
            'audio_stream',
            self.audio_callback,
            10
        )

        self.get_logger().info("Audio Stream Recorder Node has started.")
        self.get_logger().info(f"Configured sample rate: {self.sample_rate}")
        self.get_logger().info(f"Configured channels: {self.channels}")
        self.get_logger().info(f"File prefix for recordings: {self.file_prefix}")

    def audio_callback(self, msg):
        # Update sample rate and channels if different
        if self.sample_rate != msg.sample_rate:
            self.sample_rate = msg.sample_rate
            self.get_logger().info(f"Sample rate set to {self.sample_rate} Hz")

        if self.channels != msg.channels:
            self.channels = msg.channels
            self.get_logger().info(f"Number of channels set to {self.channels}")

        # Convert incoming audio data to a numpy array
        audio_data = np.array(msg.data, dtype=np.float32).reshape(-1, self.channels)
        self.audio_buffer.append(audio_data)

    def save_to_wav(self):
        # Get current date and time for the filename
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_name = f"{self.file_prefix}_{current_time}.wav"

        # Concatenate the buffered audio data
        audio_data = np.concatenate(self.audio_buffer, axis=0)

        # Write to a .wav file
        self.get_logger().info(f"Saving audio to {file_name}")
        wav.write(file_name, self.sample_rate, audio_data)

def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_wav()  # Save the recorded audio when stopping
        node.get_logger().info("Recording stopped. Audio saved.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
