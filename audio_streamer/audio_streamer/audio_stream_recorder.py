import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import numpy as np
import scipy.io.wavfile as wav
from datetime import datetime

class AudioStreamRecorder(Node):
    def __init__(self):
        super().__init__('audio_stream_recorder')

        # Parameters
        self.sample_rate = 48000  # Will be updated based on the incoming audio message
        self.channels = 1  # Will be updated based on the incoming audio message
        self.audio_buffer = []

        # Subscribe to the audio_stream topic
        self.subscription = self.create_subscription(
            Audio,
            'audio_stream',
            self.audio_callback,
            10
        )

        self.get_logger().info("Audio Stream Recorder Node has started.")

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
        file_name = f"audio_recording_{current_time}.wav"

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
