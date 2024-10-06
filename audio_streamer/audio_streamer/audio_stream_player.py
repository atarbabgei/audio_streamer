import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import sounddevice as sd
import numpy as np
import collections

class AudioStreamPlayer(Node):
    def __init__(self):
        super().__init__('audio_stream_player')

        # Declare parameters with default values
        self.declare_parameter('output_device', 'pulse')  # Default output device is PulseAudio
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('blocksize', 2048)
        self.declare_parameter('buffer_size', 10)

        # Get parameters
        self.output_device = self.get_parameter('output_device').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('blocksize').get_parameter_value().integer_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value

        # Buffer to hold audio chunks
        self.buffer = collections.deque(maxlen=self.buffer_size)  # Audio buffer

        # ROS2 Subscription
        self.subscription = self.create_subscription(
            Audio,
            'audio_stream',
            self.audio_callback,
            10
        )

        self.stream = None  # Placeholder for the audio output stream

        # Log that the node has started
        self.get_logger().info("Audio Stream Player Node has started.")
        self.get_logger().info(f"Configured output device: {self.output_device}")
        self.get_logger().info(f"Configured sample rate: {self.sample_rate}")
        self.get_logger().info(f"Configured channels: {self.channels}")
        self.get_logger().info(f"Configured blocksize: {self.blocksize}")
        self.get_logger().info(f"Configured buffer size: {self.buffer_size}")

    def audio_callback(self, msg):
        # Update sample rate and channel info based on the message
        self.sample_rate = msg.sample_rate
        self.channels = msg.channels

        # Convert incoming float32 audio data into a numpy array
        audio_data = np.array(msg.data, dtype=np.float32).reshape(-1, self.channels)

        # Add the audio data to the buffer
        self.buffer.append(audio_data)

        # Start the stream if it is not running
        if self.stream is None:
            self.get_logger().info("Starting audio output stream...")
            self.stream = sd.OutputStream(
                device=self.output_device,
                channels=self.channels,
                samplerate=self.sample_rate,
                dtype='float32',  # Ensure playback uses float32 format
                blocksize=self.blocksize,  # Block size is now parameterized
                latency='low',
                callback=self.output_callback
            )
            self.stream.start()

    def output_callback(self, outdata, frames, time, status):
        if status:
            self.get_logger().error(f"Stream error: {status}")

        # Check if there's data in the buffer
        if self.buffer:
            # Get audio data from buffer
            data = self.buffer.popleft()

            # Ensure the input data size matches the output buffer size
            if len(data) < len(outdata):
                # If the input is shorter, pad it with zeros
                outdata[:len(data)] = data
                outdata[len(data):] = 0
            else:
                # If the input is longer or the same size, trim it
                outdata[:] = data[:len(outdata)]
        else:
            # If the buffer is empty, fill output with silence
            outdata.fill(0)

    def destroy_node(self):
        # Stop and close the stream when the node is destroyed
        if self.stream:
            self.get_logger().info("Stopping audio output stream.")
            self.stream.stop()
            self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
