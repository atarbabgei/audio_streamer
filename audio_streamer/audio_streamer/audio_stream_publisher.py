import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import sounddevice as sd
import numpy as np
import collections
import time  # Import time to get system time in microseconds

class AudioStreamPublisher(Node):
    def __init__(self):
        super().__init__('audio_stream_publisher')

        # Declare parameters with default values
        self.declare_parameter('input_hw_id', 'hw:1,0')
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('blocksize', 2048)
        self.declare_parameter('buffer_size', 10)

        # Get parameters
        self.input_hw_id = self.get_parameter('input_hw_id').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.blocksize = self.get_parameter('blocksize').get_parameter_value().integer_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value

        # Buffer to hold audio chunks
        self.buffer = collections.deque(maxlen=self.buffer_size)  # Audio buffer

        # List and find the input device by hardware ID
        devices = sd.query_devices()
        self.selected_input_device = None
        for device in devices:
            if self.input_hw_id in device['name']:
                self.selected_input_device = device
                break

        if self.selected_input_device is None:
            raise ValueError(f"No device found with the hardware ID {self.input_hw_id}")
        
        self.get_logger().info("Audio Stream Publisher Node has started")
        self.get_logger().info(f"Selected input device: {self.selected_input_device['name']}")
        self.get_logger().info(f"Max input channels: {self.selected_input_device['max_input_channels']}")
        self.get_logger().info(f"Default sample rate: {self.selected_input_device['default_samplerate']}")
        self.get_logger().info(f"Configured sample rate: {self.sample_rate}")
        self.get_logger().info(f"Configured channels: {self.channels}")
        self.get_logger().info(f"Configured blocksize: {self.blocksize}")
        self.get_logger().info(f"Configured buffer size: {self.buffer_size}")

        # Get the number of input channels for the selected device
        self.channels = self.selected_input_device['max_input_channels']

        # Start the input stream with the appropriate configuration
        self.stream = sd.InputStream(
            device=self.selected_input_device['name'],
            channels=self.channels,
            samplerate=self.sample_rate,
            blocksize=self.blocksize,  # Adjustable block size via parameter
            dtype='float32',  # Ensure we capture audio in float32 format
            callback=self.input_callback,
            latency='low'
        )

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(Audio, 'audio_stream', 10)
        self.stream.start()

    def input_callback(self, indata, frames, portaudio_time, status):
        if status:
            self.get_logger().warn(f'{status}')
        
        # Get the current time in microseconds since the Unix epoch
        current_time_us = int(time.time() * 1e6)  # Convert to microseconds as integer

        # Append the input data (float32) to the buffer
        self.buffer.append(indata.copy())

        # Prepare the message with the data from the buffer
        if self.buffer:
            audio_msg = Audio()
            audio_msg.sample_rate = self.sample_rate
            audio_msg.channels = self.channels
            audio_msg.timestamp = current_time_us  # Add the timestamp in microseconds
            # Convert buffer to a flat list and send it as float32
            audio_msg.data = self.buffer[-1].flatten().tolist()  # Use the latest audio data in the buffer
            
            # Publish the audio message
            self.publisher_.publish(audio_msg)

    def destroy_node(self):
        # Stop the stream and close it when the node is destroyed
        self.stream.stop()
        self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
