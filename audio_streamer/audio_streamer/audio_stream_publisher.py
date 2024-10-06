import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import sounddevice as sd
import numpy as np
import collections

class AudioStreamPublisher(Node):
    def __init__(self):
        super().__init__('audio_stream_publisher')
        
        # Hardware ID for input (e.g., hw:1,0)
        self.input_hw_id = "hw:1,0"
        self.sample_rate = 48000  # Set your desired sample rate here
        self.channels = 1  # Set number of audio channels here
        self.buffer_size = 10  # Size of the buffer to hold audio chunks
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

        # Get the number of input channels for the selected device
        self.channels = self.selected_input_device['max_input_channels']

        # Start the input stream with the appropriate configuration
        self.stream = sd.InputStream(
            device=self.selected_input_device['name'],
            channels=self.channels,
            samplerate=self.sample_rate,
            blocksize=1024,  # Adjust blocksize if needed
            dtype='float32',  # Ensure we capture audio in float32 format
            callback=self.input_callback,
            latency='low'
        )

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(Audio, 'audio_stream', 10)
        self.stream.start()

    def input_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().error(f'Error: {status}')
        # Append the input data (float32) to the buffer
        self.buffer.append(indata.copy())

        # Prepare the message with the data from the buffer
        if self.buffer:
            audio_msg = Audio()
            audio_msg.sample_rate = self.sample_rate
            audio_msg.channels = self.channels
            # Convert buffer to a flat list and send it as float32
            audio_msg.data = self.buffer[-1].flatten().tolist()  # Use the latest audio data in the buffer
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
