import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import numpy as np

class AmplitudeStreamNode(Node):
    def __init__(self):
        super().__init__('amplitude_stream_node')

        self.subscription = self.create_subscription(
            Audio,
            'audio_stream',
            self.audio_callback,
            10
        )

        self.amplitude_publisher = self.create_publisher(Audio, 'audio_amplitude_stream', 10)

    def audio_callback(self, msg):
        audio_data = np.array(msg.data, dtype=np.float32).reshape(-1, msg.channels)
        amplitude = np.sqrt(np.mean(audio_data**2, axis=0))

        amplitude_msg = Audio()
        amplitude_msg.sample_rate = msg.sample_rate
        amplitude_msg.channels = msg.channels
        amplitude_msg.timestamp = msg.timestamp
        amplitude_msg.data = amplitude.flatten().tolist()

        self.amplitude_publisher.publish(amplitude_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AmplitudeStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
