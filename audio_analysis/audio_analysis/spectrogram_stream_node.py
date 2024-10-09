import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft

class SpectrogramStreamNode(Node):
    def __init__(self):
        super().__init__('spectrogram_stream_node')

        # Create subscriber for 'audio_stream' topic
        self.subscription = self.create_subscription(
            Audio,
            'audio_stream',
            self.listener_callback,
            10
        )

        # Variables for audio data and plotting
        self.sample_rate = 48000
        self.channels = 1
        self.fft_size = 2048  # FFT window size
        self.time_window = 100  # Number of FFT frames to display (i.e., horizontal axis for time)

        # Initialize spectrogram data
        self.fft_data = np.zeros((self.fft_size // 2, self.time_window))

        # Frequency axis in Hz
        self.freqs = np.fft.fftfreq(self.fft_size, 1.0 / self.sample_rate)[:self.fft_size // 2]

        # Set up plotting
        self.fig, self.ax = plt.subplots()
        self.spectrogram_plot = self.ax.imshow(
            np.zeros((self.fft_size // 2, self.time_window)), aspect='auto', cmap='viridis', vmin=-100, vmax=0,
            extent=[0, self.time_window, 0, self.sample_rate // 2]
        )
        self.ax.set_xlabel("Time (frames)")
        self.ax.set_ylabel("Frequency (Hz)")

    def listener_callback(self, msg):
        audio_data = np.array(msg.data, dtype=np.float32)

        # Apply FFT to the audio data
        fft_result = np.abs(fft(audio_data, self.fft_size))

        # Add a small epsilon to avoid log10(0)
        epsilon = np.finfo(float).eps
        fft_result[fft_result == 0] = epsilon

        # Keep only the positive frequency components (first half)
        fft_result = fft_result[:self.fft_size // 2]

        # Update the spectrogram data
        self.fft_data = np.roll(self.fft_data, -1, axis=1)
        self.fft_data[:, -1] = 20 * np.log10(fft_result)  # Log scale for magnitude

    def update_plot(self):
        # Update the plot data
        self.spectrogram_plot.set_data(self.fft_data)
        plt.draw()  # Redraw the figure
        plt.pause(0.001)  # Short pause to allow for real-time updates

def main(args=None):
    rclpy.init(args=args)
    node = SpectrogramStreamNode()

    try:
        # Main loop
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)  # Allow callbacks to be processed
            node.update_plot()  # Update the plot regularly

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
