from setuptools import find_packages, setup

package_name = 'audio_streamer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='atarbabgei@gmail.com',
    description='Audio streaming package for ROS2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_stream_publisher = audio_streamer.audio_stream_publisher:main',
            'audio_stream_player = audio_streamer.audio_stream_player:main',
            'audio_stream_recorder = audio_streamer.audio_stream_recorder:main', 
        ],
    },
)
