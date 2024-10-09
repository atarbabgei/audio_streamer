from setuptools import find_packages, setup

package_name = 'audio_analysis'

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
    maintainer='atar',
    maintainer_email='atarbabgei@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amplitude_stream_node = audio_analysis.amplitude_stream_node:main',
            'spectrogram_stream_node = audio_analysis.spectrogram_stream_node:main',
        ],
    },
)
