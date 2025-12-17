from setuptools import setup
import os
from glob import glob

package_name = 'r2d2_speech'

setup(
    name=package_name,
    version='0.1.0',
    packages=['r2d2_speech_ros'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Severin',
    maintainer_email='severin@r2d2.local',
    description='R2D2 speech system - OpenAI Realtime API integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_node = r2d2_speech_ros.speech_node:main',
        ],
    },
)

