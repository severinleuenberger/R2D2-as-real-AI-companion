from setuptools import setup
import os
from glob import glob

package_name = 'r2d2_audio'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Severin',
    maintainer_email='severin@r2d2.local',
    description='R2D2 audio output module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_beep_node=r2d2_audio.audio_beep_node:main',
            'audio_notification_node=r2d2_audio.audio_notification_node:main',
        ],
    },
)
