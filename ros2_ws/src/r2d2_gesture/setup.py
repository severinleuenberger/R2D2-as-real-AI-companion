from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'r2d2_gesture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='severin',
    maintainer_email='severin.leuenberger@gmail.com',
    description='Gesture recognition intent node for R2D2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_intent_node = r2d2_gesture.gesture_intent_node:main',
        ],
    },
)
