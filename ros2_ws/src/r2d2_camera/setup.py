from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'r2d2_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Severin Leuenberger',
    maintainer_email='severin.leuenberger@gmail.com',
    description='OAK-D Lite camera ROS 2 driver for R2D2 platform',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node=r2d2_camera.camera_node:main',
            'camera_stream_node=r2d2_camera.camera_stream_node:main',
        ],
    },
)
