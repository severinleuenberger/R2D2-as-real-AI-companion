from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'r2d2_head_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Severin Leuenberger',
    maintainer_email='severin@example.com',
    description='R2D2 head tracking control - tilt servo and dome motor control for face tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tilt_tracking_node = r2d2_head_control.tilt_tracking_node:main',
        ],
    },
)

