from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'r2d2_perception'

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
    author='Severin Leuenberger',
    author_email='severin@example.com',
    maintainer='Severin Leuenberger',
    maintainer_email='severin@example.com',
    url='https://github.com/severinleuenberger/R2D2-as-real-AI-companion',
    download_url='https://github.com/severinleuenberger/R2D2-as-real-AI-companion',
    keywords=['ROS', 'ROS2', 'R2D2', 'perception', 'camera', 'OAK-D'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development',
    ],
    description='R2D2 perception node for OAK-D camera integration',
    long_description='Basic perception node that subscribes to OAK-D RGB camera frames and logs diagnostics: frame count, FPS, image dimensions.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = r2d2_perception.perception_node:main',
            'image_listener = r2d2_perception.image_listener:main',
        ],
    },
)
