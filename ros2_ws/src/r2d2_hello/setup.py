from setuptools import find_packages, setup

package_name = 'r2d2_hello'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='severin',
    maintainer_email='severin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'beep_node = r2d2_hello.beep_node:main',
            'heartbeat_node = r2d2_hello.heartbeat_node:main',
     ],
},
)
