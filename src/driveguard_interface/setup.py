from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'driveguard_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        f'{package_name}.driveguard_carla',
        f'{package_name}.driveguard_vision_control',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
    ],
    zip_safe=True,
    maintainer='Qian.Cheng',
    maintainer_email='ippqw5@163.com',
    description='DriveGuard Interface Package for ROS2 communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_control = driveguard_interface.driveguard_vision_control.main:main',
        ],
    },
)