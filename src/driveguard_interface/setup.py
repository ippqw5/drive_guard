from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'driveguard_interface'

setup(
    name=package_name,
    version='1.0.0',
    # packages=['driveguard_interface'],
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'nav_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
    ],
    zip_safe=True,
    maintainer='Qian.Cheng',
    maintainer_email='ippqw5@163.com',
    description='DriveGuard Interface Package for ROS2 communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

        ],
    },
)