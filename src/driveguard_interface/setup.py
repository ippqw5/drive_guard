from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'driveguard_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='DriveGuard Interface Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

        ],
    },
)