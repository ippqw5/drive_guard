from setuptools import setup
import os
from glob import glob

package_name = 'driveguard_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        f'{package_name}.models',
        f'{package_name}.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'torchvision',
        'numpy',
        'opencv-python',
        'matplotlib',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='qc',
    maintainer_email='qc@example.com',
    description='DriveGuard AI package for autonomous driving based on PyTorch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_data = driveguard_ai.collector:record_main',
            'process_data = driveguard_ai.collector:process_main',
            'train_model = driveguard_ai.trainer:main',
            'inference = driveguard_ai.inferencer:main',
        ],
    },
)          