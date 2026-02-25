from setuptools import setup
import os
from glob import glob

package_name = 'parking_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('parking_system/config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Multi-robot parking management nodes (slot manager, mission manager, task allocator)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_slot_manager = parking_system.parking_slot_manager:main',
            'mission_manager = parking_system.mission_manager:main',
            'task_allocator = parking_system.task_allocator:main',
        ],
    },
)

