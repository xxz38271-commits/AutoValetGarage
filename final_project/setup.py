from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='archering7727@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'enter_process = final_project.enter_process:main',
            'robot_enter_node = final_project.robot_enter_node:main',
            'parking_manage_realtime = final_project.parking_manage_realtime:main',
            'convert_exit_task = final_project.convert_exit_task:main',
            'robot1_status = final_project.robot1_status:main',
            'robot2_status = final_project.robot2_status:main',
        ],
    },
)
