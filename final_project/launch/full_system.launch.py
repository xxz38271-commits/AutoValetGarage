# launch/full_system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('final_project')
    
    return LaunchDescription([
        # 1. 로봇 모니터링 시스템
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'robot_monitoring.launch.py')
            )
        ),
        
        # 2. 주차 시스템
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'parking_system.launch.py')
            )
        ),
        
        # 3. 추가 노드들
        Node(
            package='final_project',
            executable='robot_enter_node',
            name='robot_enter_node',
            output='screen',
            respawn=True
        ),
        
        Node(
            package='final_project',
            executable='enter_process',
            name='enter_process',
            output='screen',
            respawn=True
        ),
    ])