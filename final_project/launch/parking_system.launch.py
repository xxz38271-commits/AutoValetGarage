# launch/parking_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Exit Task 처리
        Node(
            package='final_project',
            executable='convert_exit_task',
            name='convert_exit_task',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
        
        # Realtime Parking Management
        Node(
            package='final_project',
            executable='parking_manage_realtime',
            name='parking_manage_realtime',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
    ])