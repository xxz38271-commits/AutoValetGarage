# launch/robot_monitoring.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Robot 1 상태 모니터링
        Node(
            package='final_project',
            executable='robot1_status',
            name='robot1_status_publisher',
            output='screen',
            parameters=[
                {'robot_name': 'robot1'}
            ],
            # 환경 변수 설정 (Supabase 키 등)
            # environment={
            #     'SUPABASE_URL': os.environ.get('SUPABASE_URL', ''),
            #     'SUPABASE_KEY': os.environ.get('SUPABASE_KEY', '')
            # },
            respawn=True,  # 크래시 시 자동 재시작
            respawn_delay=2.0
        ),
        
        # Robot 2 상태 모니터링
        Node(
            package='final_project',
            executable='robot2_status',
            name='robot2_status_publisher',
            output='screen',
            parameters=[
                {'robot_name': 'robot2'}
            ],
            respawn=True,
            respawn_delay=2.0
        ),
    ])