from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    nodes.append(
        Node(
            package='parking_system',
            executable='parking_slot_manager',
            name='parking_slot_manager',
            output='screen',
            parameters=[{
                'num_slots': 6,
                'occupied_slots': [1, 2],
            }]
        )
    )

    nodes.append(
        Node(
            package='parking_system',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
        )
    )

    nodes.append(
        Node(
            package='parking_system',
            executable='task_allocator',
            name='task_allocator',
            output='screen',
        )
    )

    nodes.append(
        Node(
            package='parking_executor',
            executable='mission_executor',
            name='mission_executor',
            namespace='robot1',
            output='screen',
        )
    )

    nodes.append(
        Node(
            package='parking_executor',
            executable='mission_executor',
            name='mission_executor',
            namespace='robot5',
            output='screen',
        )
    )

    return LaunchDescription(nodes)
