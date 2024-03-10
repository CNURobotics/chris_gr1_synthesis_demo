from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='synthesis_manager',
            executable='behavior_synthesis_server',
            name='behavior_synthesis',
            output='screen',
        ),
    ])
