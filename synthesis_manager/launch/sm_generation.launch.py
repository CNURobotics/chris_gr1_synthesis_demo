from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sm_generation',
            executable='sm_generation_server',
            name='sm_generation',
            output='screen',
        ),
    ])
