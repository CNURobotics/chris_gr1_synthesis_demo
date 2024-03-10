from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ltl_specification',
            executable='ltl_compilation_server',
            name='ltl_specification',
            output='screen',
        ),
    ])
