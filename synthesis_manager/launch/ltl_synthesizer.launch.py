from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ltl_synthesizer',
            executable='ltl_synthesis_server',
            name='ltl_synthesizer',
            output='screen',
        ),
    ])
