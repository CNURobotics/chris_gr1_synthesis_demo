from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ltl_specification',
            executable='ltl_compilation_server',
            name='ltl_specification',
            output='screen',
        ),
        Node(
            package='ltl_synthesizer',
            executable='ltl_synthesis_server',
            name='ltl_synthesizer',
            output='screen',
        ),
        Node(
            package='sm_generation',
            executable='sm_generation_server',
            name='sm_generation',
            output='screen',
        ),
        Node(
            package='synthesis_manager',
            executable='behavior_synthesis_server',
            name='behavior_synthesis',
            output='screen',
        ),
    ])
