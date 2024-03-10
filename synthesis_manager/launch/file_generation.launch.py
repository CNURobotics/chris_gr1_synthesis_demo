from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    configPath = os.path.join(get_package_share_directory('file_generation'),
                              'config', 'wgcf.yaml')

    return LaunchDescription([
        Node(
            package='file_generation',
            executable='file_generation_server',
            name='file_generation',
            output='screen',
            parameters=[{'config_file': configPath}],
        )
    ])
