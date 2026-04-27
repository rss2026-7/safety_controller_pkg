"""Launch the safety controller with its YAML config.

The config file is the ground truth — every parameter the node needs must
be present in it. The default points at the package's installed
`config/safety.yaml`; pass `config:=<absolute path>` to override.

Usage:
    ros2 launch safety_controller_pkg safety.launch.py
    ros2 launch safety_controller_pkg safety.launch.py config:=/abs/path/to/foo.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('safety_controller_pkg')
    default_config = os.path.join(pkg_share, 'config', 'safety.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Absolute path to the safety controller YAML config.',
        ),
        Node(
            package='safety_controller_pkg',
            executable='safety_controller_pkg',
            name='safety_node',
            parameters=[LaunchConfiguration('config')],
            output='screen',
            emulate_tty=True,
        ),
    ])
