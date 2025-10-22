"""Launch file for multi static transform broadcaster only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the multi static transform broadcaster node."""
    tf_config = os.path.join(get_package_share_directory('as2_behaviors_perception'),
                             'detect_landing_pad_behavior/config/ariel_static_transforms.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('tf_config_file', default_value=tf_config,
                              description='YAML file containing static transforms'),
        Node(
            package='as2_behaviors_perception',
            executable='multi_static_tf_broadcaster_node',
            name='multi_static_tf_broadcaster',
            output='screen',
            arguments=[LaunchConfiguration('tf_config_file')],
        ),
    ])
