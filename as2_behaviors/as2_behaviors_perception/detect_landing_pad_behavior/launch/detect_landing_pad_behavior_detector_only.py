"""Launch file for landing pad detector node only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the landing pad detector node."""
    config = os.path.join(get_package_share_directory('as2_behaviors_perception'),
                          'detect_landing_pad_behavior/config/params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'camera_image_topic', default_value='sensor_measurements/camera/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='sensor_measurements/camera/camera_info'),
        Node(
            package='as2_behaviors_perception',
            executable='detect_landing_pad_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[{'camera_image_topic': LaunchConfiguration('camera_image_topic'),
                         'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                         'use_sim_time': LaunchConfiguration('use_sim_time')},
                        config],
            emulate_tty=True,
        ),
    ])
