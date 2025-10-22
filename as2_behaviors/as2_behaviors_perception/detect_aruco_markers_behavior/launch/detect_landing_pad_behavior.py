"""Unified launch file: runs detector and TF-only launches."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """Launch both detector and TF-only launches together."""

    # Expande a variável de ambiente corretamente
    aerostack2_path = os.path.expandvars('$AEROSTACK2_PATH')

    # Caminhos completos dos outros launch files
    detector_launch = os.path.join(
        aerostack2_path,
        'as2_behaviors/as2_behaviors_perception/detect_landing_pad_behavior/launch/detect_landing_pad_behavior_detector_only.py'
    )

    tf_launch = os.path.join(
        aerostack2_path,
        'as2_behaviors/as2_behaviors_perception/detect_landing_pad_behavior/launch/detect_landing_pad_behavior_tf_only.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=EnvironmentVariable(
                'AEROSTACK2_SIMULATION_DRONE_ID'),
            description='Drone namespace'
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='sensor_measurements/camera/image_raw'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='sensor_measurements/camera/camera_info'
        ),

        # Inclui o launch do detector
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detector_launch),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'log_level': LaunchConfiguration('log_level'),
                'camera_image_topic': LaunchConfiguration('camera_image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            }.items(),
        ),

        # Inclui o launch dos TFs estáticos
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_launch)
        ),
    ])
