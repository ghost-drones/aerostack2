"""Launch file for multi static transform broadcaster only."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Set up multiple static_transform_publisher nodes from YAML."""
    tf_config_file = LaunchConfiguration('tf_config_file').perform(context)

    with open(tf_config_file, 'r') as f:
        config = yaml.safe_load(f)

    nodes = []
    for tf in config.get('static_transforms', []):
        # Read translation and rotation fields from YAML
        xyz = [str(x) for x in tf['translation']]
        rpy = [str(x) for x in tf['rotation']]

        # Combine all args for static_transform_publisher
        args_list = xyz + rpy + [tf['frame_id'], tf['child_frame_id']]

        # Create the static_transform_publisher node
        node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"static_tf_{tf['frame_id']}_{tf['child_frame_id']}",
            arguments=args_list,
            output='screen',
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    """Launch multiple static transform publishers from YAML."""
    default_tf_config = os.path.join(
        get_package_share_directory('as2_behaviors_perception'),
        'detect_aruco_markers_behavior/config/ariel_static_transforms.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_config_file',
            default_value=default_tf_config,
            description='YAML file containing static transforms'
        ),
        OpaqueFunction(function=launch_setup),
    ])
