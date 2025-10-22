"""Unified launch file for landing pad detector and static transform publisher."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    detector_config = os.path.join(
        get_package_share_directory('as2_behaviors_perception'),
        'detect_landing_pad_behavior/config/params.yaml'
    )

    default_tf_config = os.path.join(
        get_package_share_directory('as2_behaviors_perception'),
        'detect_landing_pad_behavior/config/ariel_static_transforms.yaml'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID'),
        description='Namespace do drone'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo simulado'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nível de log para os nós'
    )
    camera_image_topic_arg = DeclareLaunchArgument(
        'camera_image_topic',
        default_value='sensor_measurements/camera/image_raw',
        description='Tópico de imagem da câmera'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='sensor_measurements/camera/camera_info',
        description='Tópico de informações da câmera'
    )
    tf_config_arg = DeclareLaunchArgument(
        'tf_config_file',
        default_value=default_tf_config,
        description='Arquivo YAML com transforms estáticos'
    )

    detector_node = Node(
        package='as2_behaviors_perception',
        executable='detect_landing_pad_behavior_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        parameters=[
            {
                'camera_image_topic': LaunchConfiguration('camera_image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            },
            detector_config
        ],
        emulate_tty=True,
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        parameters=[LaunchConfiguration('tf_config_file')],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        log_level_arg,
        camera_image_topic_arg,
        camera_info_topic_arg,
        tf_config_arg,
        detector_node,
        static_tf_node
    ])
