"""
tf_hardware.launch.py

Static TFs for the real airframe. Reuses uav_depth_fusion's tf_static_broadcaster
node — same code that handles sim TFs — but loaded with hardware mount offsets
from hardware_tf.yaml.

If the real-drone sensor mounts differ from the SDF, only hardware_tf.yaml needs
updating; the node code is shared.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_hardware_bringup')
    default_yaml = os.path.join(pkg_share, 'config', 'hardware_tf.yaml')

    yaml_arg = DeclareLaunchArgument(
        'tf_yaml',
        default_value=default_yaml,
        description='Path to hardware static-TF parameter YAML.')

    return LaunchDescription([
        yaml_arg,
        Node(
            package='uav_depth_fusion',
            executable='tf_static_broadcaster',
            name='hardware_tf_static_broadcaster',
            output='screen',
            parameters=[LaunchConfiguration('tf_yaml')],
        ),
    ])
