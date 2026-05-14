"""
hardware_stack.launch.py — top-level real-hardware bringup.

Orchestrates (in order):
  T0  hardware TF (static, immediate)
  T0  Pixhawk DDS agent (serial)              -- waits ~3s for /fmu/* topics
  T+3 RealSense D415                          -- waits ~3s for streams
  T+6 MaixSense ToF array (5 sensors)         -- waits ~3s for clouds
  T+9 cloud_merge_node (uav_depth_fusion)     -- needs ToFs + TFs
  T+11 control + planner (existing planner_ws launches)

Run via SSH on Orin Nano:
    ros2 launch uav_hardware_bringup hardware_stack.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hw   = get_package_share_directory('uav_hardware_bringup')

    # ── Launch args ────────────────────────────────────────────────
    pixhawk_dev_arg = DeclareLaunchArgument(
        'pixhawk_dev', default_value='/dev/ttyACM0')
    pixhawk_baud_arg = DeclareLaunchArgument(
        'pixhawk_baud', default_value='921600')
    rs_profile_arg = DeclareLaunchArgument(
        'rs_profile', default_value='1280x720x30',
        description='RealSense stream profile (USB 3). Use 640x480x15 on USB 2.')

    # ── Sub-launches ───────────────────────────────────────────────
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hw, 'launch', 'tf_hardware.launch.py')),
    )

    pixhawk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hw, 'launch', 'pixhawk_serial.launch.py')),
        launch_arguments={
            'dev':  LaunchConfiguration('pixhawk_dev'),
            'baud': LaunchConfiguration('pixhawk_baud'),
        }.items(),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hw, 'launch', 'realsense.launch.py')),
        launch_arguments={'profile': LaunchConfiguration('rs_profile')}.items(),
    )

    tofs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hw, 'launch', 'maixsense_tof_array.launch.py')),
    )

    # cloud_merge_node from uav_depth_fusion (px4_odom_bridge is handled
    # inside uav_depth_fusion's own fusion.launch.py if used; here we run
    # cloud_merge directly since TFs come from our hardware tf launch)
    cloud_merge = Node(
        package='uav_depth_fusion',
        executable='cloud_merge_node',
        name='cloud_merge_node',
        output='screen',
        parameters=[{
            'target_frame':   'base_link',
            'tf_timeout_sec': 0.1,
        }],
    )

    px4_odom_bridge = Node(
        package='uav_depth_fusion',
        executable='px4_odom_bridge',
        name='px4_odom_bridge',
        output='screen',
    )

    return LaunchDescription([
        pixhawk_dev_arg,
        pixhawk_baud_arg,
        rs_profile_arg,

        # T+0: TFs + DDS agent
        tf_launch,
        pixhawk_launch,

        # T+3: RealSense (after DDS has settled)
        TimerAction(period=3.0, actions=[realsense_launch]),

        # T+6: ToF array (after RealSense USB is established)
        TimerAction(period=6.0, actions=[tofs_launch]),

        # T+9: cloud_merge + odom bridge (after all sensors publishing)
        TimerAction(period=9.0, actions=[cloud_merge, px4_odom_bridge]),
    ])
