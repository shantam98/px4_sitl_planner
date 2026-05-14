"""
realsense.launch.py

Launches the Intel RealSense D415 driver as a composable node and remaps its
default topics to the planner-stack convention (/drone/stereo/*, /drone/rgbd/*).

Composable node pattern matches the validated test launch (2026-05-15). On the
target Orin Nano with USB 3.0, all four streams (infra1, infra2, color, depth)
run simultaneously at 1280x720x30. The defaults below assume USB 3; override
the `profile` args for USB 2 development.

D415 has NO IMU. /drone/imu comes from Pixhawk via DDS (/fmu/out/vehicle_imu).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='1280x720x30',
        description='Stream profile (WxHxFPS) for depth/infra/color. '
                    'Use 640x480x15 on USB 2 development.')

    emitter_arg = DeclareLaunchArgument(
        'emitter_enabled',
        default_value='1',
        description='IR projector: 1=on (helps depth in low-texture scenes), '
                    '0=off (passive stereo, cleaner for cuVSLAM feature tracking).')

    profile = LaunchConfiguration('profile')
    emitter = LaunchConfiguration('emitter_enabled')

    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense2_camera_node',
        namespace='d415',
        parameters=[{
            'camera_name': 'd415',

            # ── Streams ────────────────────────────────────────────────
            'enable_color':         True,
            'enable_depth':         True,
            'enable_infra1':        True,
            'enable_infra2':        True,
            'align_depth.enable':   True,

            # ── Profile (resolution + fps) ─────────────────────────────
            # In v4.57 the same key applies to all depth-module streams
            'rgb_camera.profile':   profile,
            'depth_module.profile': profile,

            # ── Hardware sync + IR projector ───────────────────────────
            'enable_sync':                   True,
            'depth_module.emitter_enabled':  emitter,

            # ── No IMU on D415 (explicit) ──────────────────────────────
            'enable_gyro':  False,
            'enable_accel': False,

            # ── Frame ids (will be prefixed with d415_) ────────────────
            # Default optical frames are sufficient; static TF from
            # base_link -> d415_link defined in tf_hardware.launch.py.
        }],
        # Topic remaps to the planner-stack convention
        remappings=[
            ('infra1/image_rect_raw',          '/drone/stereo/left/image'),
            ('infra2/image_rect_raw',          '/drone/stereo/right/image'),
            ('infra1/camera_info',             '/drone/stereo/left/camera_info'),
            ('infra2/camera_info',             '/drone/stereo/right/camera_info'),
            ('color/image_raw',                '/drone/rgbd/image'),
            ('color/camera_info',              '/drone/rgbd/camera_info'),
            ('aligned_depth_to_color/image_raw',   '/drone/rgbd/depth'),
            ('aligned_depth_to_color/camera_info', '/drone/rgbd/camera_info_depth'),
            ('depth/color/points',             '/drone/rgbd/points'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[realsense_node],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        profile_arg,
        emitter_arg,
        container,
    ])
