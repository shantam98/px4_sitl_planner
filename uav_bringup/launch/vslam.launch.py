# vslam.launch.py — Brings up cuVSLAM (+ optional nvblox) inside a composable
# node container.
#
# Why a composable container: Isaac ROS Visual SLAM and nvblox ship as
# ComposableNode plugins (nvidia::isaac_ros::visual_slam::VisualSlamNode,
# nvblox::NvbloxNode). Loading them via rclcpp_components is the supported
# path; invoking the bare executable bypasses GXF library resolution.
#
# We also prepend the GXF extension dirs to LD_LIBRARY_PATH so the runtime
# linker can find libgxf_*.so. Inside isaac_vslam.sif these live at
# /opt/ros/humble/share/isaac_ros_gxf/gxf/lib/<extension>/.
#
# Deployment modes (same launch file, only invocation differs):
#   • Orin (production, JetPack): ros2 launch uav_bringup vslam.launch.py
#   • Sim/HPC (Singularity):       singularity exec --nv isaac_vslam.sif bash -c \
#                                  "source /opt/ros/humble/setup.bash && \
#                                   ros2 launch uav_bringup vslam.launch.py"
#
# When vslam.enable=false in vslam.yaml, this launch returns nothing.

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


# GXF extension dirs inside isaac_vslam.sif. Discovered via:
#   singularity exec isaac_vslam.sif ls /opt/ros/humble/share/isaac_ros_gxf/gxf/lib/
GXF_LIB_BASE = '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib'
GXF_SUBDIRS = [
    'behavior_tree', 'core', 'cuda', 'ipc', 'logger', 'multimedia',
    'network', 'npp', 'python_codelet', 'sample', 'serialization',
    'std', 'test',
]


def _load_config(config_path):
    if not os.path.exists(config_path):
        return None
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def _gxf_ld_library_path():
    extra = ':'.join(os.path.join(GXF_LIB_BASE, d) for d in GXF_SUBDIRS)
    existing = os.environ.get('LD_LIBRARY_PATH', '')
    return f'{extra}:{existing}' if existing else extra


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('vslam_config').perform(context)

    cfg = _load_config(config_path) or {}
    vslam_cfg = cfg.get('vslam', {})

    if not vslam_cfg.get('enable', False):
        return [LogInfo(msg=f'[vslam.launch] vslam.enable=false in {config_path} — no nodes launched.')]

    cuvslam_cfg = vslam_cfg.get('cuvslam', {})
    nvblox_cfg  = vslam_cfg.get('nvblox', {})
    topics_v    = cuvslam_cfg.get('topics', {})
    topics_n    = nvblox_cfg.get('topics', {})
    nvblox_on   = nvblox_cfg.get('enable', True)

    # ── cuVSLAM composable node ─────────────────────────────────────────
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': True,
            'num_cameras': cuvslam_cfg.get('num_cameras', 2),
            'enable_image_denoising': False,
            'rectified_images': cuvslam_cfg.get('rectified_images', True),
            'enable_imu_fusion': cuvslam_cfg.get('enable_imu_fusion', False),
            'map_frame':  cuvslam_cfg.get('map_frame',  'map'),
            'odom_frame': cuvslam_cfg.get('odom_frame', 'odom'),
            'base_frame': cuvslam_cfg.get('base_frame', 'base_link'),
        }],
        remappings=[
            ('visual_slam/image_0',       topics_v.get('image_0',       '/drone/stereo/left/image')),
            ('visual_slam/image_1',       topics_v.get('image_1',       '/drone/stereo/right/image')),
            ('visual_slam/camera_info_0', topics_v.get('camera_info_0', '/drone/stereo/left/camera_info')),
            ('visual_slam/camera_info_1', topics_v.get('camera_info_1', '/drone/stereo/right/camera_info')),
            ('visual_slam/imu',           topics_v.get('imu',           '/drone/imu')),
        ],
    )

    composable_nodes = [visual_slam_node]

    # ── nvblox composable node (optional) ───────────────────────────────
    if nvblox_on:
        nvblox_composable = ComposableNode(
            name='nvblox_node',
            package='nvblox_ros',
            plugin='nvblox::NvbloxNode',
            parameters=[{
                'use_sim_time': True,
                'voxel_size':                  nvblox_cfg.get('voxel_size', 0.05),
                'mapping_type':                nvblox_cfg.get('mapping_type', 'static_tsdf'),
                'esdf_slice_min_height':       nvblox_cfg.get('esdf_slice_min_height', 0.3),
                'esdf_slice_max_height':       nvblox_cfg.get('esdf_slice_max_height', 1.8),
                'max_integration_distance_m':  nvblox_cfg.get('max_integration_distance_m', 5.0),
                'global_frame':                cuvslam_cfg.get('map_frame', 'map'),
            }],
            remappings=[
                ('depth/image',         topics_n.get('depth_image', '/drone/rgbd/depth')),
                ('depth/camera_info',   topics_n.get('camera_info', '/drone/rgbd/camera_info')),
                ('color/image',         topics_n.get('color_image', '/drone/rgbd/image')),
                ('color/camera_info',   topics_n.get('camera_info', '/drone/rgbd/camera_info')),
            ],
        )
        composable_nodes.append(nvblox_composable)

    # ── Container that hosts the composable nodes ───────────────────────
    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
        emulate_tty=True,
    )

    mode_msg = 'cuVSLAM + nvblox' if nvblox_on else 'cuVSLAM standalone (nvblox.enable=false)'
    return [
        LogInfo(msg=f'[vslam.launch] Launching {mode_msg} (config: {config_path})'),
        container,
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('uav_bringup')
    default_cfg = os.path.join(bringup_dir, 'config', 'vslam.yaml')

    return LaunchDescription([
        # Make GXF extensions discoverable for the entire launch process tree.
        # This must be set BEFORE the composable container is spawned.
        SetEnvironmentVariable('LD_LIBRARY_PATH', _gxf_ld_library_path()),

        DeclareLaunchArgument(
            'vslam_config', default_value=default_cfg,
            description='Path to vslam.yaml config file.'),

        OpaqueFunction(function=_launch_setup),
    ])
