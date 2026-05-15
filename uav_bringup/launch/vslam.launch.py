# vslam.launch.py — Brings up cuVSLAM + nvblox using config from vslam.yaml.
#
# Deployment modes (same launch file, only the invocation differs):
#   • Orin (production, JetPack): ros2 launch uav_bringup vslam.launch.py
#   • Sim/HPC (Singularity):       singularity exec --nv isaac_vslam.sif bash -c \
#                                  "source /opt/ros/humble/setup.bash && \
#                                   ros2 launch uav_bringup vslam.launch.py"
#   • Dev (local source build):    ros2 launch uav_bringup vslam.launch.py
#
# This file is container-agnostic — it never references isaac_vslam.sif. The
# container concern stays in the outer invocation script. DDS connects cuVSLAM
# + nvblox to the rest of the stack regardless of where each piece is launched.
#
# When vslam.enable=false in vslam.yaml, this launch returns nothing.

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _load_config(config_path):
    if not os.path.exists(config_path):
        return None
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


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

    # ── cuVSLAM node ────────────────────────────────────────────────────
    cuvslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam_node',
        output='screen',
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

    # ── nvblox node ─────────────────────────────────────────────────────
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
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

    actions = [
        LogInfo(msg=f'[vslam.launch] Launching cuVSLAM (config: {config_path})'),
        cuvslam_node,
    ]
    if nvblox_cfg.get('enable', True):
        actions.append(LogInfo(msg='[vslam.launch] nvblox.enable=true — launching nvblox_node.'))
        actions.append(nvblox_node)
    else:
        actions.append(LogInfo(msg='[vslam.launch] nvblox.enable=false — cuVSLAM standalone mode.'))
    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('uav_bringup')
    default_cfg = os.path.join(bringup_dir, 'config', 'vslam.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vslam_config', default_value=default_cfg,
            description='Path to vslam.yaml config file.'),
        OpaqueFunction(function=_launch_setup),
    ])
