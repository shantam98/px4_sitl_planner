import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('uav_local_planner')
    mp_config = os.path.join(pkg, 'config', 'mp_params.yaml')

    use_mp           = LaunchConfiguration('use_mp',           default='true')
    planner_backend  = LaunchConfiguration('planner_backend',  default='mp')
    sensor_source    = LaunchConfiguration('sensor_source',    default='d415')

    # Condition helpers — only one MP-family node runs at a time.
    is_mp_baseline = PythonExpression([
        "'", use_mp, "' == 'true' and '", planner_backend, "' == 'mp'"])
    is_mp_esdf = PythonExpression([
        "'", use_mp, "' == 'true' and '", planner_backend, "' == 'mp_esdf'"])

    # Sensor source for the mp baseline: 'd415' remaps to the D415 RGBD cloud,
    # 'fusion' keeps the default /drone/tof_merged/points (5x ring ToFs merged).
    # Implemented as a remap target: self-remap is a no-op so fusion = identity.
    mp_cloud_topic = PythonExpression([
        "'/drone/rgbd/points' if '", sensor_source, "' == 'd415' else '/drone/tof_merged/points'"])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_mp', default_value='true',
            description='Use Motion Primitive planner (true) or legacy VFH3D (false).'),

        DeclareLaunchArgument(
            'planner_backend', default_value='mp',
            description=(
                'When use_mp:=true, picks the MP variant: '
                '"mp" (raw-cloud baseline) or "mp_esdf" (nvblox ESDF voxel hash).'
                'For the ESDF variant, ensure cuVSLAM+nvblox stack is up '
                'and publishing /nvblox_node/static_esdf_pointcloud.')),

        DeclareLaunchArgument(
            'sensor_source', default_value='d415',
            description=(
                'Obstacle cloud source for planner_backend:=mp. '
                '"d415" = forward-facing RGBD only (~65 deg FOV, long range). '
                '"fusion" = 5x ring ToFs merged in base_link (~360 deg, ~3 m range). '
                'Ignored for planner_backend:=mp_esdf (always uses nvblox ESDF).')),

        Node(
            package='uav_local_planner',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            output='screen',
            parameters=[{
                'acceptance_radius':       1.0,
                'final_acceptance_radius': 0.25,
                'replan_deviation':        2.0,
                'publish_rate_hz':         20.0,
                'use_sim_time':            True,
            }],
        ),

        # ── Motion Primitive — raw cloud baseline ─────────────────────────
        # planner_backend:=mp (default). Obstacle cloud source picked by
        # sensor_source:={d415,fusion}. Self-remap (fusion → fusion) is a no-op
        # so we can use a single Node block for both configs.
        Node(
            condition=IfCondition(is_mp_baseline),
            package='uav_local_planner',
            executable='mp_node',
            name='mp_node',
            output='screen',
            parameters=[mp_config, {'use_sim_time': True}],
            remappings=[
                ('/drone/tof_merged/points', mp_cloud_topic),
            ],
        ),

        # ── Motion Primitive — nvblox ESDF variant ────────────────────────
        # planner_backend:=mp_esdf. Reads /nvblox_node/static_esdf_pointcloud.
        # Same arc primitives, same scoring weights, different obstacle source.
        Node(
            condition=IfCondition(is_mp_esdf),
            package='uav_local_planner',
            executable='mp_esdf_node',
            name='mp_esdf_node',
            output='screen',
            parameters=[mp_config, {'use_sim_time': True}],
        ),

        # ── Legacy VFH3D (OctoMap-based) — launch with use_mp:=false ──────
        Node(
            condition=UnlessCondition(use_mp),
            package='uav_local_planner',
            executable='vfh3d_node',
            name='vfh3d_node',
            output='screen',
            parameters=[{
                'az_sectors':     72,
                'el_sectors':     36,
                'bbox_radius':    3.0,
                'robot_radius':   0.30,
                'safety_radius':  0.15,
                'h_high':         0.5,
                'h_low':          0.2,
                'w_goal':         3.0,
                'w_current':      1.0,
                'w_prev':         2.0,
                'max_speed':      2.5,
                'min_speed':      0.2,
                'max_vz':         1.0,
                'min_clearance':  0.6,
                'update_rate_hz': 20.0,
                'use_sim_time':   True,
            }],
        ),
    ])
