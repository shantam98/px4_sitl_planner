import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_mp              = LaunchConfiguration('use_mp',              default='true')
    with_global_planner = LaunchConfiguration('with_global_planner', default='true')
    with_vslam          = LaunchConfiguration('with_vslam',          default='false')

    fusion_dir  = get_package_share_directory('uav_depth_fusion')
    mapping_dir = get_package_share_directory('uav_mapping')
    planner_dir = get_package_share_directory('uav_planner_interface')
    local_dir   = get_package_share_directory('uav_local_planner')
    control_dir = get_package_share_directory('uav_control')

    # When cuVSLAM owns map→odom, OctoMap is replaced by nvblox (launched
    # separately via vslam.launch.py) — so skip the OctoMap include here.
    # OctoMap runs only when BOTH with_global_planner AND NOT with_vslam.
    octomap_condition = PythonExpression([
        "'", with_global_planner, "' == 'true' and '", with_vslam, "' != 'true'"])

    # Gate the static map→odom in fusion.launch.py when VSLAM is on (cuVSLAM
    # publishes the dynamic, corrected edge).
    publish_map_to_odom = PythonExpression([
        "'false' if '", with_vslam, "' == 'true' else 'true'"])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_mp', default_value='true',
            description='true = Motion Primitive planner, false = legacy VFH3D'),

        DeclareLaunchArgument(
            'with_global_planner', default_value='true',
            description='Launch OctoMap + A* global planner (required for nav goals).'),

        DeclareLaunchArgument(
            'with_vslam', default_value='false',
            description=(
                'Enable cuVSLAM + nvblox path: gates static map→odom and skips '
                'OctoMap. NOTE: this flag only flips internal wiring — launch '
                'vslam.launch.py separately (Orin native or inside isaac_vslam.sif).')),

        # ── T = 0 s : Sensor fusion ───────────────────────────────────────
        # Must come first: provides /drone/tof_merged/points, /drone/odom, TF tree.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fusion_dir, 'launch', 'fusion.launch.py')),
            launch_arguments={
                'publish_map_to_odom': publish_map_to_odom,
            }.items()),

        # ── T = 3 s : Local planner + waypoint manager ────────────────────
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(local_dir, 'launch', 'local_planner.launch.py')),
                launch_arguments={'use_mp': use_mp}.items())]),

        # ── T = 3 s : OctoMap (only when VSLAM is OFF) ────────────────────
        # nvblox replaces OctoMap when with_vslam:=true.
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mapping_dir, 'launch', 'mapping.launch.py')),
                condition=IfCondition(octomap_condition))]),

        # ── T = 7 s : Global planner (optional) ──────────────────────────
        TimerAction(period=7.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(planner_dir, 'launch', 'planner.launch.py')),
                condition=IfCondition(with_global_planner))]),

        # ── T = 5 s : Flight controller ───────────────────────────────────
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_dir, 'launch', 'control.launch.py')))]),
    ])
