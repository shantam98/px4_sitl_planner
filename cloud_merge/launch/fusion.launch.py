from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('uav_depth_fusion')

    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.1'),
        DeclareLaunchArgument(
            'publish_map_to_odom', default_value='true',
            description='Publish static identity map→odom. Set false when cuVSLAM owns this edge.'),

        # 1. Static TF broadcaster — sensor frames relative to base_link
        #    use_sim_time so static TF stamps match cuVSLAM's Gazebo /clock view.
        Node(
            package='uav_depth_fusion',
            executable='tf_static_broadcaster',
            name='tf_static_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # 2. PX4 odometry bridge — publishes odom→base_link TF + /drone/odom
        #    Also owns the static map→odom edge unless gated off (cuVSLAM mode).
        Node(
            package='uav_depth_fusion',
            executable='px4_odom_bridge',
            name='px4_odom_bridge',
            output='screen',
            parameters=[{
                # ParameterValue with value_type=bool forces the LaunchConfiguration
                # string ("true"/"false") to coerce into a real bool, matching the
                # C++ declare_parameter type. Without this, Humble throws
                # ParameterTypeException and kills the node.
                'publish_map_to_odom': ParameterValue(
                    LaunchConfiguration('publish_map_to_odom'),
                    value_type=bool),
                'use_sim_time': True,
            }],
        ),

        # 3. Point cloud merge — fuses 5x ToF into /drone/tof_merged/points
        Node(
            package='uav_depth_fusion',
            executable='cloud_merge_node',
            name='cloud_merge_node',
            output='screen',
            parameters=[{
                'target_frame':    LaunchConfiguration('target_frame'),
                'tf_timeout_sec':  LaunchConfiguration('tf_timeout_sec'),
                'use_sim_time':    True,
            }],
        ),

        # 4. Static stereo camera_info publisher — workaround for Gazebo Harmonic
        #    not emitting CameraInfo. Intrinsics match the SDF stereo IR pair
        #    (D415-equivalent: 640x480, 65 deg HFOV, 55 mm baseline).
        Node(
            package='uav_depth_fusion',
            executable='stereo_camera_info_publisher',
            name='stereo_camera_info_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'width': 640,
                'height': 480,
            }],
        ),
    ])