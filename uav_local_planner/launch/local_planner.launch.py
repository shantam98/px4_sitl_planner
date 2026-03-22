from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='uav_local_planner',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            output='screen',
            parameters=[{
                'acceptance_radius':       0.4,
                'final_acceptance_radius': 0.25,
                'replan_deviation':        2.0,
                'publish_rate_hz':         20.0,
            }],
        ),

        Node(
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
                'max_speed':      1.0,
                'min_speed':      0.1,
                'max_vz':         0.5,
                'min_clearance':  0.5,
                'update_rate_hz': 20.0,
            }],
        ),
    ])