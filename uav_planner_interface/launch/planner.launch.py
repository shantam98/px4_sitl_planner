from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner_plugin',
            default_value='uav_planning::AStarPlanner'),
        DeclareLaunchArgument('robot_radius',     default_value='0.30'),
        DeclareLaunchArgument('inflation_radius', default_value='0.15'),
        DeclareLaunchArgument('goal_tolerance',   default_value='0.25'),
        DeclareLaunchArgument('max_altitude',     default_value='5.0'),
        DeclareLaunchArgument('min_altitude',     default_value='0.3'),

        Node(
            package='uav_planner_interface',
            executable='planner_server_node',
            name='planner_server',
            output='screen',
            parameters=[{
                'planner_plugin':    LaunchConfiguration('planner_plugin'),
                'robot_radius':      LaunchConfiguration('robot_radius'),
                'inflation_radius':  LaunchConfiguration('inflation_radius'),
                'goal_tolerance':    LaunchConfiguration('goal_tolerance'),
                'max_altitude':      LaunchConfiguration('max_altitude'),
                'min_altitude':      LaunchConfiguration('min_altitude'),
            }],
        ),
    ])