"""
pixhawk_serial.launch.py

Starts MicroXRCE-DDS Agent in serial mode, bridging PX4 (Pixhawk via USB-C to
Orin Nano) <-> ROS 2 over /dev/ttyACM0 by default. Produces the /fmu/out/* and
accepts the /fmu/in/* topics that the planner stack uses.

Equivalent to the sim path that runs the agent in UDP mode (run_server.sh T2).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyACM0',
        description='Serial device path for Pixhawk USB-C connection.')

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description='Baud rate for the Pixhawk MAVLink/uXRCE link.')

    agent_arg = DeclareLaunchArgument(
        'agent',
        default_value='MicroXRCEAgent',
        description='Path to MicroXRCEAgent binary (must be on PATH or absolute).')

    return LaunchDescription([
        dev_arg,
        baud_arg,
        agent_arg,
        ExecuteProcess(
            cmd=[
                LaunchConfiguration('agent'),
                'serial',
                '--dev', LaunchConfiguration('dev'),
                '-b',    LaunchConfiguration('baud'),
            ],
            name='micro_xrce_dds_agent_serial',
            output='screen',
            shell=False,
        ),
    ])
