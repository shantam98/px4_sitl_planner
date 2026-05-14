"""
maixsense_tof_array.launch.py

Launches 5 instances of sipeed_tof_ms_a010 (one per MaixSense MS-A010 ToF
sensor) plus 5 instances of tof_frame_relay to rewrite the hardcoded
`frame_id = "tof"` to the per-sensor `tof_<N>_link`.

Requires:
  - sipeed_tof_ms_a010 built in a separate workspace (maixsense_ws), sourced
    BEFORE this launch
  - udev rules installed creating /dev/maixsense_tof_<0..4> symlinks
    (see udev/99-uav-hardware.rules; populate from discover_maixsense.sh)

Topic mapping per sensor N:
  sipeed pubs:   /drone/tof_N/raw/{depth,cloud}   (relative via namespace)
  relay pubs:    /drone/tof_N/{depth,points}      (with frame_id=tof_N_link)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


N_SENSORS = 5


def generate_launch_description():
    actions = []

    for i in range(N_SENSORS):
        device = f'/dev/maixsense_tof_{i}'
        ns     = f'tof_{i}_raw'

        # 1. Sipeed driver — talks to one MaixSense, publishes ./depth and ./cloud
        actions.append(Node(
            package='sipeed_tof_ms_a010',
            executable='publisher',
            name='sipeed_tof_publisher',
            namespace=ns,
            output='screen',
            parameters=[{'device': device}],
        ))

        # 2. Frame-id relay — fixes the hardcoded "tof" frame_id and
        #    republishes on the canonical /drone/tof_N/* topics.
        actions.append(Node(
            package='uav_hardware_bringup',
            executable='tof_frame_relay',
            name=f'tof_frame_relay_{i}',
            output='screen',
            parameters=[{
                'target_frame': f'tof_{i}_link',
                'depth_in':     f'/{ns}/depth',
                'cloud_in':     f'/{ns}/cloud',
                'depth_out':    f'/drone/tof_{i}/depth',
                'cloud_out':    f'/drone/tof_{i}/points',
            }],
        ))

    return LaunchDescription(actions)
