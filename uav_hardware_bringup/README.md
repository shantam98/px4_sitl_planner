# uav_hardware_bringup

Launch + driver orchestration for the real UAV hardware. Sibling to `uav_bringup`
(which orchestrates the simulation stack). After running `hardware_stack.launch.py`
on the Orin Nano, the same topic contract as the simulator is available — so the
rest of `planner_ws` (motion primitives, A*, frontier explorer, control) runs
unchanged.

## Hardware assumed

- **Pixhawk FMU-V3** running PX4 v1.16.1, USB-C to Orin Nano (`/dev/ttyACM0`)
- **Intel RealSense D415**, USB 3.0 to Orin Nano
- **5× MaixSense MS-A010 ToF** sensors, 4 via USB hub + 1 direct = 5 USB serial ports
- **Jetson Orin Nano 8 GB** running JetPack ROS 2 Humble

## Topic contract

After this package is running, downstream nodes see:

| Topic | Type | Source |
|---|---|---|
| `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_odometry`, `/fmu/in/*` | px4_msgs | MicroXRCEAgent (serial mode) |
| `/drone/stereo/{left,right}/{image,camera_info}` | sensor_msgs/{Image,CameraInfo} | D415 infra1/infra2 (remapped) |
| `/drone/rgbd/{image,depth,camera_info,points}` | sensor_msgs/{Image,CameraInfo,PointCloud2} | D415 color + aligned depth (remapped) |
| `/drone/tof_<N>/{depth,points}`  N=0..4 | sensor_msgs/{Image,PointCloud2} | sipeed_tof_ms_a010 → tof_frame_relay |
| `/drone/imu` | sensor_msgs/Imu | (planned) Pixhawk via DDS — D415 has NO IMU |
| `/drone/odom` | nav_msgs/Odometry | px4_odom_bridge (existing, unchanged from sim) |
| `tf: map → odom`, `odom → base_link`, sensor frames | TF2 | tf_static_broadcaster + px4_odom_bridge |

## Prerequisites

### apt packages on the Orin Nano

```bash
sudo apt update
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    ros-humble-realsense2-description \
    ros-humble-rclcpp-components
```

### Adjacent workspaces

- `~/irobot/maixsense_ws/` — contains `sipeed_tof_ms_a010` (ToF driver). Build with `colcon build` and source before launching.
- `~/irobot/px4_ros2_ws/` — `px4_msgs` + `px4_ros_com`. Required for DDS topic types.
- `~/irobot/planner_ws/` — this package + the rest of the planner stack.
- `~/irobot/Micro-XRCE-DDS-Agent/build/` — MicroXRCEAgent binary on PATH or override via `agent:=` launch arg.

### udev rules (one-time, per Orin)

The 5 MaixSense devices show up as `/dev/ttyUSB0..4` in unpredictable order on each boot. Pin them to stable names by serial:

```bash
cd ~/irobot/planner_ws/src/uav_hardware_bringup

# 1. Discover serials. Plug each sensor IN TURN and run:
./scripts/discover_maixsense.sh
# Note which serial corresponds to which physical direction
# (front, front-left, rear-left, rear-right, front-right).

# 2. Edit udev/99-uav-hardware.rules — replace the 5 REPLACE_WITH_* placeholders
#    with the discovered serials.
nano udev/99-uav-hardware.rules

# 3. Install:
sudo cp udev/99-uav-hardware.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger

# 4. Unplug + replug each sensor. Verify:
ls -la /dev/maixsense_tof_*
# Expect: 5 symlinks pointing to /dev/ttyUSB<X>
```

## Build

```bash
cd ~/irobot/planner_ws
colcon build --packages-select uav_hardware_bringup --symlink-install
source install/setup.bash
```

## Launch

After build + udev setup, over SSH on the Orin:

```bash
# Source all the workspaces (order matters: ROS base → DDS msgs → ToF → planner)
source /opt/ros/humble/setup.bash
source ~/irobot/px4_ros2_ws/install/setup.bash
source ~/irobot/maixsense_ws/install/setup.bash
source ~/irobot/planner_ws/install/setup.bash

# All hardware up
ros2 launch uav_hardware_bringup hardware_stack.launch.py

# OR just one sub-system at a time during bring-up debugging
ros2 launch uav_hardware_bringup realsense.launch.py
ros2 launch uav_hardware_bringup maixsense_tof_array.launch.py
ros2 launch uav_hardware_bringup pixhawk_serial.launch.py
```

Once `hardware_stack` is up and topics flow, run the planner stack as you would
in sim:

```bash
# In a separate SSH session, with same sourcing
ros2 launch uav_bringup full_stack.launch.py
```

`uav_bringup`'s nodes consume `/drone/*` and `/fmu/*` topics — they have no idea
the source is real hardware rather than Gazebo.

## Sub-launches

| File | Starts |
|---|---|
| `hardware_stack.launch.py` | Everything below, staggered (TFs immediate → DDS → RealSense at +3s → ToFs at +6s → cloud_merge at +9s) |
| `pixhawk_serial.launch.py` | `MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600` |
| `realsense.launch.py` | D415 as composable node; remaps to `/drone/stereo/*` + `/drone/rgbd/*` |
| `maixsense_tof_array.launch.py` | 5× `sipeed_tof_ms_a010` + 5× `tof_frame_relay` (this package) |
| `tf_hardware.launch.py` | Static TFs (reuses `tf_static_broadcaster` from `uav_depth_fusion`, params from `config/hardware_tf.yaml`) |

## Config

| File | What |
|---|---|
| `config/realsense_d415.yaml` | Reference of D415 params used in `realsense.launch.py`. Currently most params are baked into the launch — this file documents them and is a target for a future parameter-file refactor. |
| `config/maixsense_tofs.yaml` | 5-sensor layout (direction ↔ device path ↔ frame id) — reference only; launch file currently bakes the same mapping. |
| `config/hardware_tf.yaml` | Sensor mount offsets relative to `base_link`. Mirrors the SDF in v1; edit when real mounts are measured. |

## Notes / gotchas

- **D415 has no IMU.** `enable_gyro` / `enable_accel` are no-ops. `/drone/imu` will eventually come from Pixhawk via DDS — work item, not addressed in v1 of this package.
- **USB 2 limits** the D415 to a subset of streams at reduced resolution. The Orin Nano has USB 3 — verify the cable supports it.
- **ToF frame_id workaround**: the upstream `sipeed_tof_ms_a010` driver hardcodes `header.frame_id = "tof"`. We launch 5 copies and use `tof_frame_relay` (this package) to retag each output to `tof_<N>_link`. If upstream ever exposes a `frame_id` parameter, we remove the relay.
- **udev rules are critical.** Without serial-pinned device names, ToF directions are scrambled across reboots and the obstacle avoidance becomes a coin flip.
- **`map → odom`**: in v1 this is a static identity (`publish_map_to_odom: true` in `hardware_tf.yaml`). When cuVSLAM lands on the Orin (Phase 2/8 of the master plan), flip this to `false` so cuVSLAM owns that edge.

## See also

- `../uav_bringup/` — the simulation-side counterpart
- `../README.md` (planner_ws root) — full project overview
- `../PHASE2_VSLAM_INTEGRATION.md` — cuVSLAM integration plan (sim first, hardware later)
- `../../px4_sim/PX4_VERSION_MIGRATION.md` — keeping SITL and Pixhawk firmware aligned
