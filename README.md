# UAV Obstacle Avoidance Stack

A ROS 2 (Humble) obstacle avoidance system for UAVs using 3DVFH+ local planning on a live OctoMap, with a plugin-based global planner (A* default). Designed for PX4 SITL with Gazebo Harmonic, and validated on an F450 frame with a 5× ToF depth sensor array.

---

## Architecture

```
Sensing          5× A010 ToF cameras (100×100, 20 Hz, 72° ring)
                 + RGBD front camera + bottom depth camera
      ↓
Fusion           cloud_merge — merges point clouds into base_link frame
      ↓
Mapping          uav_mapping — OctoMap 3D occupancy (0.2 m voxels)
      ↓
Global Planner   uav_planner_interface — A* on OctoMap (plugin-based)
      ↓
Local Planner    uav_local_planner — 3DVFH+ reactive avoidance at 20 Hz
      ↓
Control          uav_control — velocity setpoints → PX4 TrajectorySetpoint (NED)
```

---

## Packages

| Package | Description |
|---|---|
| `cloud_merge` | Point cloud merge node, static TF broadcaster, PX4 odometry bridge (NED→ENU) |
| `uav_mapping` | OctoMap server configuration and launch |
| `uav_planner_interface` | `GlobalPlannerBase` plugin interface, A* planner plugin, action server |
| `uav_local_planner` | 3DVFH+ algorithm, VFH3D ROS 2 node, waypoint manager |
| `uav_control` | Full lifecycle setpoint publisher (takeoff → hover → autonomous) |

---

## Requirements

### System
- ROS 2 Humble
- Gazebo Harmonic
- PX4 Autopilot (SITL)
- Python 3.10+

### ROS 2 packages
```bash
sudo apt install \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-octomap-server \
  ros-humble-octomap-ros \
  ros-humble-octomap-msgs \
  ros-humble-octomap \
  ros-humble-pluginlib \
  ros-humble-rclcpp-action \
  ros-humble-eigen3-cmake-module \
  libeigen3-dev
```

### Workspace dependencies
`px4_msgs` must be present in your workspace:
```bash
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
```

---

## Setup

### 1. Clone and build
```bash
git clone https://github.com/shantam98/px4_sitl_planner.git
cd px4_sitl_planner
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Configure sensor transforms
Edit `cloud_merge/src/tf_static_broadcaster.cpp` if your sensor mounting positions differ from the default F450 layout:

| Sensor | Position (x, y, z) | Yaw |
|---|---|---|
| tof_0 | (0.25, 0.00, 0) | 0° (front) |
| tof_1 | (0.077, 0.236, 0) | 72° (front-left) |
| tof_2 | (-0.202, 0.147, 0) | 144° (rear-left) |
| tof_3 | (-0.202, -0.147, 0) | -144° (rear-right) |
| tof_4 | (0.077, -0.236, 0) | -72° (front-right) |

### 3. Configure ROS-Gazebo bridge
The included `sensor_bridge.yaml` maps Gazebo sensor topics to ROS 2:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=<path>/sensor_bridge.yaml
```

---

## Launch

Start each component in a separate terminal **in order**:

```bash
# 1. Sensor fusion + TF + odometry bridge
ros2 launch cloud_merge fusion.launch.py

# 2. OctoMap 3D mapping
ros2 launch uav_mapping mapping.launch.py

# 3. Global planner action server
ros2 launch uav_planner_interface planner.launch.py

# 4. Local planner + waypoint manager
ros2 launch uav_local_planner local_planner.launch.py

# 5. Flight controller (handles takeoff automatically)
ros2 launch uav_control control.launch.py
```

The control node handles the full flight lifecycle automatically:
- **STARTUP** — 5 s EKF2 warmup
- **TAKEOFF** — arms, engages offboard, climbs to 5 m
- **HOVER** — holds position, waits for a navigation goal
- **AUTONOMOUS** — executes VFH3D velocity commands
- **HOVER** — returns to hover on mission complete

---

## Sending Goals

Once the drone is in HOVER state, send a navigation goal via the action interface:

```bash
ros2 action send_goal /uav/navigate_to_goal \
  uav_planner_interface/action/NavigateToGoal \
  "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}}, planning_timeout_sec: 15.0}"
```

Goals are in the `map` frame (ENU). The planner will return `SUCCESS`, `NO_PATH`, `TIMEOUT`, or `INVALID_GOAL`.

---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/drone/tof_merged/points` | `PointCloud2` | Fused 5× ToF cloud in `base_link` |
| `/octomap_binary` | `Octomap` | Live 3D occupancy map |
| `/octomap_occupied_cells_vis_array` | `MarkerArray` | RViz2 visualisation |
| `/uav/global_path` | `Path` | A* planned path in `map` frame |
| `/uav/current_waypoint` | `PointStamped` | Active waypoint fed to VFH3D |
| `/uav/cmd_vel` | `TwistStamped` | VFH3D velocity output (ENU) |
| `/uav/vfh_status` | `String` | `IDLE` / `NOMINAL` / `AVOIDING` / `ESTOP` |
| `/uav/mission_complete` | `Bool` | Fires true when final waypoint reached |
| `/drone/odom` | `Odometry` | Drone pose in `map` frame (ENU) |

---

## RViz2 Setup

Set **Fixed Frame** to `map`, then add:

- `PointCloud2` → `/drone/tof_merged/points` — color by Z axis, Reliability: Best Effort
- `MarkerArray` → `/octomap_occupied_cells_vis_array`
- `Path` → `/uav/global_path`
- `Odometry` → `/drone/odom`
- `TF` — to visualise sensor frames on drone body

---

## Parameters

### cloud_merge
| Parameter | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame to merge clouds into |
| `tf_timeout_sec` | `0.1` | TF lookup timeout |

### uav_mapping
| Parameter | Default | Description |
|---|---|---|
| `resolution` | `0.2` | OctoMap voxel size (metres) |
| `sensor_model_max_range` | `3.0` | Max sensor range (A010 = 3 m) |
| `transform_tolerance` | `0.2` | TF timestamp tolerance |

### uav_planner_interface
| Parameter | Default | Description |
|---|---|---|
| `planner_plugin` | `uav_planning::AStarPlanner` | Plugin class name |
| `robot_radius` | `0.30` | F450 radius with props (metres) |
| `inflation_radius` | `0.15` | Extra obstacle clearance |
| `max_planning_time` | `10.0` | A* timeout (seconds) |

### uav_local_planner
| Parameter | Default | Description |
|---|---|---|
| `max_speed` | `1.0` | Max flight speed (m/s) |
| `min_clearance` | `0.6` | E-stop distance (metres) |
| `h_high` / `h_low` | `0.5` / `0.2` | VFH histogram thresholds |
| `acceptance_radius` | `0.4` | Waypoint acceptance (metres) |
| `final_acceptance_radius` | `0.25` | Final waypoint acceptance |

### uav_control
| Parameter | Default | Description |
|---|---|---|
| `takeoff_height` | `-5.0` | Takeoff altitude NED (negative = up) |
| `startup_delay_s` | `5.0` | EKF2 warmup delay |
| `cmd_timeout_s` | `0.5` | Stale cmd_vel fallback to hover |

---

## Adding a New Global Planner Plugin

The global planner is plugin-based via `pluginlib`. To add a custom planner (e.g. RRT*, ML policy):

1. Create a class inheriting `uav_planning::GlobalPlannerBase`
2. Implement `initialize()`, `computePath()`, and `name()`
3. Register with `PLUGINLIB_EXPORT_CLASS`
4. Add a `planner_plugins.xml` entry
5. Switch at launch time:

```bash
ros2 launch uav_planner_interface planner.launch.py \
  planner_plugin:=my_package::MyPlanner
```

No changes to any other node required.

---

## TF Tree

```
map
└── odom                          (static identity — replace with SLAM)
    └── base_link                 (PX4 odometry, NED→ENU converted)
        ├── tof_array_link        (identity joint)
        │   ├── tof_0_link        (front, 0°)
        │   ├── tof_1_link        (front-left, 72°)
        │   ├── tof_2_link        (rear-left, 144°)
        │   ├── tof_3_link        (rear-right, -144°)
        │   └── tof_4_link        (front-right, -72°)
        ├── rgbd_cam_link         (front RGBD, 0.2m forward)
        └── bottom_cam_link       (downward, pitched 90°)
```

---

## Known Limitations & Future Work

- `map → odom` is a static identity transform. Replace with a SLAM or VIO solution (e.g. RTAB-Map, ORB-SLAM3) for drift-free long-range navigation.
- The A* planner treats unknown voxels as free. In highly unstructured environments consider switching to a conservative unknown=occupied policy once the map is sufficiently explored.
- VFH3D elevation resolution (5°) may cause the drone to miss narrow vertical gaps. Tune `el_sectors` for your specific environment.
- The global planner plugin interface is ready for ML-based planners — the `computePath()` signature accepts the full OctoMap and a feedback callback, giving a policy full environmental context.

---

## License

MIT


---

## Requirements

### System
- ROS 2 Humble
- Gazebo Harmonic
- PX4 Autopilot (SITL)
- Python 3.10+

### ROS 2 packages
```bash
sudo apt install \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-octomap-server \
  ros-humble-octomap-ros \
  ros-humble-octomap-msgs \
  ros-humble-octomap \
  ros-humble-pluginlib \
  ros-humble-rclcpp-action \
  ros-humble-eigen3-cmake-module \
  libeigen3-dev
```

### Workspace dependencies
`px4_msgs` must be present in your workspace:
```bash
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
```

---

## Setup

### 1. Clone and build
```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Configure sensor transforms
Edit `uav_depth_fusion/src/tf_static_broadcaster.cpp` if your sensor mounting positions differ from the default F450 layout:

| Sensor | Position (x, y, z) | Yaw |
|---|---|---|
| tof_0 | (0.25, 0.00, 0) | 0° (front) |
| tof_1 | (0.077, 0.236, 0) | 72° (front-left) |
| tof_2 | (-0.202, 0.147, 0) | 144° (rear-left) |
| tof_3 | (-0.202, -0.147, 0) | -144° (rear-right) |
| tof_4 | (0.077, -0.236, 0) | -72° (front-right) |

### 3. Configure ROS-Gazebo bridge
The included `sensor_bridge.yaml` maps Gazebo sensor topics to ROS 2:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=<path>/sensor_bridge.yaml
```

---

## Launch

Start each component in a separate terminal **in order**:

```bash
# 1. Sensor fusion + TF + odometry bridge
ros2 launch uav_depth_fusion fusion.launch.py

# 2. OctoMap 3D mapping
ros2 launch uav_mapping mapping.launch.py

# 3. Global planner action server
ros2 launch uav_planner_interface planner.launch.py

# 4. Local planner + waypoint manager
ros2 launch uav_local_planner local_planner.launch.py

# 5. Flight controller (handles takeoff automatically)
ros2 launch uav_control control.launch.py
```

The control node handles the full flight lifecycle automatically:
- **STARTUP** — 5 s EKF2 warmup
- **TAKEOFF** — arms, engages offboard, climbs to 5 m
- **HOVER** — holds position, waits for a navigation goal
- **AUTONOMOUS** — executes VFH3D velocity commands
- **HOVER** — returns to hover on mission complete

---

## Sending Goals

Once the drone is in HOVER state, send a navigation goal via the action interface:

```bash
ros2 action send_goal /uav/navigate_to_goal \
  uav_planner_interface/action/NavigateToGoal \
  "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}}, planning_timeout_sec: 15.0}"
```

Goals are in the `map` frame (ENU). The planner will return `SUCCESS`, `NO_PATH`, `TIMEOUT`, or `INVALID_GOAL`.

---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/drone/tof_merged/points` | `PointCloud2` | Fused 5× ToF cloud in `base_link` |
| `/octomap_binary` | `Octomap` | Live 3D occupancy map |
| `/octomap_occupied_cells_vis_array` | `MarkerArray` | RViz2 visualisation |
| `/uav/global_path` | `Path` | A* planned path in `map` frame |
| `/uav/current_waypoint` | `PointStamped` | Active waypoint fed to VFH3D |
| `/uav/cmd_vel` | `TwistStamped` | VFH3D velocity output (ENU) |
| `/uav/vfh_status` | `String` | `IDLE` / `NOMINAL` / `AVOIDING` / `ESTOP` |
| `/uav/mission_complete` | `Bool` | Fires true when final waypoint reached |
| `/drone/odom` | `Odometry` | Drone pose in `map` frame (ENU) |

---

## RViz2 Setup

Set **Fixed Frame** to `map`, then add:

- `PointCloud2` → `/drone/tof_merged/points` — color by Z axis, Reliability: Best Effort
- `MarkerArray` → `/octomap_occupied_cells_vis_array`
- `Path` → `/uav/global_path`
- `Odometry` → `/drone/odom`
- `TF` — to visualise sensor frames on drone body

---

## Parameters

### uav_depth_fusion
| Parameter | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame to merge clouds into |
| `tf_timeout_sec` | `0.1` | TF lookup timeout |

### uav_mapping
| Parameter | Default | Description |
|---|---|---|
| `resolution` | `0.2` | OctoMap voxel size (metres) |
| `sensor_model_max_range` | `3.0` | Max sensor range (A010 = 3 m) |
| `transform_tolerance` | `0.2` | TF timestamp tolerance |

### uav_planner_interface
| Parameter | Default | Description |
|---|---|---|
| `planner_plugin` | `uav_planning::AStarPlanner` | Plugin class name |
| `robot_radius` | `0.30` | F450 radius with props (metres) |
| `inflation_radius` | `0.15` | Extra obstacle clearance |
| `max_planning_time` | `10.0` | A* timeout (seconds) |

### uav_local_planner
| Parameter | Default | Description |
|---|---|---|
| `max_speed` | `1.0` | Max flight speed (m/s) |
| `min_clearance` | `0.6` | E-stop distance (metres) |
| `h_high` / `h_low` | `0.5` / `0.2` | VFH histogram thresholds |
| `acceptance_radius` | `0.4` | Waypoint acceptance (metres) |
| `final_acceptance_radius` | `0.25` | Final waypoint acceptance |

### uav_control
| Parameter | Default | Description |
|---|---|---|
| `takeoff_height` | `-5.0` | Takeoff altitude NED (negative = up) |
| `startup_delay_s` | `5.0` | EKF2 warmup delay |
| `cmd_timeout_s` | `0.5` | Stale cmd_vel fallback to hover |

---

## Adding a New Global Planner Plugin

The global planner is plugin-based via `pluginlib`. To add a custom planner (e.g. RRT*, ML policy):

1. Create a class inheriting `uav_planning::GlobalPlannerBase`
2. Implement `initialize()`, `computePath()`, and `name()`
3. Register with `PLUGINLIB_EXPORT_CLASS`
4. Add a `planner_plugins.xml` entry
5. Switch at launch time:

```bash
ros2 launch uav_planner_interface planner.launch.py \
  planner_plugin:=my_package::MyPlanner
```

No changes to any other node required.

---

## TF Tree

```
map
└── odom                          (static identity — replace with SLAM)
    └── base_link                 (PX4 odometry, NED→ENU converted)
        ├── tof_array_link        (identity joint)
        │   ├── tof_0_link        (front, 0°)
        │   ├── tof_1_link        (front-left, 72°)
        │   ├── tof_2_link        (rear-left, 144°)
        │   ├── tof_3_link        (rear-right, -144°)
        │   └── tof_4_link        (front-right, -72°)
        ├── rgbd_cam_link         (front RGBD, 0.2m forward)
        └── bottom_cam_link       (downward, pitched 90°)
```

---

