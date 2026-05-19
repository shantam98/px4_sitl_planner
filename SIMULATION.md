# Simulation Setup

This document enumerates every component required to run the InspectBot stack in simulation, the role each plays, and the communication channels between them. It is the source-of-truth reference for the simulation schematic diagram.

The goal of the simulation environment is to host PX4 SITL, a Gazebo physics world with the F450 model and its sensors, and any of the four InspectBot modules under test — all running inside containerised environments on the NUS Vanda Cluster.

---

## 1. Components

| # | Component | Role | Hosted in |
|---|---|---|---|
| 1 | **NUS Vanda Cluster (A40 GPU node)** | Compute substrate. Provides the CUDA / GPU resources required by Isaac ROS (cuVSLAM, nvblox) and the Gazebo renderer. Selected over local laptops because no team workstation could run PX4-SITL + Gazebo + Isaac ROS concurrently. | Bare-metal node, accessed via SSH / desktop session. |
| 2 | **Apptainer containers (`.sif` files)** | Reproducible OS/library environment. Two containers are used: `isaac_vslam.sif` (Isaac ROS Visual SLAM + nvblox + ROS 2 Humble) and `uav_stack.sif` (ROS 2 Humble + PX4 build deps + Gazebo Harmonic + planner stack). The cluster does not allow native package installs, so all software lives inside these containers. | Cluster filesystem. |
| 3 | **PX4 SITL** | Software-in-the-loop instance of the PX4 v1.16.1 autopilot firmware. Runs EKF2 state estimation, attitude / position controllers, and the offboard interface. Reads simulated IMU/GPS/baro/mag from Gazebo and outputs motor commands. | `uav_stack.sif` |
| 4 | **Gazebo Harmonic** | Physics + sensor simulator. Loads the F450 SDF model and the scenario world, simulates rigid-body dynamics + sensor plugins (D415 RGBD, 5× MaixSense ToFs, stereo IR pair, bottom camera), and renders the scene. | `uav_stack.sif` |
| 5 | **px4_sim repo (`~/irobot/px4_sim/`)** | All simulation-side artifacts the cluster needs: F450 model SDF (`px4_files/f450_base/model.sdf`), per-scenario worlds (`px4_files/scenario_*.sdf`), `sensor_bridge.yaml` (Gazebo↔ROS topic map), `init_env.sh` (environment setup), `run_sim.sh` (one-shot launcher), and the `ablation/` harness. | Mounted into the container at runtime. |
| 6 | **MicroXRCEAgent** | uXRCE-DDS bridge between PX4 and ROS 2. PX4 publishes its native messages over a UDP socket; the agent translates them into ROS 2 DDS topics under the `/fmu/*` namespace, and vice versa for inbound commands. | `uav_stack.sif` |
| 7 | **`ros_gz_bridge parameter_bridge`** | Topic bridge between Gazebo Transport and ROS 2 DDS. Configured by `sensor_bridge.yaml`; converts each Gazebo sensor topic (e.g. `/world/.../sensor/depth_image`) into a ROS 2 topic (e.g. `/drone/rgbd/depth`) with the correct message type. | `uav_stack.sif` |
| 8 | **planner_ws (this repo)** | The InspectBot ROS 2 workspace. Provides the four module layers — Cognitive (uav_brain), Semantic (uav_semantic_slam), Local Planner (uav_local_planner + cloud_merge), Failsafe (emergency_landing_sim + uav_safety) — plus the bringup launch files. Any one of the four modules can be the unit under test. | `uav_stack.sif` (planner-side) or `isaac_vslam.sif` (when cuVSLAM/nvblox are involved). |

---

## 2. Communication Channels

Every component-to-component edge that matters for the schematic, with protocol and direction.

| Edge | Producer → Consumer | Protocol / Port | Payload |
|---|---|---|---|
| **E1** | Gazebo → PX4 SITL | UDP, port 4560 (PX4 simulator API) | Simulated IMU, GPS, baro, mag, motor speeds (closed loop) |
| **E2** | PX4 SITL ↔ MicroXRCEAgent | uXRCE-DDS over UDP, port 8888 | PX4 native messages (`VehicleOdometry`, `VehicleStatus`, `TrajectorySetpoint`, etc.) |
| **E3** | MicroXRCEAgent → ROS 2 DDS | DDS multicast on loopback | ROS topics under `/fmu/out/*` and `/fmu/in/*` |
| **E4** | Gazebo → `ros_gz_bridge` | Gazebo Transport (shared memory + UDP) | Sensor data (point clouds, depth images, RGB, camera_info, clock) |
| **E5** | `ros_gz_bridge` → ROS 2 DDS | DDS multicast on loopback | `/drone/rgbd/*`, `/drone/tof_*`, `/drone/stereo/*/camera_info`, `/clock` |
| **E6** | `cloud_merge` (planner_ws) → local planner | ROS 2 DDS | `/drone/tof_merged/points` (5-ToF fused cloud in base_link) |
| **E7** | `px4_odom_bridge` (planner_ws) → local planner | ROS 2 DDS | `/drone/odom` (ENU/FLU pose from PX4 EKF2, repackaged) |
| **E8** | `tf_static_broadcaster` → all consumers | `/tf_static` (latched DDS topic) | Static TF tree: base_link → tof_N_link, rgbd_cam_link, stereo_*, etc. |
| **E9** | `px4_odom_bridge` → all consumers | `/tf` (DDS topic) | Dynamic `odom → base_link` transform |
| **E10** | Module under test → `setpoint_publisher_node` | ROS 2 DDS | `/uav/cmd_vel` (TwistStamped, map frame) |
| **E11** | `setpoint_publisher_node` → MicroXRCEAgent | ROS 2 DDS → uXRCE-DDS (via E3 in reverse) | `/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_command` |
| **E12** | MicroXRCEAgent → PX4 SITL | uXRCE-DDS over UDP, port 8888 | PX4 native inbound messages → fed into EKF2 / mixer |
| **E13** | Operator → planner_ws | stdin / `ros2 topic pub` | `/uav/current_waypoint`, `/user/instruction`, or a `navigate_to_goal` action goal |
| **E14** | planner_ws → operator | terminal / RViz / rosbag | `/uav/vfh_status`, `/uav/mp_diag`, `/drone/odom`, `/uav/cmd_vel`, etc. |

### Notes on edges

- **E1 + E2 form the PX4 closed loop.** Gazebo's physics is what PX4 thinks the world looks like; PX4's motor commands are what Gazebo applies to the F450 rigid body.
- **E3 and E5 share one DDS bus.** All ROS 2 nodes — bridges, planner, module under test — discover each other on the same DDS multicast domain.
- **E6 (cloud_merge) is in-container, in-process-tree.** It sits between the raw Gazebo→ROS bridge and the planner, transforming each ToF cloud into base_link and concatenating them before the planner consumes them.
- **E11 closes the control loop.** The module's velocity command becomes a PX4 trajectory setpoint, which becomes attitude/thrust commands, which become motor speeds, which Gazebo applies via E1.
- **E13/E14 are out-of-band** with respect to the simulation but matter for the diagram because they are how the human enters and observes a run.

---

## 3. Startup Sequence

Components are launched in this order (each in its own terminal inside the container). This sequence is encoded in `~/irobot/px4_sim/run_sim.sh` for one-shot launches, and in `ablation/run_one.sh` for headless ablation runs.

| Stage | Command (abbreviated) | Brings up |
|---|---|---|
| T0 | `source ~/irobot/px4_sim/init_env.sh` | Sources ROS 2, the planner_ws install, and exports `PX4_DIR`, `DDS_AGENT`, `BRIDGE_YAML`, `PLANNER_WS`. |
| T1 | `PX4_GZ_WORLD=scenario_X make px4_sitl gz_f450` | PX4 SITL + Gazebo Harmonic with the F450 model in the chosen world. Components 3 + 4 + 5. |
| T2 | `MicroXRCEAgent udp4 -p 8888` | DDS bridge between PX4 and ROS 2. Component 6. |
| T3 | `ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$BRIDGE_YAML` | Gazebo↔ROS topic bridge for all sensors and clock. Component 7. |
| T4 | `ros2 launch uav_bringup full_stack.launch.py sensor_source:={d415,fusion} ...` | The InspectBot planner_ws stack: `px4_odom_bridge`, `tf_static_broadcaster`, `cloud_merge_node`, `setpoint_publisher_node`, and whichever modules the launch args enable (`with_vlm`, `with_brain`, `with_semantic_slam`, `with_emergency_landing`, …). Component 8. |

Each terminal can run inside the same container instance (with the X server forwarded for Gazebo/RViz) or in separate `apptainer exec` invocations sharing the same DDS bus.

---

## 4. Test Targets — Pluggability of the Four Modules

The simulation harness above is **module-agnostic**: the only thing that changes between testing different layers is which set of launch args (and which `.sif` container) is invoked in T4.

| Layer under test | What is added at T4 | What is exercised |
|---|---|---|
| **Layer 3 — Local Planner** (default) | `full_stack.launch.py sensor_source:=fusion` | `mp_node` / `mp_esdf_node` against live cloud + odom |
| **Layer 2 — Semantic Perception** | `full_stack.launch.py with_semantic_slam:=true` + start `isaac_vslam.sif` for cuVSLAM | cuVSLAM pose, NanoOWL detections (or mock), Semantic Graph Combiner, Redis |
| **Layer 1 — Cognitive (Brain)** | `full_stack.launch.py with_brain:=true with_vlm:=true` | LLM intent router, VLM Spatial Grounding, dispatch to `/spf/target_pose` |
| **Layer 4 — Failsafe** | `full_stack.launch.py with_emergency_landing:=true with_safety_mux:=true` | `emergency_landing_node_px4`, `cmd_vel_mux`, `vslam_watchdog` |

Each module subscribes to the same simulated sensor / state inputs (E5–E9) and produces its own outputs into the same DDS bus, so they can be tested independently or stacked. The PX4 + Gazebo half of the diagram is the constant; the right-hand half is what varies.

---

## 5. Quick Component Reference

For the schematic, the components fall into three host-level groups:

1. **Simulation engine** — Gazebo Harmonic + PX4 SITL (closed loop via E1). Renders the world, runs the firmware.
2. **Bridges** — MicroXRCEAgent (PX4↔ROS) and `ros_gz_bridge` (Gazebo↔ROS). Together they project the simulation onto the ROS 2 DDS bus.
3. **InspectBot stack (planner_ws)** — `cloud_merge`, `px4_odom_bridge`, `tf_static_broadcaster`, `setpoint_publisher_node`, plus the module-under-test from one of the four layers. The only group that varies between experiments.

All three groups run inside Apptainer containers on the NUS Vanda Cluster's A40 GPU node, sharing one DDS multicast domain on the cluster's loopback. The operator interacts via a forwarded desktop session (Gazebo GUI, RViz) and a terminal (`ros2 topic pub`, log output).
