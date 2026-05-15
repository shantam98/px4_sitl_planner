# emergency_landing_sim Repository Analysis (2026-05-15)

Located at `/home/shantam/irobot/emergency_landing_sim`. A Gazebo simulation + PX4 deployment package for F450 drone emergency landing using 6 depth sensors (5 side + 1 bottom).

## Nodes

### Simulation (registered in setup.py)

**`emergency_landing_node`** — Main FSM: MONITOR -> STABILIZE -> SCAN -> REPOSITION/DESCEND -> LANDED
- Sub: `/battery_percent` (Float32), `/signal_ok` (Bool), `/sensor_ok` (Bool), `/landing_complete` (Bool)
- Sub: `/sensor_side_1..5/depth/image_raw` (Image x5), `/sensor_bottom/depth/image_raw` (Image)
- Pub: `/cmd_vel` (Twist), `/emergency_landing_status` (String)

**`gazebo_cmdvel_bridge`** — Kinematic bridge converting `/cmd_vel` to Gazebo SetEntityState calls. Detects landing at floor_z.
- Sub: `/cmd_vel` (Twist), `/model_states` (ModelStates)
- Pub: `/landing_complete` (Bool)
- Srv Client: `/set_entity_state` (SetEntityState)

**`rotor_spin_node`** — Visual-only rotor animation tied to FSM state string.
- Sub: `/drone_state` (String)
- Pub: `/joint_states` (JointState)

### Deployment (in deployment_script/, NOT in setup.py)

**`emergency_landing_node_px4`** — Production FSM with FAILSAFE_DESCEND and retry limits.
- Sub: `/battery_percent` (Float32), `/signal_ok` (Bool), `/sensor_ok` (Bool), `/landing/offboard_ready` (Bool)
- Sub: `/sensor_side_1..5/depth/image_raw` (Image x5, configurable), `/sensor_bottom/depth/image_raw` (Image, configurable)
- Pub: `/landing/cmd_vel` (Twist), `/landing/enable` (Bool), `/emergency_landing_status` (String)

**`pixhawk_offboard_bridge`** — Translates landing velocity commands to PX4 offboard control messages.
- Sub: `/landing/cmd_vel` (Twist), `/landing/enable` (Bool), `/fmu/out/vehicle_status` (VehicleStatus)
- Pub: `/landing/offboard_ready` (Bool), `/fmu/in/offboard_control_mode` (OffboardControlMode), `/fmu/in/trajectory_setpoint` (TrajectorySetpoint), `/fmu/in/vehicle_command` (VehicleCommand)

## URDF Sensors (Gazebo plugins)

Defined in `urdf/pixhawk450_6depth.urdf.xacro`:
- 5 side depth cameras at 72 deg spacing, 20 Hz, 100x100 px, range 0.2-2.5m
- 1 bottom depth camera, range 0.01-3.0m
- 1 front RGB-D camera, 30 Hz, 640x480, range 0.1-10.0m (not used by any node)

## Integration with PX4-SITL/HITL

### What the landing nodes NEED (inputs your system must provide):

| Required Topic | Type | Source |
|---------------|------|--------|
| `/battery_percent` | Float32 | PX4 battery status or custom monitor |
| `/signal_ok` | Bool | RC/telemetry monitor |
| `/sensor_ok` | Bool | Sensor health monitor |
| `/sensor_bottom/depth/image_raw` | Image | Bottom depth camera |
| `/sensor_side_1..5/depth/image_raw` | Image (x5) | Side depth cameras |
| `/fmu/out/vehicle_status` | VehicleStatus | PX4 (via uXRCE-DDS) |

### What the landing nodes PROVIDE:

| Output Topic | Type | Consumer |
|-------------|------|----------|
| `/landing/cmd_vel` | Twist | pixhawk_offboard_bridge -> PX4 |
| `/landing/enable` | Bool | pixhawk_offboard_bridge (also mutex for planner) |
| `/emergency_landing_status` | String | Monitoring/logging |
| `/fmu/in/offboard_control_mode` | OffboardControlMode | PX4 |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | PX4 |
| `/fmu/in/vehicle_command` | VehicleCommand | PX4 |

### Integration notes

1. `emergency_landing_node_px4` is the node to integrate. Needs 6 depth images + 3 health signals.
2. `pixhawk_offboard_bridge` translates landing velocity to PX4 offboard control — keep as-is for SITL/HITL.
3. When emergency landing is active, the normal planner must yield control. Use `/landing/enable` as the mutex.
4. Deployment scripts are not in `setup.py` — either add them as entry_points or run via `python3`.
