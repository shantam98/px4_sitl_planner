# Plan — Motion Planner Benchmark: MP vs MIGHTY vs EGO-Planner-v2

## Context

The current local planner (`mp_node` in `uav_local_planner`) is a custom 20 Hz motion-primitive avoidance loop validated on the f450 + ToF array. The mid-term report commits to a "Dynamic Planner running at 50-60 Hz" for the inspection-by-drone use case — a target our current MP doesn't hit, and which would ideally be informed by SoTA literature rather than tuned in-house.

This plan benchmarks three planners against each other in simulation, with the goal of either:
- Validating that our `mp_node` is good enough for the use case, or
- Picking a SoTA successor to integrate into the production stack.

**No code in `mp_node.cpp` changes.** It is the baseline. MIGHTY (MIT-ACL, Hermite-spline trajectory opt, ROS 2 native) and EGO-Planner-v2 (ZJU FAST Lab, gradient-based B-spline traj opt, official ROS 2 port at `ego-planner-swarm/ros2_version`) are added as alternative *backends* behind a launch-arg switch.

User decisions baked into this plan:
- **Depth input**: D415-only (`/drone/rgbd/points` with voxel downsample to 0.05 m). Accepting the ~65° forward FOV limitation as a constant across all three planners.
- **Output contract**: `geometry_msgs/TwistStamped` on `/uav/cmd_vel` for all three. Existing `setpoint_publisher_node` (uav_control) is the unchanged controller. Adapters convert MIGHTY/EGO native outputs to TwistStamped.
- **Target**: Sim only for v1 (Gazebo `indoor_obstacle` on the cluster). Hardware validation deferred until a winner is chosen.

---

## Scope

| Planner | Source | ROS distro | Output contract used | Effort |
|---|---|---|---|---|
| **mp_node** (baseline) | `planner_ws/uav_local_planner/src/mp_node.cpp` (existing) | ROS 2 Humble | Native TwistStamped on `/uav/cmd_vel` | 0 (baseline) |
| **MIGHTY** | `mit-acl/mighty` | ROS 2 Humble | Hermite-spline setpoints → adapter → TwistStamped | ~3-4 days |
| **EGO-Planner-v2** | `ZJU-FAST-Lab/ego-planner-swarm` branch `ros2_version` | ROS 2 Humble | `quadrotor_msgs/PositionCommand` → adapter → TwistStamped | ~3-4 days |

Plus shared work:
- D415-only sim subscription wiring: ~0.5 day (just topic remaps + one YAML edit)
- Comparison harness (scenarios, automation, metrics, report): ~2-3 days
- Run + analyze: ~1-2 days

**Total: ~10-14 engineering days.** With one person, plan ~3 weeks calendar time.

### Why no external voxel filter

Originally I had a `pcl::VoxelGrid` step downsampling D415's 920K points before feeding the planners. Dropped after closer look:

- **EGO-Planner-v2** maintains its own internal `grid_map` module — subscribes to raw point cloud and builds an occupancy voxel grid at its own resolution. Trajectory optimizer queries the grid, not the cloud. EGO handles density internally.
- **MIGHTY** uses an internal ESDF representation similarly.
- **MP** is the only planner without internal voxelization. It iterates `O(points × primitives)` on the raw cloud. But MP already has a `history_subsample` parameter (default 2). For D415's higher point count, bump to 8 (keep 1-in-8) → ~115K usable points × 90 primitives × 20 Hz = ~200M ops/sec, well within Orin Nano budget.

Net: each planner manages cloud density its own way; no shared filter node needed. Cleaner comparison, less infrastructure.

---

## Work breakdown

### Phase A — D415-only sim wiring (0.5 day)

Goal: All three planners subscribe to D415's raw point cloud. Each planner manages density internally.

1. **All three planners subscribe to `/drone/rgbd/points`** (the existing topic from `realsense2_camera` / Gazebo bridge).
2. **MP node config bump**: edit `planner_ws/uav_local_planner/config/mp_params.yaml`:
   - `history_subsample: 2 → 8` (keeps ~115K of ~920K D415 points per cycle — comfortable for 20 Hz on Orin)
   - Optionally: also bump if running at higher rate
3. **5 ToFs stay running** in sim (no SDF edits). They just aren't consumed by the planners under test. Keeps the rest of the stack (visualization, octomap) unaffected during benchmark.

Critical files:
- `planner_ws/uav_local_planner/config/mp_params.yaml` — single line edit (`history_subsample`)
- `planner_ws/uav_local_planner/src/mp_node.cpp` — confirm cloud subscription topic is parameterized (line ~218 per exploration). If hardcoded to `/drone/tof_merged/points`, change to a launch arg with default `/drone/rgbd/points` for D415-only mode.
- No new nodes, no new packages.

### Phase B — MIGHTY integration (3-4 days)

Workspace layout:
```
~/irobot/mighty_ws/src/mighty               ← external clone (BSD-3)
~/irobot/mighty_ws/src/uav_mighty_adapter   ← NEW, our shim package
```

**B.1 — External dependencies (0.5 day)**
MIGHTY needs:
- DecompROS2 (clone, build): `git clone https://github.com/sikang/DecompROS2.git`
- L-BFGS solver: header-only, vendor into the workspace or apt
- Livox-SDK2 / livox_ros_driver2: **not needed for us** — we feed standard `sensor_msgs/PointCloud2` directly. Verify their planner subscribes to a generic topic name; if Livox-coupled, write an adapter cloud node.

**B.2 — Input adapter (0.5 day)**
- Remap MIGHTY's expected input topic to `/drone/depth/points_voxel` and `/drone/odom`.
- If MIGHTY assumes a body-fixed point cloud and we ship a `rgbd_cam_link`-frame cloud, add a TF transform in the adapter (probably already handled if their planner uses TF).

**B.3 — Output adapter (1 day)**
- New node `uav_mighty_adapter/src/mighty_to_cmdvel.cpp`.
- Subscribes to MIGHTY's setpoint output (likely a setpoint stream with position/velocity/acceleration).
- Extracts the velocity component, converts frame if needed (map ENU is what `/uav/cmd_vel` consumers expect).
- Publishes `geometry_msgs/TwistStamped` on `/uav/cmd_vel`.
- Throttle to 50 Hz if MIGHTY publishes faster.

**B.4 — Launch wiring (0.5 day)**
- New `uav_mighty_adapter/launch/mighty_planner.launch.py`.
- Includes MIGHTY's stock launch, our adapter, and a static TF if needed.
- Surfaced from `local_planner.launch.py` via `planner_backend:=mighty`.

**B.5 — Sim test + debug (1-1.5 days)**
- Run `mighty_planner.launch.py` with `setpoint_publisher_node` + Gazebo.
- Confirm waypoint missions complete.
- Tune any MIGHTY-specific parameters (look-ahead distance, smoothness weight).

### Phase C — EGO-Planner-v2 ROS 2 integration (3-4 days)

Workspace layout:
```
~/irobot/ego_planner_ws/src/ego-planner-swarm    ← clone, branch ros2_version (GPL-3)
~/irobot/ego_planner_ws/src/uav_ego_adapter      ← NEW, our shim package
```

GPL-3.0 license: be aware that anything *statically linked* to EGO becomes GPL. Our adapter and `setpoint_publisher_node` only consume EGO output via topics — that's not derivative work, MIT/proprietary code can interop. Just don't include EGO source in any proprietary deliverable.

**C.1 — External dependencies (0.5 day)**
- `sudo apt install ros-humble-rmw-cyclonedds-cpp libvtk7-dev` (per their README)
- PCL is already in our `uav_stack.sif`
- Clone branch: `git clone -b ros2_version https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git`
- `colcon build --packages-select ego_planner` (build only the planner, skip the simulator helpers)

**C.2 — Input adapter — odometry (~0.1 day)**
- EGO subscribes to `visual_slam/odom`. Simple topic remap to `/drone/odom`. No format conversion (both `nav_msgs/Odometry`).

**C.3 — Input adapter — point cloud (~0.5 day)**
- EGO subscribes to `pcl_render_node/cloud` (their sim's cloud output).
- Remap to `/drone/depth/points_voxel`.
- Their grid_map module expects clouds in `world` frame; if it doesn't auto-transform from `rgbd_cam_link`, write a small TF-aware republisher.

**C.4 — Output adapter (1 day)**
- New node `uav_ego_adapter/src/ego_to_cmdvel.cpp`.
- Subscribes to `drone_0_planning/pos_cmd` (`quadrotor_msgs/PositionCommand`).
- Extracts the `velocity` field, converts to `geometry_msgs/TwistStamped`, publishes on `/uav/cmd_vel`.
- Also subscribe to `drone_0_planning/bspline` if we want richer trajectory data for analysis.

**C.5 — Launch wiring (0.5 day)**
- New `uav_ego_adapter/launch/ego_planner.launch.py`.
- Includes `ego_planner`'s `single_run_in_sim.launch.py` (minus the simulator parts — we have Gazebo).
- Surfaced via `planner_backend:=ego`.

**C.6 — Sim test + debug (1-1.5 days)**
- Run, observe, tune EGO params (`max_vel`, `max_acc`, `safety_margin`).
- ROS 2 port has 40 commits — budget extra time for missing remaps or stale params.

### Phase D — Comparison harness (2-3 days)

New package: `planner_ws/uav_benchmark/`

```
uav_benchmark/
├── package.xml
├── setup.py                            # Python-only package (ament_python)
├── scenarios/
│   ├── straight_corridor.yaml
│   ├── two_pillar_slalom.yaml
│   ├── narrow_gap.yaml
│   ├── dead_end_recovery.yaml
│   └── long_traverse.yaml
├── uav_benchmark/
│   ├── run_comparison.py               # main automation
│   ├── analyze_bags.py                 # metrics extractor
│   └── plot_results.py                 # pandas + matplotlib
└── reports/                            # output: tables + plots
```

**D.1 — Scenario definitions (0.5 day)**
Each YAML defines: start pose, goal waypoint(s), obstacle layout (re-use `indoor_obstacle.sdf` or add variants), success criterion (reach goal within N seconds), failure mode (crash detected via `obstacle_distance < 0.1`).

**D.2 — Automation script (1 day)**
`run_comparison.py`:
- Args: `--planner {mp, mighty, ego}`, `--scenario <name>`, `--seed <N>`, `--repeats <N>`
- For each repeat:
  1. Start Gazebo with scenario
  2. Wait for `/fmu/out/vehicle_status_v1` to publish (PX4 ready)
  3. Start `local_planner.launch.py planner_backend:=<X>`
  4. Wait for HOVER state
  5. Send waypoint via `ros2 action send_goal /uav/navigate_to_goal`
  6. Start `ros2 bag record -o run_<planner>_<scenario>_<seed>` for all relevant topics
  7. Watch `/uav/mission_complete` or `/uav/vfh_status == ESTOP` or timeout
  8. Stop bag, tear down, repeat

**D.3 — Metrics extractor (1 day)**
`analyze_bags.py`:
- Iterates a directory of `.db3` rosbags
- For each: extracts time-to-goal, path length, average velocity, min obstacle distance, jerk RMS, success flag, mean per-cycle compute time (from `/uav/mp_diag` or planner-specific diagnostic topic)
- Aggregates into a pandas DataFrame, dumps CSV

**D.4 — Report generator (0.5 day)**
`plot_results.py`:
- Per-metric box plots per planner per scenario
- Summary table: planner × scenario success rate (%), mean ± stddev for each metric
- Output: PNG plots + Markdown report ready for the team

### Phase E — Run benchmark + analyze (1-2 days)

- 5 scenarios × 3 planners × 20 repeats = **300 sim runs**
- Each ~45 s mission + ~30 s overhead = ~3.75 hr compute (assuming sequential)
- Run on cluster (parallelize across nodes if possible to drop to ~1 hr)
- Analyze → final report → present to team

---

## Critical files

### New external workspaces
| Path | Origin |
|---|---|
| `~/irobot/mighty_ws/src/mighty` | clone `mit-acl/mighty` (BSD-3) |
| `~/irobot/ego_planner_ws/src/ego-planner-swarm` | clone `ZJU-FAST-Lab/ego-planner-swarm` `ros2_version` (GPL-3) |

### New packages in planner_ws
| Path | Purpose |
|---|---|
| `planner_ws/uav_mighty_adapter/` | MIGHTY setpoint → TwistStamped adapter + launch |
| `planner_ws/uav_ego_adapter/` | EGO PositionCommand → TwistStamped adapter + launch |
| `planner_ws/uav_benchmark/` | Comparison harness (Python, ament_python) |

### Modified existing files
| File | Change |
|---|---|
| `planner_ws/uav_local_planner/config/mp_params.yaml` | Bump `history_subsample: 2 → 8` (for D415's higher density) |
| `planner_ws/uav_local_planner/src/mp_node.cpp` | Parameterize cloud subscription topic (currently hardcoded to `/drone/tof_merged/points` per line ~218); accept `cloud_topic` launch arg |
| `planner_ws/uav_local_planner/launch/local_planner.launch.py` | Add `planner_backend:={mp,mighty,ego}` launch arg + `cloud_topic` arg (default `/drone/rgbd/points`); conditional include |

### Unchanged
- `planner_ws/uav_local_planner/src/mp_node.cpp` — baseline, no edits
- `planner_ws/uav_control/*` — controller stays the same for fair A/B
- `planner_ws/uav_planner_interface/*` — global planner unaffected
- PX4-Autopilot, sim world, model SDFs — no changes

---

## Reused existing utilities

- **`setpoint_publisher_node`** (`uav_control`) — the shared controller. All three planners' output funnels through this.
- **`waypoint_manager_node`** (`uav_local_planner`) — provides `/uav/current_waypoint`. Works for all three planners.
- **`tf_static_broadcaster`** (`uav_depth_fusion`) — TF setup unchanged.
- **`pcl::VoxelGrid`** (PCL library) — used in voxel filter step; standard apt-shipped library.
- **`rosbag2`** — for run recording in the comparison harness. Available in `uav_stack.sif`.

---

## Verification

End-to-end success criteria:

1. **Each planner launches and runs in isolation**
   - `ros2 launch uav_local_planner local_planner.launch.py planner_backend:=mp` → drone reaches a hand-sent waypoint
   - Same for `:=mighty` and `:=ego`

2. **Voxel filter sane**
   - `ros2 topic hz /drone/depth/points_voxel` → ~30 Hz
   - `ros2 topic echo /drone/depth/points_voxel | grep "data.length"` → reasonable point count (~25-50K)

3. **Comparison harness produces output**
   - `python3 uav_benchmark/uav_benchmark/run_comparison.py --planner mp --scenario straight_corridor --repeats 3`
   - Produces 3 bag files
   - `python3 uav_benchmark/uav_benchmark/analyze_bags.py bags/` → CSV with 3 rows
   - `python3 uav_benchmark/uav_benchmark/plot_results.py results.csv` → plots in `reports/`

4. **Final benchmark report exists**
   - 5 scenarios × 3 planners × ≥10 runs each → ≥150 successful bag files
   - Table with success rate, time-to-goal, path length, min clearance per planner per scenario
   - Clear winner OR equally clear "MP is good enough" verdict

---

## Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| MIGHTY's L-BFGS / DecompROS2 deps don't install in `uav_stack.sif` | Medium | Build MIGHTY in its own Singularity image; share data via DDS on the host |
| EGO's `ros2_version` branch has stale params / undeclared deps | Medium | Budget extra debug time in Phase C.6; community issues on GitHub may help |
| Adapter latency biases benchmark | Low | Measure adapter overhead separately, subtract from time-to-goal if significant |
| 20 runs per cell is statistically thin | Medium | If results are close, scale to 50; cluster can absorb the cost |
| MIGHTY's output format isn't documented | Medium | Reverse-engineer from their example launches and the published RA-L paper |
| EGO assumes a flat-ground sim and breaks on z-axis missions | Low | Indoor_obstacle world is mostly flat; if it matters, restrict test scenarios to constant altitude |

---

## Out of scope (deliberately)

- Algorithmic modifications to `mp_node.cpp` — only the cloud-topic param + `history_subsample` tuning. The algorithm itself is the baseline and must not change.
- D415 + ToF fusion — separately considered, deferred. v1 uses D415-only across all three planners.
- Mode-switch state machine (commute vs hold) — deferred until comparison picks a winner.
- Hardware flight tests — sim first, hardware after a winner is chosen.
- **Feeding nvblox ESDF directly to EGO/MIGHTY** — requires modifying their internal `grid_map`/ESDF subscribers (not just adapter work). Each planner uses its own internal voxelization for v1. Re-evaluate as a follow-up once nvblox is online in `planner_ws` (Phase 3 of master VSLAM plan) — at that point we can swap the *winner's* internal grid for nvblox ESDF queries (free CPU savings, same algorithm).
- External voxel pre-filter — dropped from the plan. Each planner handles density internally; MP's existing `history_subsample` parameter is sufficient.
- Fourth planner (Fast-Planner, etc.) — not adding more axes; 3-way is already enough.
