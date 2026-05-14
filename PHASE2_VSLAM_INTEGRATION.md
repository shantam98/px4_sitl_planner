# Phase 2 — VSLAM Integration: TF Gate + Config + Launch Wiring

> Hand-off doc for implementing Components D + E of the VSLAM integration plan.
> Companion to `SEMANTIC_LAYER.md` (Component I) and the master plan at `~/.claude/plans/deep-wishing-alpaca.md`.

## Where this fits

Phase 1 is **done** — we have:
- Stereo IR camera pair added to the f450 SDF (mirrors RealSense D415)
- Bridge entries in `sensor_bridge.yaml` publishing `/drone/stereo/{left,right}/{image,camera_info}`
- `stereo_camera_info_publisher` node in `cloud_merge` publishing static camera intrinsics (workaround for Gazebo Harmonic not emitting camera_info)
- `isaac_vslam.sif` Singularity container with cuVSLAM + nvblox installed, verified to load on GPU

Phase 2 is what we do **next**: wire cuVSLAM + nvblox into the existing planner stack via a flag-gated, config-driven launch path. After Phase 2, running `ros2 launch uav_bringup full_stack.launch.py with_vslam:=true` will start cuVSLAM and nvblox alongside the existing nodes, with cuVSLAM owning the `map → odom` TF.

## Architecture recap

```
Gazebo Harmonic  ─ros_gz_bridge─►  /drone/stereo/{left,right}/image,camera_info
                                   /drone/rgbd/image, /drone/rgbd/depth
                                   /drone/imu
                                          │
                                          ▼
                            isaac_vslam.sif (Singularity)
                            ┌────────────────────────────┐
                            │ cuVSLAM (visual_slam_node) │
                            │   tracking_mode=0 stereo   │
                            │   publishes map→odom TF    │
                            └──────────────┬─────────────┘
                                           │
                            ┌──────────────▼─────────────┐
                            │ nvblox_node                │
                            │   consumes RGBD + cuVSLAM  │
                            │   pose → TSDF/ESDF/occupy  │
                            └────────────────────────────┘

(planner_ws nodes — motion primitives, A*, frontier, control — unchanged,
 but now operating in the drift-corrected map frame via cuVSLAM)
```

---

## Component D — Gate the static `map → odom` TF

**Why**: Today, `tf_static_broadcaster` publishes a static identity `map → odom`. When cuVSLAM starts, it ALSO publishes `map → odom` (drift-corrected, dynamic). Two publishers on the same TF edge causes TF_OLD_DATA warnings and unpredictable lookups. We need to silence the static one when cuVSLAM is running.

**Files**

| File | Change |
|---|---|
| `cloud_merge/src/tf_static_broadcaster.cpp` (lines 49-83) | Add ROS2 parameter `publish_map_to_odom` (default `true`). When `false`, skip the static map→odom broadcast. All other static TFs (base_link → tof_array_link, base_link → rgbd_cam_link, etc.) continue to publish. |
| `cloud_merge/launch/fusion.launch.py` | Pass `publish_map_to_odom` through as a launch arg with default `true`. |

**Pattern** (existing C++ style in this package):

```cpp
// declare parameter at the top of the node constructor
const bool publish_map_to_odom = declare_parameter("publish_map_to_odom", true);

// in the static TF broadcast block (around lines 49-83):
if (publish_map_to_odom) {
    geometry_msgs::msg::TransformStamped t_map_odom;
    // ... existing setup of t_map_odom ...
    static_transforms.push_back(t_map_odom);
}
// other transforms (tof_array_link, etc.) always publish
```

**Behavior**:
- `with_vslam:=false` (default) → `publish_map_to_odom: true` → behavior unchanged from today.
- `with_vslam:=true` → `publish_map_to_odom: false` → cuVSLAM owns the edge.

---

## Component E — Central config + Launch wiring

Three new files plus one edit. Goal: a single YAML knob controls whether VSLAM runs and how.

### E.1 — `uav_bringup/config/vslam.yaml` (NEW)

Master config for all VSLAM-related flags. Operators flip values here, restart the stack, no code edits.

```yaml
# uav_bringup/config/vslam.yaml
vslam:
  # Master switch — false reverts to the original (OctoMap + EKF2-only) stack
  enable: false

  # cuVSLAM
  cuvslam:
    tracking_mode: 0          # 0 = multicamera stereo (recommended for v1)
    num_cameras: 2
    enable_imu_fusion: false  # turn on later when IMU is calibrated
    rectified_images: true    # Gazebo cameras are pinhole-perfect

  # nvblox dense mapping
  nvblox:
    voxel_size: 0.05
    mapping_type: static_tsdf
    esdf_slice_min_height: 0.3
    esdf_slice_max_height: 1.8
    max_integration_distance_m: 5.0

  # PX4 vision feedback (Component H — closes loop to EKF2)
  # Ships disabled; flip true only after VSLAM tracking is validated.
  px4_feedback:
    enable: false
    publish_rate_hz: 30.0
    min_cov_position: 0.01
    min_cov_orientation: 0.001
    drop_when_not_tracking: true

  # Map persistence
  persistence:
    map_dir: /scratch/$USER/irobot/maps
    load_map_from: ""     # set to env_name (e.g. "lab_v1") to auto-localize on boot

  # Semantic layer (Component I) — wired but disabled for now
  semantic:
    enable: false
    mode: nvblox_channel  # "nvblox_channel" | "keyframe_db" | "off"
```

### E.2 — `uav_bringup/launch/vslam.launch.py` (NEW)

Wraps `singularity exec --nv isaac_vslam.sif …` for both cuVSLAM and nvblox, with topic remaps to our sim topics.

Key responsibilities:
- Read `vslam.yaml` via `yaml.safe_load`
- Start cuVSLAM via `ExecuteProcess` invoking `singularity exec` against `isaac_vslam.sif`
- Topic remaps:
  - `visual_slam/image_0` ← `/drone/stereo/left/image`
  - `visual_slam/image_1` ← `/drone/stereo/right/image`
  - `visual_slam/camera_info_0` ← `/drone/stereo/left/camera_info`
  - `visual_slam/camera_info_1` ← `/drone/stereo/right/camera_info`
  - (optional) `visual_slam/imu` ← `/drone/imu` (only if `enable_imu_fusion: true`)
- After cuVSLAM is healthy (~5 s), start nvblox in the same container
- nvblox topic remaps:
  - `nvblox_node/depth_image` ← `/drone/rgbd/depth`
  - `nvblox_node/color_image` ← `/drone/rgbd/image`
  - `nvblox_node/camera_info` ← `/drone/rgbd/camera_info`

Conditional behavior driven by `vslam.yaml`:
- `vslam.enable: false` → this launch returns empty (no nodes)
- `nvblox.enable` is implicit when `vslam.enable: true` (nvblox is required, not optional)
- `px4_feedback.enable: true` → also launch `vslam_to_px4_bridge` node (separate package — Component H, scheduled for later phase)

### E.3 — `uav_bringup/launch/full_stack.launch.py` (EDIT)

Add a `with_vslam` launch arg (default `false`). When true:
1. Pass `publish_map_to_odom:=false` to the fusion sub-launch (so the static TF is gated).
2. Include `vslam.launch.py` after T+5s (gives cameras + TF time to settle).
3. Drop `mapping.launch.py` (OctoMap) — nvblox replaces it.

Existing arg pattern in this file shows how to add the new one with `DeclareLaunchArgument` and `IfCondition`.

### E.4 — Hand-off contract

After Phase 2:
- `ros2 launch uav_bringup full_stack.launch.py` → original stack, unchanged.
- `ros2 launch uav_bringup full_stack.launch.py with_vslam:=true` → VSLAM stack:
  - Static `map → odom` is gated off
  - cuVSLAM runs in `isaac_vslam.sif`, owns the corrected `map → odom`
  - nvblox runs in `isaac_vslam.sif`, publishes ESDF + occupancy
  - Existing motion primitives, VFH, A*, setpoint publisher, frontier explorer continue running — they consume drift-corrected TF transparently
  - OctoMap server is NOT launched
- All toggles tunable via `uav_bringup/config/vslam.yaml`

---

## File map

| Path | Status | What changes |
|---|---|---|
| `cloud_merge/src/tf_static_broadcaster.cpp` | EDIT | Add `publish_map_to_odom` parameter; gate static map→odom broadcast |
| `cloud_merge/launch/fusion.launch.py` | EDIT | Pass `publish_map_to_odom` through |
| `uav_bringup/config/vslam.yaml` | NEW | Central VSLAM config |
| `uav_bringup/launch/vslam.launch.py` | NEW | Spawns cuVSLAM + nvblox in isaac_vslam.sif |
| `uav_bringup/launch/full_stack.launch.py` | EDIT | Add `with_vslam` arg; conditional include of `vslam.launch.py`; drop OctoMap when VSLAM on |
| `uav_bringup/CMakeLists.txt` | EDIT | Install config dir |
| `uav_bringup/package.xml` | (no change) | — |

No edits to: motion primitives, VFH, A*, setpoint publisher, frontier explorer, PX4 firmware, `Singularity.def` for `uav_stack.sif`. They benefit from cuVSLAM transparently via TF.

---

## Implementation order

Suggested sequence — each step independently testable:

1. **Component D**: edit `tf_static_broadcaster.cpp` + `fusion.launch.py`. Test with `publish_map_to_odom:=false` on the existing stack (no VSLAM yet) — confirm `map → odom` edge disappears from `ros2 run tf2_tools view_frames`.
2. **E.1**: write `vslam.yaml`. No code consumes it yet — just commit.
3. **E.2**: write `vslam.launch.py`. Test standalone:
   ```bash
   ros2 launch uav_bringup vslam.launch.py
   ```
   Confirm cuVSLAM + nvblox start in their container and subscribe to the stereo + RGBD topics.
4. **E.3**: edit `full_stack.launch.py`. Test with `with_vslam:=true`:
   ```bash
   ros2 launch uav_bringup full_stack.launch.py with_vslam:=true
   ```

After each step, regression-test that `with_vslam:=false` is identical to today.

---

## Verification

1. `ros2 run tf2_tools view_frames` with `with_vslam:=true`:
   - `map → odom` edge present and timestamp advancing (cuVSLAM owns it)
   - No "TF_OLD_DATA" warnings about duplicate publishers

2. cuVSLAM tracking healthy:
   ```bash
   ros2 topic echo visual_slam/status --once
   # vo_state should be 1 (TRACKING) once flying
   ```

3. nvblox publishing:
   ```bash
   ros2 topic hz nvblox_node/static_esdf_pointcloud   # ~5-10 Hz
   ros2 topic echo nvblox_node/static_map_slice --once | head
   ```

4. Regression with `with_vslam:=false`: standard waypoint mission completes as before; OctoMap stack runs unchanged.

---

## Open decisions before implementation

1. **How to invoke `isaac_vslam.sif` from inside `uav_stack.sif`**:
   Running `singularity exec` while already inside another Singularity container can be tricky. The most likely path: invoke from the host (i.e., don't call `ros2 launch full_stack.launch.py` from inside `uav_stack.sif`; instead have `run_server.sh` spawn `full_stack.launch.py` in T4 inside `uav_stack.sif`, and spawn `vslam.launch.py` in a SEPARATE terminal/process inside `isaac_vslam.sif`).
   Need to verify the exact pattern works on the cluster — may need to refactor `run_server.sh` to add a T6 for the VSLAM container.

2. **vslam.yaml location vs `$USER` expansion**:
   The `persistence.map_dir: /scratch/$USER/...` will need expansion at launch time. Python launch can do `os.path.expandvars`.

3. **Order of operations for `with_vslam:=true`**:
   cuVSLAM needs `tf: map → odom` to NOT exist before it starts (to avoid the duplicate publisher race). Currently `tf_static_broadcaster` starts at T+0. We launch VSLAM at T+5s. We must ensure `publish_map_to_odom:=false` propagates into `tf_static_broadcaster` BEFORE it starts at T+0.

---

## References

- Master plan: `~/.claude/plans/deep-wishing-alpaca.md`
- Phase 1 changes: f450 SDF, sensor_bridge.yaml, stereo_camera_info_publisher (Components B/C, already merged)
- Semantic layer hand-off: `planner_ws/SEMANTIC_LAYER.md`
- Container build: `px4_sim/IsaacVslam.def`
- Isaac ROS Visual SLAM docs: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/
- Isaac ROS Nvblox docs: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/
