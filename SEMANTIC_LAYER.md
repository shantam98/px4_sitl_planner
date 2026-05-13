# Semantic Layer — Design Context

> Hand-off doc for implementing the VLM-driven semantic layer + a semantic-aware exploration algorithm.

## Where this fits

The UAV stack is being upgraded from purely-geometric SLAM + frontier exploration to a **vision-grounded** stack:

```
PX4 SITL + Gazebo Harmonic                Isaac ROS (isaac_vslam.sif)
  drone + stereo IR + RGBD          ──►   cuVSLAM (pose + map→odom TF)
                                          nvblox  (TSDF / ESDF / occupancy)
                                                       │
                                                       ▼
                  planner_ws (existing, drift-corrected automatically)
                  ├── motion primitives / VFH (local avoidance, ToF)
                  ├── A* global planner       (now consumes nvblox ESDF)
                  └── frontier exploration    ← YOU EXTEND THIS WITH SEMANTICS
```

The **Semantic Layer** is what makes exploration *informed* instead of *geometric only*. Today the frontier explorer picks the biggest/closest unknown frontier. With semantics it can prefer "rooms we haven't seen", avoid "kitchen-like areas", or be commanded "find the door".

---

## What's already built (current setup)

Phase 1 of the VSLAM integration is complete:

### Sim sensors (Gazebo Harmonic, f450 model)
- `/drone/stereo/left/image`, `/drone/stereo/right/image` — mono 1280×720 @ 30 Hz (D415-equivalent IR pair, 65° HFOV, 55 mm baseline)
- `/drone/stereo/{left,right}/camera_info` — static intrinsics published by `cloud_merge/stereo_camera_info_publisher` (workaround for a Gazebo Harmonic bug)
- `/drone/rgbd/image`, `/drone/rgbd/depth`, `/drone/rgbd/camera_info`, `/drone/rgbd/points` — RGBD front camera, 1280×720 @ 30 Hz
- `/drone/tof_merged/points` — 5× ToF fused to a single PointCloud2 (used by local avoidance, unchanged)
- `/drone/imu`, `/drone/odom`, TF tree, etc.

### Isaac ROS container (`isaac_vslam.sif`, Singularity)
- Base: `nvcr.io/nvidia/tritonserver:24.08-py3` (CUDA 12.6)
- ROS 2 Humble base
- `ros-humble-isaac-ros-visual-slam` + `-interfaces`
- `ros-humble-isaac-ros-nvblox` + `-msgs`
- `ros-humble-isaac-ros-image-proc`

Topics this container will publish once launched against the sim:
- `tf: map → odom` (drift-corrected via cuVSLAM loop closure)
- `visual_slam/tracking/odometry`, `visual_slam/tracking/slam_path`
- `visual_slam/vis/landmarks_cloud`, `visual_slam/vis/pose_graph_nodes`
- `nvblox_node/static_esdf_pointcloud`, `nvblox_node/static_map_slice`, `nvblox_node/static_occupancy`, `nvblox_node/mesh`

### Existing exploration node
- File: `planner_ws/uav_exploration/src/frontier_explorer_node.cpp`
- Logic: detects frontiers in the occupancy map, scores them on a weighted combination of size, distance, and unknown-volume gain.
- Currently consumes the OctoMap; **will be re-wired to consume nvblox** (`nvblox_node/static_occupancy` or the 2D slice) in a separate task. Assume that swap will happen.

---

## The Semantic Layer (Component I from the master plan)

cuVSLAM's map is opaque and geometric. We can't inject semantics into it. Two **parallel** structures, both fed by the same VLM/segmentation backbone:

### I.1 — Shadow keyframe graph (sparse, scene-level)

New package: `uav_semantic_keyframes` (Python, ROS 2).

Subscribes to:
- `/drone/stereo/left/image` (or `/drone/rgbd/image`) — the RGB to annotate
- `visual_slam/tracking/odometry` — current pose
- `visual_slam/vis/pose_graph_nodes` — cuVSLAM keyframe poses + IDs (we *mirror* them, we don't create them)

Trigger sources (configurable via `vslam.yaml`):
- `auto_mirror_cuvslam`: every time cuVSLAM emits a new pose-graph node, snapshot + VLM
- `manual`: `/uav/semantic_keyframe` service — operator or autonomy explicitly annotates
- `scene_change`: perceptual-hash / CLIP-embedding distance threshold
- `periodic`: every N seconds (safety net)

On trigger:
1. Snapshot latest RGB
2. Latch pose + cuVSLAM graph node ID
3. Run VLM (LLaVA-1.5-7B on the A40, or a lighter alternative — open choice) and extract: scene label, object list with bboxes, free-text description, CLIP embedding
4. Persist to SQLite at `/scratch/$USER/irobot/maps/<env>/semantic_keyframes.db`

Schema:
```sql
CREATE TABLE keyframes (
  id INTEGER PRIMARY KEY,
  cuvslam_node_id INTEGER,
  timestamp REAL,
  pose_x REAL, pose_y REAL, pose_z REAL,
  pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
  image_path TEXT,
  scene_label TEXT,
  objects_json TEXT,
  description TEXT,
  embedding BLOB
);
```

**Loop-closure resilience**: subscribe to `pose_graph_nodes`; when cuVSLAM updates a node pose, update the DB row keyed by `cuvslam_node_id`. Annotations follow cuVSLAM's optimization automatically.

Outputs:
- `/uav/semantic_keyframes/latest` — newest annotation (custom msg)
- `/uav/slam_path_semantic` — cuVSLAM's `slam_path` augmented with per-pose scene labels (for downstream planners)

### I.2 — Dense semantic channel in nvblox (per-voxel)

Run a segmentation model (CLIPSeg, Mask2Former, or similar) producing a per-pixel class image at depth rate:
- Publishes `/drone/semantic/image`
- Feed into `nvblox_node/color_image` slot — nvblox's TSDF integrator paints class labels onto voxels
- Saved inside the `.nvblx` file; no separate persistence

Use I.1 for high-level scene reasoning. Use I.2 for fast spatial queries (e.g., "is there a window-class voxel within 2 m?").

---

## Semantic-aware exploration (the part you build)

The frontier explorer becomes a **policy** that consumes:
- Geometric frontiers (existing): centroids, sizes, unknown-volume gains
- Semantic context (new):
  - `/uav/slam_path_semantic` — where we've been, what was there
  - `/uav/semantic_keyframes/latest` — most recent scene annotation
  - nvblox semantic channel — per-voxel labels in the map

### Suggested scoring extension

Existing scoring (today):
```
score = w_size · size + w_dist · 1/distance + w_gain · unknown_volume
```

Extended scoring:
```
score = (existing terms)
      + w_novelty   · (1 − max_similarity_to_visited_embeddings)
      + w_semantic  · scene_label_priority[target_label]
      + w_unfamiliar · (frontier_in_unlabeled_region ? 1 : 0)
```

Where:
- `max_similarity_to_visited_embeddings`: cosine-distance the frontier's *current view embedding* against the keyframe DB. Low similarity → unfamiliar → score up.
- `scene_label_priority`: operator-tunable map, e.g. `{"corridor": 1.5, "open_area": 1.0, "dead_end": 0.2}`.
- `w_*` weights all default to 0 → graceful fallback to geometric-only.

### Goal-conditioned exploration (stretch)

Operator publishes a text goal on `/uav/exploration_goal` (e.g. "find the red box"). Encode via CLIP text encoder once. Each frontier candidate gets a forward simulation: "if I go to this frontier and look forward, what would I see?" — use the keyframe DB / nvblox semantic channel to estimate. Score by `cosine(goal_embedding, predicted_view_embedding)`.

This is research-grade — start with the simpler scored-priority approach above.

---

## Where the code lives

| Component | Path | Type |
|---|---|---|
| Shadow keyframe graph + VLM | `planner_ws/uav_semantic_keyframes/` | NEW pkg, Python |
| Custom msg (SemanticPath / SemanticKeyframe) | `planner_ws/uav_planner_interface/msg/` | Add to existing pkg |
| Dense seg model node | `planner_ws/uav_semantic_keyframes/` (or split) | NEW node, Python |
| Semantic-aware scoring | `planner_ws/uav_exploration/src/frontier_explorer_node.cpp` | EDIT existing |
| Central config | `planner_ws/uav_bringup/config/vslam.yaml` (`semantic:` section) | YAML |

All flag-gated under `semantic.enable` in `vslam.yaml` so the stack runs with `enable: false` until ready.

---

## Open decisions for you to make

1. **VLM choice**: LLaVA-1.5-7B (~14 GB VRAM on A40, full multimodal), or lighter — CLIP + classifier head, BLIP-2, etc. Trade query latency vs richness.
2. **Segmentation model for nvblox channel**: CLIPSeg (open-vocab) vs Mask2Former (closed-set, faster). Open-vocab is more useful for goal-conditioned exploration.
3. **Query rate**: VLM at 1 Hz is comfortable; segmentation at 30 Hz to keep up with depth.
4. **Storage of embeddings**: SQLite BLOB is fine for thousands of keyframes; for retrieval at scale consider FAISS or `pgvector`. Probably overkill for v1.
5. **Trigger thresholds**: scene-change cosine distance (start ~0.3), periodic interval (start ~10 s).

---

## How to test

1. Confirm cuVSLAM + nvblox are running against the Gazebo sim (use `isaac_vslam.sif`, launch cuVSLAM in stereo `tracking_mode=0`, remap to `/drone/stereo/{left,right}/{image,camera_info}`).
2. Verify `visual_slam/vis/pose_graph_nodes` increments as the drone flies.
3. Implement the shadow keyframe node — confirm DB rows appear when nodes increment.
4. Implement semantic scoring on the existing waypoint missions — A/B by toggling `w_semantic` between 0 and a non-zero value, compare exploration trajectories.

---

## Key references

- Master plan with full architecture: see `/home/shantam/.claude/plans/deep-wishing-alpaca.md`
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/
- Isaac ROS Nvblox: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/
- Existing exploration node to extend: `planner_ws/uav_exploration/src/frontier_explorer_node.cpp` (scoring weights at lines 71–74 and 214–223)
