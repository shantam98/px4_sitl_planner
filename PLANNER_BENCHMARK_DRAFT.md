# Plan — Motion Planner Benchmark: MP vs MP+ESDF (vs VFH3D+/ESDF, optional)

## Status — 2026-05-17

| Phase | State | Notes |
|---|---|---|
| A — nvblox ESDF queryable | **Done** | cuVSLAM (stereo, tracking_mode=0) + nvblox up via `vslam.launch.py`. Chosen topic: `/nvblox_node/static_esdf_pointcloud` (PointCloud2, intensity = signed distance, ~5–10 Hz, frame `map`). Local infra unblocked: Apptainer 1.19 + NVIDIA CDI working after CUDA driver reboot. |
| B — MP + ESDF integration | **Done (impl), verification pending** | Implemented as a **separate** `mp_esdf_node` (per decision: do not modify `mp_node`). Library additions (`updateEsdf`, `esdfLookup`, `updateWithEsdf`, voxel-hash member) are additive on `MotionPrimitives` — `mp_node`'s code path untouched. Backend switch is a launch arg, **not** a YAML flag. |
| B.5 verification | **Pending** | End-to-end smoke: `planner_backend:=mp_esdf` + `with_vslam:=true`, confirm `/uav/mp_diag` shows `esdf_mode=1.0` and `esdf_voxel_count > 0`, drone reaches a hand-sent waypoint. Blocked behind current Gazebo lag (see below). |
| C — VFH3D+ + ESDF | **Skipped** | Decision: MP-only ablation is sufficient to answer the headline question. VFH3D path stays in tree as legacy fallback (`use_mp:=false`), not part of the benchmark. |
| D — Benchmark harness | **Not started** | `uav_benchmark/` package, scenarios, automation, metrics, report. |
| E — Run + analyze | **Not started** | 2 configs × 5 scenarios × 20 seeds = 200 runs. |

**Implementation-vs-plan reconciliation (Phase B):** original draft proposed a `use_esdf` YAML flag toggling one binary. Final implementation is a second binary (`mp_esdf_node`) selected by `planner_backend:={mp, mp_esdf}` in `local_planner.launch.py`. Same `mp_params.yaml`, same scoring weights, same state machine — only the obstacle source differs (`/drone/rgbd/points` vs `/nvblox_node/static_esdf_pointcloud`). This change has no effect on the A/B comparison; it just trades a runtime flag for a build-time-selected executable, which keeps `mp_node.cpp` byte-for-byte stable across the benchmark.

**Current blocker for resuming:** Gazebo sim lag after reverting cameras to 1280×720 (cuVSLAM rejected the earlier 640×480 downsize because `camera_info` still advertised 1280×720). Needs root-causing — likely either drop only the stereo pair to 480p (keep RGBD at 720p for the planner) or fix the camera_info publication to match the image dims.

**Files actually shipped for Phase B:**
- `uav_local_planner/include/uav_local_planner/motion_primitives.hpp` — added `updateEsdf()`, `esdfLookup()`, `updateWithEsdf()`, `esdfVoxelCount()`; `VoxelKey` + `VoxelKeyHash` (boost-style mixer); `esdf_voxels_` unordered_map; `esdf_voxel_size_{0.05}`.
- `uav_local_planner/src/motion_primitives.cpp` — implementations; 20 samples/primitive in `updateWithEsdf`, body→world via `drone_pos + R(yaw)·p_body`, same hysteresis/pessimistic-temporal-filter/e-stop/scoring weights as cloud path.
- `uav_local_planner/src/mp_esdf_node.cpp` — new (~330 lines); subscribes to `/nvblox_node/static_esdf_pointcloud`; same state machine (stall, orbit, recovery, slew-limit) as `mp_node`; diag publishes `esdf_mode=1.0` + `esdf_voxel_count`.
- `uav_local_planner/CMakeLists.txt` — `add_executable(mp_esdf_node …)`, link to `motion_primitives`.
- `uav_local_planner/launch/local_planner.launch.py` — added `planner_backend:={mp, mp_esdf}` launch arg; conditional `Node(...)` per backend; `mp_node` carries remap `/drone/tof_merged/points` → `/drone/rgbd/points` (D415-only mode).
- `uav_local_planner/config/mp_params.yaml` — `history_subsample: 2 → 8` (D415 is denser than ToFs).

**Not in `mp_node.cpp`:** confirmed unchanged — the baseline is byte-stable.

---

## Context

The current local planner (`mp_node` in `uav_local_planner`) is a custom 20 Hz motion-primitive avoidance loop that operates on a short raw-cloud history (no persistent map). The mid-term report commits to a "Dynamic Planner running at 50–60 Hz" for the inspection-by-drone use case — a target that today's MP doesn't comfortably hit, in part because per-cycle obstacle checks iterate a ~15k-point kdtree.

This plan answers a focused question:

> **Does a persistent voxel map (nvblox ESDF) materially improve a primitive-scoring reactive planner — in avoidance quality and/or compute cost — versus the mapless raw-cloud-history approach we have today?**

If yes → MP migrates to ESDF as a production upgrade; the work is reusable. If no → MP's mapless architecture is validated, and we stop chasing the map for this class of planner.

This intentionally **skips third-party planners** (MIGHTY, EGO-Planner-v2, DWA-3D). The earlier MIGHTY/EGO route was abandoned because (a) dependency surface is large (Livox-SDK, custom Gazebo plugins, GPL caveats), and (b) the SoTA question is premature — first we should know whether the *map itself* matters for a reactive planner of our class. If the answer is "yes," then SoTA-trajectory-optimization comparators become a worthwhile follow-up; if "no," they aren't.

User decisions baked into this plan:
- **Sensor input**: D415 only (`/drone/rgbd/points`). 5 ToFs stay running in sim but unused by the planners under test.
- **Map source**: nvblox ESDF, owned by the `vslam.launch.py` stack (Phase 2 work). OctoMap is **out** — we don't want it.
- **Output contract**: `geometry_msgs/TwistStamped` on `/uav/cmd_vel`. Existing `setpoint_publisher_node` unchanged. Both configs share the same controller.
- **Target**: Sim only for v1 (Gazebo `indoor_obstacle` on the cluster). Hardware validation deferred until a winner is chosen.
- **Memory cap mechanism**: use nvblox's `clear_outside_radius_m` (drops voxel blocks beyond X m of the drone) rather than `max_integration_distance_m` (which only limits *new* observations, not existing voxels).
- **Scenario sizing constraint**: cul-de-sac depth in `dead_end_recovery.yaml` must be strictly less than `clear_outside_radius_m` so the back wall stays in ESDF memory while the drone retreats. `long_traverse.yaml` is forward-only — no memory recall required — so it isn't bounded by this constraint.

---

## Scope

| Config | Planner | Obstacle source | New work |
|---|---|---|---|
| **A — MP baseline (mapless)** | `mp_node` (unchanged) | Raw cloud history (today's behaviour) | None |
| **B — MP + ESDF** | `mp_node` (patched) | nvblox `static_esdf_pointcloud` | New ESDF subscriber; replace `pointToArcDist2D` inner loop with `esdf.lookup()` |
| **C — VFH3D+ + ESDF** (optional, gated) | `vfh3d_node` (patched) | nvblox `static_esdf_pointcloud` | Swap OctoMap subscription for ESDF; adapt histogram-build inner loop |

**Total minimum scope: A + B + harness.** Adding C costs ~+2 days and is contingent on A vs B being inconclusive (or the team wanting a stronger story).

### Why this design

- **Single-variable A/B** (with/without persistent map) — controls for everything else (planner algorithm, sensor input, controller, scoring weights). Results are directly attributable to the map.
- **Zero external dependencies** — both configs are already in our workspace; only thing new is the nvblox subscription, and nvblox is already wired in Phase 2.
- **Production-aligned** — whichever config wins is immediately shippable; no second integration sweep.

---

## Work breakdown

### Phase A — Confirm nvblox ESDF is queryable (0.5 day)

Goal: verify Phase 2 actually produces a usable ESDF stream from our sim cloud, and pick the exact topic the planners will subscribe to.

1. Run `run_sitl_slam.sh` Variant 2 (cuVSLAM + nvblox).
2. Confirm `nvblox_node/static_esdf_pointcloud` is at ≥5 Hz with reasonable point counts.
3. Decide between subscription forms (in priority order):
   - **`static_esdf_pointcloud`** (`PointCloud2` with distance in intensity field) — easiest, lowest coupling.
   - **`esdf_layer`** direct subscription via nvblox C++ API — fastest, but couples to nvblox internals.
   - **Service-based lookup** — out of scope for v1.
4. Document the chosen topic + frame + QoS in `vslam.yaml` config comments.

Output: one paragraph in this doc updating §"Decisions" with the actual topic name.

### Phase B — MP + ESDF integration (1.5–2 days)

Files modified:
- `uav_local_planner/include/uav_local_planner/motion_primitives.hpp`
- `uav_local_planner/src/motion_primitives.cpp`
- `uav_local_planner/src/mp_node.cpp`
- `uav_local_planner/config/mp_params.yaml`

**B.1 — Add ESDF subscriber (~30 min)**
In `mp_node.cpp`, add a subscription to `nvblox_node/static_esdf_pointcloud`. Maintain a member voxel hash (key: voxel index, value: signed distance). Update on each callback. Keep raw-cloud subscription too — controlled by config flag `use_esdf: {true,false}`.

**B.2 — ESDF voxel lookup helper (~1 hour)**
New method in `MotionPrimitives`: `double esdfLookup(const Eigen::Vector3f& p) const` — quantizes `p` to a voxel key, returns the stored distance (or `+inf` if no voxel). Voxel size matches nvblox config (default 0.05 m).

**B.3 — Replace inner loop (~3 hours)**
In `motion_primitives.cpp`'s scoring loop, today:
```cpp
for each primitive:
  for each point in point_buf_:
    d = pointToArcDist2D(point, arc); ...
```
Change to:
```cpp
for each primitive:
  for sample along arc (~20 samples evenly spaced):
    d = esdfLookup(sample);
    if d < collision_radius: collision = true; closest_d = min(closest_d, d);
```
- 20 samples per arc × 90 arcs = 1800 lookups per cycle (down from ~1.4M cloud-distance computations).
- Use config flag `use_esdf` to switch between today's path and the new one in the same binary — required for the A/B benchmark to share one build.

**B.4 — Gradient-aware cost (optional, +0.5 day)**
nvblox publishes per-voxel ESDF; gradient can be computed via finite differences across 6 neighbours. Add a penalty proportional to `−∇d · arc_heading` (primitives heading *toward* the nearest obstacle cost more than primitives heading *past* it). Skip for v1 if Phase B fits the day budget; add as Phase B.5 if there's headroom.

**B.5 — Diagnostic topic**
Extend `/uav/mp_diag` with ESDF-mode flag and a per-cycle ESDF voxel count. Lets the analysis script confirm config B actually used the ESDF when expected.

### Phase C — VFH3D+ + ESDF integration (1.5–2 days, OPTIONAL)

Files modified:
- `uav_local_planner/include/uav_local_planner/vfh3d.hpp`
- `uav_local_planner/src/vfh3d.cpp`
- `uav_local_planner/src/vfh3d_node.cpp`
- `uav_local_planner/CMakeLists.txt` (drop `octomap` link if no longer needed)
- `uav_local_planner/package.xml` (drop `octomap_msgs`, `octomap`)

**C.1 — Replace OctoMap subscription**
- `vfh3d_node.cpp:51` swaps `octomap_msgs::msg::Octomap` subscription on `/octomap_binary` for `sensor_msgs::msg::PointCloud2` on `nvblox_node/static_esdf_pointcloud`.
- Today the callback hydrates an `octomap::OcTree*`; new callback fills a `pcl::PointCloud<pcl::PointXYZI>` (intensity = signed distance).

**C.2 — Replace histogram build**
- `vfh3d.cpp:38` and `:116`: instead of `octree.search(query)` returning an `OcTreeNode*` with binary occupancy, the inner loop iterates ESDF points within the bbox and bins them into the polar histogram. Bins record the *minimum* distance in their cone — that's the natural fit for ESDF data.
- The "occupied" threshold becomes `distance < collision_radius`.

**C.3 — Strip OctoMap deps**
- Confirm no other planner_ws consumer needs OctoMap. (Already discussed in `SYSTEM_ARCHITECTURE.md` §4 — `uav_planner_interface` is the only other potential consumer; we'd handle that in Phase 3 of the master VSLAM plan.)
- Drop `octomap`/`octomap_msgs` from `CMakeLists.txt` and `package.xml` of `uav_local_planner`.

**C.4 — Gate**
Run **only after** Phase B has produced results. If A vs B differ by ≥10% on at least one headline metric (success rate or time-to-goal), the question is answered and C is optional. If they're within 5% of each other across scenarios, run C to determine whether the planning *algorithm* matters more than the map.

### Phase D — Benchmark harness (2–3 days)

New package: `planner_ws/uav_benchmark/`

```
uav_benchmark/
├── package.xml
├── setup.py                            # ament_python
├── scenarios/
│   ├── straight_corridor.yaml
│   ├── two_pillar_slalom.yaml
│   ├── narrow_gap.yaml
│   ├── dead_end_recovery.yaml
│   └── long_traverse.yaml
├── uav_benchmark/
│   ├── run_comparison.py
│   ├── analyze_bags.py
│   └── plot_results.py
└── reports/
```

**D.1 — Scenario definitions (0.5 day)**
Per YAML: start pose, goal waypoint(s), obstacle layout (re-use `indoor_obstacle.sdf` or variants), success criterion (reach goal within N seconds), failure criterion (`min_obstacle_distance < 0.1` during run = crash).

**D.2 — Automation script (1 day)**
`run_comparison.py`:
- Args: `--config {A,B,C} --scenario <name> --seed <N> --repeats <N>`
- Per repeat:
  1. Start Gazebo with scenario world.
  2. Wait for `/fmu/out/vehicle_status_v2` to publish (PX4 ready).
  3. Start `run_sitl_slam.sh` Variant N (nvblox required for B and C; not for A).
  4. Start `local_planner.launch.py use_esdf:=<bool> use_vfh:=<bool>` matching the config.
  5. Wait for HOVER.
  6. Send waypoint via `ros2 action send_goal /uav/navigate_to_goal`.
  7. Start `ros2 bag record -o run_<config>_<scenario>_<seed>` (relevant topics only).
  8. Watch `/uav/mission_complete` OR ESTOP OR timeout.
  9. Stop bag, tear down.

**D.3 — Metrics extractor (1 day)**
`analyze_bags.py`:
- Iterates `.db3` rosbags
- Per run extracts: time-to-goal, path length, mean velocity, min obstacle distance, jerk RMS, success flag, mean per-cycle compute time from `/uav/mp_diag` (or `vfh3d_diag`).
- Aggregates into pandas DataFrame, dumps CSV.

**D.4 — Report generator (0.5 day)**
`plot_results.py`:
- Per-metric box plots per config per scenario.
- Summary table: config × scenario success rate (%), mean ± stddev for each metric.
- Markdown report ready for team sharing.

### Phase E — Run + analyze (1 day)

- 2 configs × 5 scenarios × 20 repeats = **200 sim runs** (3 configs → 300 if C included).
- Each ~45 s mission + ~30 s overhead = ~2.5 hr sequential compute.
- Run on cluster (parallelize if possible) and analyse.

---

## Effort summary

| Phase | Min scope (A + B) | With C |
|---|---|---|
| A | 0.5 day | 0.5 day |
| B | 2 days | 2 days |
| C | — | 2 days |
| D | 2.5 days | 2.5 days |
| E | 1 day | 1.5 days |
| **Total** | **~6 days (~1.5 weeks calendar)** | **~8.5 days (~2 weeks calendar)** |

Vs original MIGHTY/EGO scope (10–14 days): **~50–60% reduction**, zero external deps, no license concerns.

---

## Critical files

| File | Change |
|---|---|
| `uav_local_planner/include/uav_local_planner/motion_primitives.hpp` | Add `esdfLookup()`, voxel hash member, `use_esdf` flag |
| `uav_local_planner/src/motion_primitives.cpp` | Replace cloud-iteration with ESDF lookup when `use_esdf=true` |
| `uav_local_planner/src/mp_node.cpp` | Subscribe to `nvblox_node/static_esdf_pointcloud`; pass to `MotionPrimitives` |
| `uav_local_planner/config/mp_params.yaml` | Add `use_esdf: false` (default) |
| `uav_local_planner/launch/local_planner.launch.py` | Add `use_esdf` launch arg |
| `uav_bringup/config/vslam.yaml` | Ensure `nvblox.enable: true` for variant 2 |
| `PLANNER_BENCHMARK_DRAFT.md` (this file) | Phase A output: confirm topic name |
| `uav_benchmark/` (new) | Harness, scenarios, analysis |

### Optional (Phase C)
| File | Change |
|---|---|
| `uav_local_planner/include/uav_local_planner/vfh3d.hpp` | Drop `octomap.h`; change query interface to ESDF point cloud |
| `uav_local_planner/src/vfh3d.cpp` | New histogram-build loop iterating ESDF points |
| `uav_local_planner/src/vfh3d_node.cpp` | Replace OctoMap subscription with ESDF subscription |
| `uav_local_planner/CMakeLists.txt` | Drop `octomap` link |
| `uav_local_planner/package.xml` | Drop `octomap`, `octomap_msgs` |

### Unchanged
- `mp_node.cpp` algorithm (only subscriber + flag added)
- `setpoint_publisher_node` — controller stays identical for fair A/B
- `waypoint_manager_node` — global planner unaffected
- PX4 firmware, sim worlds (other than scenario variants), SDFs

---

## Verification

1. **Phase A**: `ros2 topic hz /nvblox_node/static_esdf_pointcloud` ≥ 5 Hz; `ros2 topic echo … --once` shows non-zero `data.length`.
2. **Phase B**: `ros2 launch uav_local_planner local_planner.launch.py use_esdf:=true` runs; `/uav/mp_diag` flags `esdf_mode: true`; drone reaches a hand-sent waypoint.
3. **Phase D**: `python3 uav_benchmark/run_comparison.py --config B --scenario straight_corridor --repeats 3` produces 3 bag files; `analyze_bags.py bags/` produces a CSV; `plot_results.py` produces plots.
4. **Final report**: Markdown with success-rate table, time-to-goal box plot, compute-time table for both configs, plain-text verdict.

---

## Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| nvblox ESDF rate is too low for 20 Hz MP | Medium | Phase A measures it. If <5 Hz, increase nvblox `voxel_size` or reduce `max_integration_distance_m`. |
| ESDF voxel size (0.05 m) is too coarse for MP's `collision_radius=0.75 m` margin | Low | 0.05 m << 0.75 m. Plenty of resolution. |
| ESDF lookup with simple voxel-hash is itself slow | Low | 1800 lookups per cycle is trivial; std::unordered_map suffices. If slow, replace with flat array indexed by voxel key. |
| Phase B works in sim but ESDF is empty/stale on Orin in flight | Out of scope for v1 | Hardware test deferred. |
| MP+ESDF achieves identical results to MP-mapless → "nothing to show" | Low (this *is* a real result) | Frame the report around that finding: "raw-cloud history was sufficient." Worth knowing. |

---

## Out of scope (deliberately)

- Algorithmic changes to MP's primitive library, scoring weights, or recovery logic — the algorithm is constant across A and B.
- Third-party planner integration (MIGHTY, EGO-Planner-v2, DWA-3D). Defer until Phase B results justify it.
- D415 + ToF fusion — separately considered.
- Hardware flight tests — sim first.
- Closing the loop with cuVSLAM pose feedback to PX4 EKF2 — that's Phase H of the master VSLAM plan, not a planner-benchmark concern.
- Mission_phase guard for SPF compatibility — separate change, applied after benchmark winner is chosen.
