# Plan — Integrate VLM Exploration + Semantic VSLAM + Intent-Routing Brain into planner_ws

## Context

We have two external repos that each cover one half of an inspection-by-drone mission and a missing piece that ties them together:

- **`~/irobot/uav-vlm-exploration`** — "exploration" side. Takes a user text instruction, grounds it to a 3D pose via a VLM (OpenAI / Ollama), and feeds that pose into the existing planner action server. Already contains its own (forked) copies of `waypoint_manager`, `setpoint_publisher`, and `mp_node` with a `mission_phase` FSM (IDLE → ROTATING → TRANSLATING → COMPLETE) that lets the drone yaw-align *before* translating — needed because the VLM grounds a target relative to the current camera view.

- **`~/irobot/Semantic_Cuda_optimized_Visual_Slam`** — "exploitation" side. Wraps stock Isaac ROS cuVSLAM (no source changes) and adds 3 Python nodes: NanoOWL inference, semantic-graph combiner (synthesises landmarks from bbox + bearing + pose), Redis writer (persists landmark graph). Output is a Redis-resident map keyed by label.

- **Missing piece — global brain.** Today, the SPF orchestrator in `uav-vlm-exploration` blindly forwards any VLM-grounded target to the action server. It does *not* check whether the requested object is already a known Redis landmark (exploit), only knows the VLM-grounding (explore) path, and has no "just look at the scene" (analyse) path. We need a single decision point that classifies the user's intent into **analyse | exploit | explore** and dispatches accordingly.

Our `planner_ws` is the canonical tree (it has the recent Phase A/B work — `mp_esdf_node`, nvblox+ESDF voxel hash, `vslam.launch.py`). The other two repos are integrated *into* it, not the other way around.

User decisions baked in:
- **Merge strategy**: cherry-pick `uav-vlm-exploration`'s new packages + per-node patches into `planner_ws`. No tree swap.
- **NanoOWL**: deferred for v1 — integrate the semantic graph + Redis pipeline against a mock detection publisher. NanoOWL TRT engine + nanoowl Python deps come online in v2.
- **Intent routing**: single LLM call (Ollama / OpenAI) takes `{instruction, redis_label_summary}` and returns `{intent, target_label, target_pose?}`. No keyword rules, no separate embedding model.
- **User input**: stdin via `user_instruction_node` (reused as-is from `uav-vlm-exploration`).

---

## Architecture after integration

```
   stdin
     │
     ▼
user_instruction_node ──/user/instruction (std_msgs/String)──┐
                                                              ▼
                                              ┌─────────────────────────────┐
                                              │   uav_brain (NEW)            │
                                              │   single LLM call:           │
                                              │   inst + Redis labels →      │
                                              │   { analyse | exploit |      │
                                              │     explore, target }        │
                                              └───────┬───────┬───────────┬──┘
                                                      │       │           │
                              ┌───────────────────────┘       │           └────────────────────────────┐
                              ▼                               ▼                                        ▼
            vlm_spatial_grounding (explore)        Redis pose lookup (exploit)         analyse_helper (NEW, thin)
            existing — VLM call → /spf/target_pose    publishes /spf/target_pose       captures frame,
                                                       direct from semantic_graph      runs VLM describe,
                                                                                       publishes /uav/scene_report
                                          │
                                          └─→ spf_orchestrator
                                                  │
                                                  ▼
                                       /uav/navigate_to_goal (action) ── unchanged downstream:
                                                  │       planner_server → waypoint_manager (FSM)
                                                  ▼       → mp_node / mp_esdf_node → setpoint_publisher → PX4
                                              (existing)

   cuVSLAM (existing vslam.launch.py)  ──pose──┐
                                                ▼
            mock_detections_node (v1) ──┐    semantic_graph_combiner ──/semantic_graph──→ redis_writer
            nanoowl_inference_node (v2) ┘                                                     │
                                                                                              ▼
                                                                                            Redis
                                                                                              ▲
                                                                              (uav_brain polls)
```

The brain is the only new decision node; everything else is a port of existing code with topic remaps.

---

## Work breakdown

### Phase 1 — Cherry-pick exploration packages into planner_ws (~1 day)

Goal: bring the SPF FSM + VLM grounding online inside `planner_ws` without disturbing the Phase A/B nvblox/ESDF work.

1.1  **Copy new packages** verbatim from `uav-vlm-exploration/` into `planner_ws/`:
   - `uav_vlm/` (Python; contains `user_instruction_node.py`, `vlm_spatial_grounding.py`, `launch/vlm.launch.py`)
   - `uav_global_planner/` (Python; contains `spf_orchestrator.py`, `launch/spf_orchestrator.launch.py`)

1.2  **Apply per-node patches** (edits, not file replacements — `planner_ws` has Phase A/B changes that must be preserved):
   - **`uav_local_planner/src/waypoint_manager_node.cpp`** — add `SPFPhase` enum (IDLE/ROTATING/TRANSLATING/COMPLETE), `yaw_alignment_threshold_deg` + `yaw_alignment_hold_cycles` params, `/uav/mission_phase` (std_msgs/String) publisher, rotate-then-translate FSM in the update loop. Diff anchor: header comment, line ~1; new `enum class SPFPhase` near top; FSM body in `update()`.
   - **`uav_control/src/setpoint_publisher_node.cpp`** — add `ROTATING` to `FlightState`, `/uav/mission_phase` subscriber, ROTATING-state handler (velocity=0, latched hover z, `target_yaw_ned_` from VLM), bump `cmd_timeout_s` 0.5 → 1.0. **Preserve** the local-only `vehicle_local_position_v1` topic we already set for the laptop sim (the source tree uses `_v2` — keep `_v1`).
   - **`uav_local_planner/src/mp_node.cpp`** — add `/uav/mission_phase` subscriber; when phase == "ROTATING", publish zero `cmd_vel` to bypass motion primitives. **No change to `mp_esdf_node.cpp`** in v1 — only the cloud baseline path needs the gate; we can extend to ESDF later.
   - **`uav_planner_interface/src/planner_server_node.cpp`** — add `spf_direct_mode` parameter (default `false`); when `true`, emit a single-pose Path directly to the target instead of running A*. Required for the explore branch when there's no map yet.

1.3  **Update `uav_bringup/launch/full_stack.launch.py`** to add launch args + conditional inclusion: `with_vlm:=true|false`, `with_brain:=true|false`, `spf_direct_mode:=true|false`. Default all `false` so existing benchmarks still work.

1.4  **Topic adaptation** — `vlm_spatial_grounding.py` hardcodes `/drone/rgbd/image`, `/drone/rgbd/depth`, `/drone/odom`, `/fmu/out/vehicle_local_position_v1`. Confirm these match our `sensor_bridge.yaml` topic names; remap in the launch file if not.

1.5  **Smoke test** (sim) — start the stack with `with_vlm:=true`, type "go to the red box", confirm the drone yaws then translates. No brain in the loop yet — instruction goes straight to `vlm_spatial_grounding`.

**Critical files**:
- `planner_ws/uav_vlm/` (new — copied)
- `planner_ws/uav_global_planner/` (new — copied)
- `planner_ws/uav_local_planner/src/waypoint_manager_node.cpp` (patched)
- `planner_ws/uav_control/src/setpoint_publisher_node.cpp` (patched)
- `planner_ws/uav_local_planner/src/mp_node.cpp` (patched)
- `planner_ws/uav_planner_interface/src/planner_server_node.cpp` (patched)
- `planner_ws/uav_bringup/launch/full_stack.launch.py` (launch args)

### Phase 2 — Port semantic graph + Redis (NanoOWL deferred) (~1.5 days)

Goal: bring the landmark graph + Redis persistence online against mocked detections so the brain has something to query.

2.1  **New package `uav_semantic_slam/`** in `planner_ws/`. Copy three Python nodes verbatim from `Semantic_Cuda_optimized_Visual_Slam/vslam_semantic/`:
   - `semantic_graph_combiner_node.py` — pose + Detection2DArray → JSON landmark graph at 5 Hz.
   - `redis_writer_node.py` — JSON graph → Redis (`semantic_graph:*` keys, pubsub channel, ZSET index).
   - `_helpers.py` — math utilities.
   - Skip `nanoowl_inference_node.py` and `px4_imu_bridge_node.py` for v1.

2.2  **NEW `mock_detections_node.py`** — Python node in `uav_semantic_slam/`. Publishes `vision_msgs/Detection2DArray` on `/nanoowl/detections` at 5 Hz with a fixed list of test labels at hand-picked image coordinates ("red box" at (640, 360), "doorway" at (300, 360)). Triggers the combiner pipeline without any GPU model. Configurable via YAML so the same node can be repointed at sim-world ground-truth objects.

2.3  **Launch file `uav_semantic_slam/launch/semantic_slam.launch.py`** — starts the three real nodes + `mock_detections_node`, takes the same topic remaps that the Semantic_Cuda repo's `cuvslam_d415.launch.py` uses. Subscribes pose from our existing `vslam.launch.py` output (`visual_slam/tracking/vo_pose`).

2.4  **Redis** — install on the host (`apt install redis-server`; runs as a systemd service on `localhost:6379`). No new container.

2.5  **Wire into `full_stack.launch.py`** under `with_semantic_slam:=true`. Off by default until the brain in Phase 3 needs it.

2.6  **Verification** — `ros2 launch ...` brings up the stack; `redis-cli GET semantic_graph:latest` returns a JSON snapshot; `redis-cli ZRANGE semantic_graph:nodes_index 0 -1` lists mock landmarks.

**Critical files**:
- `planner_ws/uav_semantic_slam/package.xml` (new)
- `planner_ws/uav_semantic_slam/setup.py` (new, ament_python)
- `planner_ws/uav_semantic_slam/uav_semantic_slam/semantic_graph_combiner_node.py` (copied)
- `planner_ws/uav_semantic_slam/uav_semantic_slam/redis_writer_node.py` (copied)
- `planner_ws/uav_semantic_slam/uav_semantic_slam/_helpers.py` (copied)
- `planner_ws/uav_semantic_slam/uav_semantic_slam/mock_detections_node.py` (new)
- `planner_ws/uav_semantic_slam/launch/semantic_slam.launch.py` (new)
- `planner_ws/uav_bringup/launch/full_stack.launch.py` (extended)

### Phase 3 — Global brain: intent routing (~1 day)

Goal: a single Python node that owns the analyse / exploit / explore decision and dispatches to the right downstream.

3.1  **NEW `uav_brain/` package** (Python). Single node `brain_node.py`:
   - **Sub**: `/user/instruction` (std_msgs/String).
   - **Pub**: `/spf/target_pose` (geometry_msgs/PoseStamped — same contract that `vlm_spatial_grounding` uses today; downstream is unchanged), `/uav/scene_report` (std_msgs/String, for the analyse branch).
   - **Redis client**: connect to localhost:6379, read `semantic_graph:nodes_index` + per-node hashes on every instruction (no caching — graph mutates).
   - **LLM call** (reuse `vlm_spatial_grounding`'s existing Ollama/OpenAI client; factor out into a shared helper module). Prompt template:
     ```
     System: You are a UAV mission router. Classify the instruction.
     Known landmarks (label → position): {redis_summary}
     Instruction: {user_text}
     Return JSON: {"intent": "analyse"|"exploit"|"explore",
                   "target_label": str|null,
                   "rationale": str}
     ```
   - **Dispatch**:
     - `exploit` → look up `target_label` in Redis, build `PoseStamped` from `position_world`, publish on `/spf/target_pose`, ensure `spf_direct_mode:=true` on the planner_server.
     - `explore` → forward the instruction to `vlm_spatial_grounding` (republish on `/user/instruction_explore` — a new dedicated topic to break the loop) which then runs the existing pipeline.
     - `analyse` → call `analyse_helper.describe_scene(current_frame)` (one VLM call, full frame, instruction "describe what you see"), publish reply on `/uav/scene_report`.

3.2  **`vlm_spatial_grounding.py` retarget** — change its input subscription from `/user/instruction` to `/user/instruction_explore` so the brain can interpose. One-line change.

3.3  **Launch `uav_brain/launch/brain.launch.py`** — starts brain_node + the (renamed-input) `vlm_spatial_grounding`. Wired into `full_stack.launch.py` via `with_brain:=true`. When `with_brain:=false`, the old direct path (`user_instruction_node → vlm_spatial_grounding → spf_orchestrator`) still works — the brain is purely additive.

3.4  **Verification** — see §Verification below. Three cases: analyse, exploit-of-known-label, explore-of-unknown-label.

**Critical files**:
- `planner_ws/uav_brain/package.xml` (new)
- `planner_ws/uav_brain/setup.py` (new)
- `planner_ws/uav_brain/uav_brain/brain_node.py` (new)
- `planner_ws/uav_brain/uav_brain/llm_client.py` (factored out of `vlm_spatial_grounding.py` — shared Ollama/OpenAI helper)
- `planner_ws/uav_brain/uav_brain/analyse_helper.py` (new — single-shot VLM describe)
- `planner_ws/uav_brain/launch/brain.launch.py` (new)
- `planner_ws/uav_vlm/uav_vlm/vlm_spatial_grounding.py` (sub topic rename)
- `planner_ws/uav_bringup/launch/full_stack.launch.py` (extended)

### Phase 4 — NanoOWL real model (deferred, ~1 day when picked up)

Out of scope for v1 — flagged here so the structure assumes it. When picked up:
- Build TensorRT engine for sm_89 / x86 on the laptop (or rebuild for Orin on hardware).
- Drop in `nanoowl_inference_node.py` from the Semantic_Cuda repo.
- Disable `mock_detections_node`.
- No changes to brain, combiner, redis_writer or downstream — the detection topic contract is identical.

---

## Reused existing utilities

- **Action server `/uav/navigate_to_goal`** (`uav_planner_interface`) — unchanged contract. Brain → spf_orchestrator → action goal stays the existing path.
- **`waypoint_manager_node`** — same node, just gains the FSM patch.
- **`setpoint_publisher_node`** — same node, gains the ROTATING state.
- **`mp_node` / `mp_esdf_node`** — both already in tree from Phase A/B. mp_node gets the mission_phase gate; mp_esdf_node is unchanged in v1.
- **`vslam.launch.py`** (already in `planner_ws/uav_bringup/`) — stays as the cuVSLAM+nvblox owner. The Semantic_Cuda repo's launch file is *not* copied; we only port the three Python nodes downstream of cuVSLAM.
- **`llm_client`** — factor out of `vlm_spatial_grounding.py` once, reuse from brain + analyse_helper.

---

## Verification

End-to-end checks, in order. None of these requires Phase 4 (NanoOWL real model).

1. **Phase 1 smoke** — `ros2 launch uav_bringup full_stack.launch.py with_vlm:=true spf_direct_mode:=true`. In a separate shell type `go to the red box` into the user_instruction prompt; confirm `/uav/mission_phase` cycles `IDLE → ROTATING → TRANSLATING → COMPLETE`, drone yaws then translates, no oscillation between ROTATING and TRANSLATING (`yaw_alignment_hold_cycles` doing its job).

2. **Phase 2 Redis** — start `with_semantic_slam:=true with_vslam:=true`. Drone hovers; `redis-cli GET semantic_graph:latest` returns valid JSON with ≥1 mock landmark; `redis-cli ZRANGE semantic_graph:nodes_index 0 -1 WITHSCORES` shows monotonically increasing `last_seen_sec`.

3. **Phase 3 brain — analyse path** — `with_brain:=true`. Type `what do you see?`. Brain publishes a `/uav/scene_report` string within ~2 s; no `/spf/target_pose` is published; no action goal is sent.

4. **Phase 3 brain — exploit path** — pre-populate Redis with a known landmark `"workbench"`. Type `go to the workbench`. Brain emits a `/spf/target_pose` whose position matches the Redis `position_world` field; downstream action server receives the goal; drone executes.

5. **Phase 3 brain — explore path** — empty Redis. Type `find the toolbox`. Brain forwards on `/user/instruction_explore`; `vlm_spatial_grounding` runs; `/spf/target_pose` is published from the VLM grounding pipeline (not from Redis); drone executes.

6. **No regression in benchmark** — `ros2 launch uav_local_planner local_planner.launch.py planner_backend:=mp_esdf` still launches cleanly with `with_brain:=false with_vlm:=false`. The benchmark draft (PLANNER_BENCHMARK_DRAFT.md) is untouched.

---

## Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| `uav-vlm-exploration` patches collide with our Phase A/B edits to the same files | Medium | Apply each patch by hand using the diff output we already captured; preserve our `_v1` topic override and any other local divergence. Cherry-pick, not file overwrite. |
| Redis goes down mid-mission → brain crashes | Low | Wrap Redis client calls in `try/except`; on failure, route every instruction down the explore branch (graceful degradation). |
| LLM latency in brain (~1 s OpenAI, ~0.7 s Ollama) blocks user input | Low | Acceptable for v1 (user-initiated). Add a "thinking…" log line. Move to async if it ever batches multiple instructions. |
| Mocked detections diverge from real NanoOWL output shape (when Phase 4 lands) | Low | Copy the Detection2DArray fields produced by the real `nanoowl_inference_node.py` exactly — `class_id = label_string`, `score = confidence`. Verify with `ros2 topic echo /nanoowl/detections` once the real node lands. |
| `spf_direct_mode` skips collision checking → drone flies through obstacles when exploiting | Medium | The downstream `mp_node` / `mp_esdf_node` still runs avoidance on the cmd_vel side. spf_direct_mode only bypasses *global* A*; local primitive scoring is intact. Document this clearly in the launch arg description. |
| `mission_phase` subscriber added to `mp_node` only — `mp_esdf_node` doesn't gate on ROTATING | Low | v1 acceptable (most missions are exploit-style with known target). Track as a v1.1 cleanup: lift the gate into `MotionPrimitives` so both nodes share it. |

---

## Out of scope (deliberately)

- NanoOWL real inference (Phase 4 — separate plan).
- Embedding-similarity fallback for fuzzy label match (deferred; rely on LLM to handle "red box" ≈ "scarlet container").
- Multi-user / queued instructions — v1 is one-at-a-time.
- Hardware flight tests.
- Changing the planner benchmark scope. The MP vs MP+ESDF ablation in `PLANNER_BENCHMARK_DRAFT.md` is unaffected by this plan.
