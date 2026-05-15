# See-Point-Fly Implementation Plan

> Implementation plan for the **Global Planner** — a VLM-driven navigation layer built on top of the See-Point-Fly paradigm (arXiv:2509.22653).
>
> Companion: `SYSTEM_ARCHITECTURE.md` (overall stack), `SEMANTIC_LAYER.md` (semantic map / keyframe DB).

---

## 1. Context

The mid-term report's "Goal Management & Decision Making" module is the responsibility of this stack. User issues a natural-language instruction; the system needs to translate that into safe, executed flight.

The See-Point-Fly (SPF) paper (Lin et al., arXiv:2509.22653) proposes treating action prediction as a **2D spatial grounding task**: the VLM annotates 2D waypoints on the input image and predicts a traveling distance, which are combined into 3D displacement vectors. This dramatically outperforms text-only VLM trajectory generation (+63% absolute on AVLN benchmark).

Our extension for the inspection-by-drone use case:
- VLM takes user instruction + RGB image → emits a **final target pose** (not a sequence of micro-waypoints)
- Target pose is sent into the existing `waypoint_manager_node`, which is upgraded with a **rotate-then-translate FSM**:
  1. **ROTATING phase** — waypoint_manager publishes target yaw via `/uav/current_waypoint`; `setpoint_publisher_node` emits trajectory_setpoint with that yaw and zero velocity
  2. **TRANSLATING phase** — waypoint_manager publishes target XY+Z; `mp_node` runs normally with obstacle avoidance
- Two execution modes:
  - **One-shot** (v1): VLM called once, target pose locked, drone executes
  - **Closed-loop** (v2): VLM re-called between phases with the new view; canonical SPF behavior

**Key architectural choice**: the rotate-then-translate FSM lives in `waypoint_manager_node`, not in the SPF Orchestrator. This means:
- SPF Orchestrator stays trivial — just calls `/uav/navigate_to_goal` with the full target pose, waits for the action result
- Any other navigation source (exploration, RViz goal, scripted mission) gets pose-aware execution for free
- Single ownership of `/fmu/in/trajectory_setpoint` stays at `setpoint_publisher_node` — no second publisher or mux complication
- VLM latency (1-2 s per call) is fully decoupled from controller rate — see §6 for analysis

**Hard prerequisite for SPF**: `setpoint_publisher_node`'s `FlightState` enum needs a `ROTATING` value added (~5-10 lines). This is required for yaw-only trajectory_setpoint emission in §4.5.3. It's small enough to fold into Phase SPF-D's checklist.

**Not blockers for SPF, but for safe co-running with emergency landing**: `uav_safety/cmd_vel_mux` and `uav_safety/vslam_watchdog`. These are system-level concerns (multiple cmd_vel publishers competing, cuVSLAM tracking loss as a flight risk) owned by `SYSTEM_ARCHITECTURE.md` §4.2. SPF v1 sim demo runs fine without them — they only matter when SPF + emergency landing + flight hardware all co-run.

---

## 2. Pipeline

```
  "Fly to the red box"
        │
        ▼
  User Instruction Interface  ──── /user/instruction ───►
                                                       │
                                                       ▼
                                       VLM Spatial Grounding
                                       (extension of vlm_api_inference_node)
                                                       │
                                                       ▼
                                       /spf/target_pose
                                       (geometry_msgs/PoseStamped, full pose with yaw)
                                                       │
                                                       ▼
                                       SPF Orchestrator (TRIVIAL)
                                         - calls /uav/navigate_to_goal action
                                           with full target pose
                                         - waits for action result
                                           (/uav/mission_complete topic is
                                            separately emitted for other
                                            consumers; orchestrator uses the
                                            action server's return value)
                                                       │
                                                       ▼
                                       Planner Server (action server)
                                         - emits /uav/global_path
                                           (one-pose path for SPF)
                                                       │
                                                       ▼
                                       Waypoint Manager  ◄── FSM lives HERE
                                       ┌────────────────────────────────┐
                                       │ phase = ROTATING:              │
                                       │   /uav/current_waypoint =      │
                                       │     (current XY, target yaw)   │
                                       │   /uav/mission_phase=ROTATING  │
                                       │     │                          │
                                       │     │   setpoint_publisher_node│
                                       │     │     emits NaN velocity   │
                                       │     │     + yaw=target         │
                                       │     │   mp_node hovers (0 cmd) │
                                       │     │                          │
                                       │     │ wait until yaw aligned   │
                                       │     ▼                          │
                                       │ phase = TRANSLATING:           │
                                       │   /uav/current_waypoint =      │
                                       │     (target XY+Z, yaw locked)  │
                                       │   /uav/mission_phase=TRANSLATING│
                                       │     │                          │
                                       │     │   mp_node generates      │
                                       │     │     cmd_vel normally     │
                                       │     │   setpoint_publisher     │
                                       │     │     forwards as velocity │
                                       │     │                          │
                                       │     │ wait until reached       │
                                       │     ▼                          │
                                       │ phase = COMPLETE               │
                                       └────────────────────────────────┘
                                                       │
                                                       ▼ /uav/cmd_vel
                                                       │
                                              cmd_vel MUX (mode arbiter)
                                                       │
                                                       ▼
                                              setpoint_publisher_node
                                                       │ /fmu/in/trajectory_setpoint
                                                       ▼
                                              MicroXRCEAgent ──► PX4 ──► motors
```

Note that:
- The rotation phase publishes trajectory_setpoint via `setpoint_publisher_node` (NOT bypassing it). Single owner of `/fmu/in/trajectory_setpoint`.
- The mux is unchanged from its previous design — it sits between (cmd_vel from mp_node or landing) and setpoint_publisher.
- mp_node is enhanced to read `/uav/mission_phase` and emit zero cmd_vel during ROTATING.

---

## 3. Existing assets at `/home/shantam/irobot/Cuvslam/`

Three Python files already implement parts of the perception side of SPF:

| File | What it does | Reuse in SPF |
|---|---|---|
| `vlm_api_inference_node.py` | OpenAI API call on RGB image; publishes `/vlm/detections` (Detection2DArray) for fixed prompts ("door", "window", "person", "chair") | **Extend** with a new `mode: spatial_grounding` that takes free-text instruction + image and outputs a target pose |
| `nanoowl_inference.py` | Local NVIDIA NanoOWL inference for open-vocab detection | Alternative to API variant; same extension path |
| `semantic_graph_combiner.py` | Fuses depth + detections + camera info → 3D-projected semantic graph published as JSON | Not directly used in SPF; orthogonal (semantic map for exploration, see SEMANTIC_LAYER.md) |
| `run_sitl_slam.sh`, `run_sitl_slam_API.sh` | Launch shell scripts | Replace with proper ROS 2 launch files in the new package |

These need to be **repackaged into `planner_ws/uav_vlm/`** as a proper ROS 2 package before they can integrate cleanly. Currently they reference non-canonical topic names (`/realsense/color/image_raw` instead of `/drone/rgbd/image`) — fixable in the packaging step via topic remaps.

---

## 4. New components to build

### 4.1 `uav_vlm` package (re-home `/Cuvslam` scripts)

```
planner_ws/uav_vlm/
├── package.xml
├── setup.py
├── uav_vlm/
│   ├── vlm_api_inference_node.py      # moved + topic remaps (existing filename)
│   ├── nanoowl_inference.py           # moved + topic remaps (existing filename)
│   └── semantic_graph_combiner.py     # moved + topic remaps
├── launch/
│   ├── vlm_api.launch.py              # picks API variant
│   ├── vlm_local.launch.py            # picks NanoOWL variant
│   └── semantic_graph.launch.py
└── config/
    └── vlm.yaml                        # prompts, API model, thresholds
```

NB: existing filenames preserved (no `_node` suffix on the NanoOWL one — matches `/Cuvslam/nanoowl_inference.py`).

Topic remaps applied at launch:
- `/realsense/color/image_raw` → `/drone/rgbd/image`
- `/realsense/depth/image_rect_raw` → `/drone/rgbd/depth`
- `/realsense/color/camera_info` → `/drone/rgbd/camera_info`

### 4.2 VLM Spatial Grounding extension

Extend `vlm_api_inference_node.py` (and/or its NanoOWL sibling) with a new mode triggered by an instruction on `/user/instruction`. Behavior:

1. Subscribe to `/user/instruction` (`std_msgs/String`)
2. On new instruction, capture latest `/drone/rgbd/image`
3. Build a prompt like:

```
"You are guiding a drone. The user says: '<instruction>'.
Given this image, output JSON:
  {
    'target_uv': [u, v],    # pixel coordinates to fly toward
    'distance_m': float,    # estimated travel distance in meters
    'yaw_change_deg': float # rotation needed before translation
  }
"
```

4. Call OpenAI API (or NanoOWL spatial grounding head)
5. Project `(u, v, distance_m)` to a 3D pose using depth from `/drone/rgbd/depth` at that pixel + camera intrinsics + current drone pose (TF lookup `map → rgbd_cam_link`)
6. Publish on `/spf/target_pose` (`geometry_msgs/PoseStamped`, frame `map`)

Concurrency: use a `is_processing` lock (already present in the existing node) so repeated instructions don't spam the API.

### 4.3 User Instruction Interface (~30 lines)

Minimum viable: a tiny Python node that listens on `/user/instruction` and logs received instructions. Operator publishes via:

```bash
ros2 topic pub --once /user/instruction std_msgs/String "data: 'fly to the red box'"
```

Optional v2: small Gradio/Streamlit web UI on the Orin that exposes a text box and publishes to the topic. Useful for demos.

### 4.4 SPF Orchestrator (= Global Planner) — TRIVIAL VERSION

New package `uav_global_planner/`. Python. Because the rotate-then-translate FSM lives in `waypoint_manager`, the Orchestrator just glues VLM output to the action server.

**Inputs:**
- `/spf/target_pose` (PoseStamped) — from VLM spatial grounding
- `/uav/emergency_trigger` — from watchdog/health (aborts SPF)

**Outputs:**
- Action client to `/uav/navigate_to_goal` (sends full target pose, including yaw)
- `/uav/global_planner_status` (String) — IDLE / EXECUTING / COMPLETE / ABORTED

**State machine** (much simpler than the old design):

```
[IDLE]
  └─ on /spf/target_pose received → [EXECUTING]

[EXECUTING]
  ├─ send NavigateToGoal action with full target_pose (rotation + translation
  │   handled internally by waypoint_manager)
  ├─ wait for action result
  ├─ on success → [COMPLETE]
  ├─ on action failure → [ABORTED]
  └─ on /uav/emergency_trigger → [ABORTED]

[COMPLETE]
  └─ publish status, return to [IDLE]

[ABORTED]
  └─ log reason, publish status, return to [IDLE]
```

~80 lines of Python total. The hard part was always the rotate-then-translate semantics; that now lives in waypoint_manager where it's reusable.

### 4.5 Waypoint Manager FSM upgrade

This is the biggest change. `waypoint_manager_node.cpp` gains pose-aware rotate-then-translate semantics. Affects three files in `planner_ws`.

#### 4.5.1 `waypoint_manager_node.cpp` changes

**Input topic** (unchanged): `/uav/global_path` (nav_msgs/Path). Each PoseStamped's `orientation` field is now read for target yaw.

**New output**: `/uav/mission_phase` (`std_msgs/String`) — values `"IDLE" | "ROTATING" | "TRANSLATING" | "COMPLETE"`. Published every cycle.

**Output topic change**: `/uav/current_waypoint` upgraded from `geometry_msgs/PointStamped` → `geometry_msgs/PoseStamped`. This is a **breaking change** for current subscribers (only `mp_node` consumes it today).

**FSM per waypoint** (event-driven; runs in the existing timer callback at the node's publish_rate_hz — currently 20 Hz):

```
state variable: spf_state ∈ {IDLE, ROTATING, TRANSLATING, COMPLETE}

on new waypoint pose received (from /uav/global_path):
    target_yaw_rad = quaternion_to_yaw(pose.orientation)
    yaw_error_rad = normalize_to_pi(target_yaw_rad - current_drone_yaw_rad)
    threshold_rad = deg2rad(yaw_alignment_threshold_deg)   # 15° default
    if |yaw_error_rad| > threshold_rad:
        spf_state = ROTATING
        cycles_in_tolerance = 0
    else:
        spf_state = TRANSLATING            # already aligned, skip rotation

on each timer tick (20 Hz):
    case spf_state == ROTATING:
        publish /uav/current_waypoint:
            position    = (current drone XY, current drone Z)    # hover in place
            orientation = target_yaw_quaternion
        publish /uav/mission_phase = "ROTATING"
        recompute yaw_error_rad
        if |yaw_error_rad| < threshold_rad:
            cycles_in_tolerance++
            if cycles_in_tolerance >= yaw_alignment_hold_cycles (default 10):
                spf_state = TRANSLATING
        else:
            cycles_in_tolerance = 0

    case spf_state == TRANSLATING:
        publish /uav/current_waypoint:
            position    = target XY+Z
            orientation = target_yaw_quaternion
        publish /uav/mission_phase = "TRANSLATING"
        if distance_to_target < acceptance_radius:
            spf_state = COMPLETE

    case spf_state == COMPLETE:
        publish /uav/mission_phase = "COMPLETE"
        publish /uav/mission_complete = true
        advance to next waypoint in path or return to IDLE
```

Units: all yaw math in radians inside the node. The user-facing parameter is in degrees for ergonomics; convert once at startup.

New parameters:
- `yaw_alignment_threshold_deg` (default 15.0) — yaw error tolerance to consider rotation done
- `yaw_alignment_hold_cycles` (default 10) — cycles to hold yaw within tolerance before transitioning (at 20 Hz this is 0.5 s)

Estimated LOC: ~150 new lines (FSM + parameter declarations + state-machine helpers).

#### 4.5.2 `mp_node.cpp` changes (~25 lines)

- Subscribe to `/uav/mission_phase` (latched)
- **Update existing `/uav/current_waypoint` subscriber type** from `PointStamped` → `PoseStamped`. Use `msg.pose.position` for the existing waypoint-following math; ignore `orientation` (only setpoint_publisher and waypoint_manager care about yaw).
- In the per-cycle `update()`, check phase:
  - If `phase == "ROTATING"`: emit cmd_vel with linear=0, angular=0 (hover); skip primitive evaluation
  - Otherwise: normal behavior
- Critical: keep publishing at the normal rate (don't go silent during ROTATING), otherwise `cmd_vel` timeout will fire on the consumer side

#### 4.5.3 `setpoint_publisher_node.cpp` changes (~30-40 lines)

**Prerequisite: add `ROTATING` to `FlightState` enum** — the existing FSM has STARTUP / TAKEOFF / HOVER / AUTONOMOUS. Add `ROTATING` between HOVER and AUTONOMOUS (or branched off AUTONOMOUS depending on style — see implementation). Folded into Phase SPF-D's checklist as the first item; without it, the rest of §4.5.3 has nothing to switch into.

- Subscribe to `/uav/mission_phase` and `/uav/current_waypoint` (PoseStamped)
- Add transition logic into and out of `ROTATING`:
  - From AUTONOMOUS to ROTATING when `/uav/mission_phase == "ROTATING"` arrives
  - From ROTATING to AUTONOMOUS when `/uav/mission_phase` transitions to TRANSLATING
- In the trajectory_setpoint generation loop:
  - If `FlightState == ROTATING`:
    - Hover mode — pick ONE of:
      - **Option A (simpler)**: `velocity = [0, 0, 0]`, `position = NaN`, `yaw = target_yaw`. PX4 uses velocity controller to hover; sensitive to wind drift but acceptable indoors.
      - **Option B (more robust)**: `position = (current pose latched at ROTATING entry)`, `velocity = NaN`, `yaw = target_yaw`. PX4 uses position controller to hold; better drift rejection but requires latching pose on state entry.
    - **Recommendation: Option A** for v1 simplicity; switch to Option B if drift during yaw rotation becomes a flight-test concern.
  - Otherwise (TRANSLATING / IDLE / etc.): existing cmd_vel-driven path unchanged. Since mp_node already emits zero cmd_vel during ROTATING (per §4.5.2), one could argue this change is unnecessary — see "Simplification consideration" below.

**Simplification consideration (BUT risky)**: if mp_node already produces zero velocity during ROTATING, the existing `cmd_vel` → trajectory_setpoint.velocity path naturally yields zero velocity. Tempting: drop the FlightState change and have `setpoint_publisher_node` always set `trajectory_setpoint.yaw = current_waypoint.orientation.yaw` whenever velocity is zero. ~10 lines, not 40.

**Why we DON'T do this**: mp_node also emits zero velocity during **stall detection** and **orbit recovery** — situations where the drone hovers because of avoidance issues, not because of a deliberate rotation phase. If setpoint_publisher conflates "zero velocity" with "rotate to target yaw", the drone would incorrectly start spinning toward a yaw target during stalls. That's a real safety issue.

**The explicit `FlightState::ROTATING` state avoids this** by gating yaw-rotation behavior on the mission_phase signal, not on the velocity-is-zero heuristic. Worth the extra ~30 LOC.

**Action**: inspect `setpoint_publisher_node.cpp` for current FlightState transitions before final LOC estimate. The "~40 lines" figure should be close.

This preserves single ownership of `/fmu/in/trajectory_setpoint` at `setpoint_publisher_node`. No mux additions, no second publisher.

---

## 5. TODO checklist

Recommended implementation order (**v1 total: ~6-7 days**; SPF-G v2 deferred adds another 3-5).

### Dependency graph

```
SPF-A, SPF-B, SPF-C, SPF-D — all independent; can run in parallel

SPF-E (orchestrator) — depends on SPF-D (waypoint_manager FSM emits the mission_phase
                                          the action server needs)
                       depends on SPF-B (orchestrator listens for /spf/target_pose)

SPF-F (validation)  — depends on all above
```

Notable independence: SPF-D (the local planner FSM upgrade) doesn't touch any VLM code, so it can proceed in parallel with SPF-A/B/C. Sequencing them is a workflow preference, not a hard dependency.

### Phase SPF-A — Repackaging (~1 day)

- [ ] Create `planner_ws/uav_vlm/` skeleton (package.xml, setup.py, launch/, config/)
- [ ] Move `vlm_api_inference_node.py`, `nanoowl_inference.py`, `semantic_graph_combiner.py`
- [ ] Apply topic remaps so all three consume `/drone/rgbd/*` (not `/realsense/*`)
- [ ] Write launch files (`vlm_api.launch.py`, `vlm_local.launch.py`, `semantic_graph.launch.py`)
- [ ] Add to colcon build, confirm `ros2 launch uav_vlm vlm_api.launch.py` works against the existing RealSense stack
- [ ] Smoke test: send a frame, confirm `/vlm/detections` publishes
- [ ] **Wire into top-level bringup**: add `with_vlm:=false` arg to `planner_ws/uav_bringup/launch/full_stack.launch.py`; when true, include `uav_vlm`'s `vlm_api.launch.py` (or `vlm_local.launch.py` based on a sub-arg)

### Phase SPF-B — VLM Spatial Grounding (~1.5 days)

- [ ] In `vlm_api_inference_node.py`, add `mode` parameter (`detection` | `spatial_grounding`)
- [ ] Implement spatial grounding prompt + response parser (JSON expected)
- [ ] Subscribe to `/user/instruction` — gate spatial grounding on new instructions
- [ ] Subscribe to `/drone/rgbd/depth` + `/drone/rgbd/camera_info` for 3D projection
- [ ] TF lookup `map → rgbd_cam_link` for current drone pose
- [ ] Publish `/spf/target_pose` (PoseStamped, frame `map`)
- [ ] Test with hand-crafted instructions; visualize target pose in RViz

### Phase SPF-C — User Instruction Interface (~0.5 day)

- [ ] Simple Python node — listens on `/user/instruction`, logs receipt
- [ ] Document the `ros2 topic pub` invocation for operator use
- [ ] (Optional, v2) Gradio web UI

### Phase SPF-D — Waypoint Manager FSM upgrade (~2 days)

This is the biggest single change but unlocks the cleanest architecture. Touches three files.

**Prerequisite (small, ~30 min):**
- [ ] **Add `ROTATING` to `FlightState` enum in `setpoint_publisher_node.cpp`** — single line in the enum definition; needed before the rest of this phase's setpoint_publisher work has a target state to transition into.

**waypoint_manager_node.cpp:**
- [ ] Upgrade `/uav/current_waypoint` topic type from `PointStamped` to `PoseStamped`
- [ ] Add `/uav/mission_phase` (`std_msgs/String`) publisher
- [ ] Implement per-waypoint FSM: ROTATING → TRANSLATING → COMPLETE (see §4.5.1 pseudocode)
- [ ] Add parameters `yaw_alignment_threshold_deg` (default 15.0) and `yaw_alignment_hold_cycles` (default 10)

**mp_node.cpp:**
- [ ] Update `/uav/current_waypoint` subscriber from `PointStamped` → `PoseStamped` (use `msg.pose.position`)
- [ ] Subscribe to `/uav/mission_phase`; hover (zero cmd_vel) during ROTATING

**setpoint_publisher_node.cpp:**
- [ ] Add transition rules into/out of ROTATING (AUTONOMOUS ⇄ ROTATING based on `/uav/mission_phase`)
- [ ] Subscribe to `/uav/current_waypoint` (PoseStamped); in ROTATING, emit trajectory_setpoint per §4.5.3 Option A (velocity=0, position=NaN, yaw=target)
- [ ] (Optional companion) Bump `cmd_timeout_s: 0.5 → 1.0`; add throttled log line when timeout fires

**Tests:**
- [ ] Unit test: publish a single-pose path with 90° yaw rotation, confirm drone rotates first (visible in `/uav/mission_phase = ROTATING`), then translates.
- [ ] Regression test: existing missions (no yaw change requested) skip directly to TRANSLATING — drone goes straight from current pose to target without unnecessary rotation.

### Phase SPF-E — SPF Orchestrator (~0.5 day, much simpler now)

- [ ] Create `planner_ws/uav_global_planner/` Python package
- [ ] Subscribe to `/spf/target_pose` and `/uav/emergency_trigger`
- [ ] Action client for `/uav/navigate_to_goal`
- [ ] Trivial state machine (IDLE → EXECUTING → COMPLETE/ABORTED)
- [ ] Publish status on `/uav/global_planner_status`
- [ ] Integration test: send `/user/instruction`, observe full pipeline (rotation + translation handled by waypoint_manager internally)
- [ ] **Wire into top-level bringup**: add `with_spf:=false` arg to `full_stack.launch.py`; when true, start `uav_global_planner`. Also enable `with_vlm:=true` as a default when `with_spf:=true` (since SPF orchestrator is useless without the VLM publishing `/spf/target_pose`).

### Phase SPF-F — Validation in sim (~1 day)

- [ ] Place colored boxes in `indoor_obstacle.sdf` (or new test world)
- [ ] Run full stack: `ros2 launch uav_bringup full_stack.launch.py with_vslam:=true with_vlm:=true with_spf:=true`
- [ ] Issue test instructions: "fly to the red box", "go to the doorway", "approach the chair"
- [ ] Record bags, verify successful navigation
- [ ] Document VLM accuracy / failure modes

### Phase SPF-G — Closed-loop SPF (v2, deferred ~3-5 days)

- [ ] After translation phase, re-capture image and re-prompt VLM
- [ ] If new target_pose is close to original, terminate (target reached)
- [ ] Otherwise, loop back to rotation phase
- [ ] Bound max iterations (e.g., 5) to prevent infinite loops

---

## 6. Latency analysis — VLM timing vs cmd_vel timeout

The VLM API call takes 1-2 s. `setpoint_publisher_node` has `cmd_timeout_s: 0.5` (stale `/uav/cmd_vel` → hover). This raised a concern: does VLM latency conflict with the timeout?

**Short answer: no, neither the old design nor the new design is harmed by the 0.5 s timeout. Both safely hover during the 1-2 s VLM thinking window.**

### What `cmd_timeout_s` actually does

`cmd_timeout_s` only affects `/uav/cmd_vel` staleness:
- When `/uav/cmd_vel` goes stale, `setpoint_publisher_node` falls back to zero velocity internally
- It **keeps publishing** `/fmu/in/trajectory_setpoint` at its own rate (well above PX4's 2 Hz minimum)
- PX4 stays in offboard mode; drone holds position
- The timeout NEVER causes a flight-mode loss or trajectory_setpoint gap

### What happens during the 1-2 s VLM call

```
t = 0       user issues instruction → VLM API call starts (async)
t = 0..2    VLM thinking
              meanwhile, NO ACTIVE MISSION YET:
                waypoint_manager has no active waypoint → no current_waypoint published
                mp_node has no current_waypoint → may stop publishing or emit zero cmd_vel
                  (either is fine here; we are pre-mission)
                cmd_vel goes stale → setpoint_publisher_node falls back to zero velocity
                setpoint_publisher_node keeps publishing trajectory_setpoint @ its rate
                → DRONE HOVERS SAFELY
t ≈ 2       VLM returns target_pose
              SPF Orchestrator calls /uav/navigate_to_goal action
              waypoint_manager enters ROTATING phase
              FROM HERE ON, mp_node MUST keep publishing (zero cmd_vel during ROTATING,
                normal cmd_vel during TRANSLATING) — silence becomes a real failure mode.
              normal execution resumes
```

So VLM latency just becomes 1-2 s of harmless hovering. No glitches, no failsafe trigger.

**The distinction matters**: pre-mission (no waypoint yet), mp_node going silent is fine — cmd_vel timeout safely defaults to hover. Post-waypoint inside an active mission, silence is bad — it masks node crashes. The concerns table below reflects the active-mission rule.

### Why the new design is more robust to latency (especially for closed-loop v2)

In the OLD design (Orchestrator owned the FSM), during the rotation phase:
- SPF Orchestrator was responsible for publishing the yaw command
- If Orchestrator got blocked on the next VLM call (closed-loop v2), yaw command might go stale
- Ambiguous behavior; needed careful design

In the NEW design (waypoint_manager owns the FSM):
- Once `/uav/navigate_to_goal` is invoked, execution is fully decoupled from SPF Orchestrator
- waypoint_manager publishes `/uav/current_waypoint` continuously at its own rate (20 Hz)
- setpoint_publisher_node reads latched targets — no dependency on VLM rate
- Drone executes rotation cleanly even if SPF Orchestrator is busy calling VLM for the *next* instruction

### Concrete concerns and mitigations

| Concern | Mitigation |
|---|---|
| First-instruction startup time: 1-2 s hover before flight starts on each new user command | Acceptable for inspection use case. UX note: show "thinking" status to operator. |
| VLM API timeout / failure | Add Orchestrator-side timeout (default 10 s); on timeout, publish ABORTED status; drone continues hover. |
| Closed-loop SPF (v2) jerkiness from repeated 1-2 s VLM waits | Don't pause flight during re-prompts — continue executing previous target; new target supersedes when it arrives. Alternative: switch to local NanoOWL (~200 ms latency) for closed-loop. |
| mp_node going silent during ROTATING phase | Critical: mp_node MUST keep publishing zero cmd_vel during ROTATING (every cycle), not go silent. Silence would let cmd_vel time out and mask mp_node crashes. Tested in Phase SPF-D regression test. |
| 0.5 s `cmd_timeout_s` being too tight for phase transitions | Optional: bump to 1.0 s. PX4's own failsafe still active. Easier debugging window between mission phases. |
| Silent timeout firing in logs | Add a log line "cmd_vel timeout — switching to hover" in `setpoint_publisher_node` so operators can diagnose. 5-line change. |

### Recommended companion edits to `setpoint_publisher_node`

Independently of SPF, two small quality-of-life changes:

1. **Bump `cmd_timeout_s` default from 0.5 to 1.0** — more forgiving during phase transitions, no safety impact
2. **Log when timeout fires** — currently silent; one `RCLCPP_WARN_THROTTLE` line makes debugging much easier

Both are 5-line changes; recommended even outside SPF work.

---

## 7. Open decisions

1. **Local VLM vs API**:
   - API (OpenAI GPT-4 Vision) — better accuracy, latency 1-3s, costs $$
   - Local NanoOWL on Orin — free, faster (~200ms), but limited to open-vocab detection (not full instruction understanding)
   - **Recommendation**: API for v1 testing/demos, local for production once accuracy is acceptable. Make it a launch arg.

2. **Distance estimation**:
   - VLM-predicted distance (per SPF paper)
   - Depth lookup at target pixel — more accurate when target is visible in depth
   - **Recommendation**: depth lookup first, fall back to VLM estimate if depth is invalid (NaN/out of range)

3. **What if target pixel has no valid depth?**:
   - Use VLM's predicted distance + ray-cast from drone in target_uv direction
   - Or: ask VLM for a closer intermediate waypoint
   - For v1: bail and report failure

4. **Yaw direction wrap-around**:
   - Always rotate the short way (-180° to +180° normalized)
   - Implement in `waypoint_manager_node`'s ROTATING phase, in the `normalize_to_pi(target - current)` step shown in §4.5.1 pseudocode

5. **Coordinate frame for target_pose**:
   - `map` frame (recommended) — stable across the mission. With cuVSLAM Phase 2 online, also drift-corrected via the dynamic `map → odom` TF.
   - `odom` frame — drifts; target pose may not match the original visual reference after long flight
   - **Recommendation**: `map` frame. Works today (static identity `map → odom`) — *improved* by cuVSLAM Phase 2, but not strictly required.

6. **What happens during emergency landing**:
   - SPF orchestrator must abort on `/uav/emergency_trigger`
   - The cmd_vel mux ensures landing wins regardless of SPF state
   - SPF orchestrator should NOT try to publish new yaw commands during landing — gate on emergency trigger

7. **Re-prompting cadence for closed-loop (v2)**:
   - Every N seconds during translation?
   - On each waypoint reached?
   - On significant image change (CLIP similarity drop)?
   - Defer; not a v1 problem.

---

## 8. Integration with rest of system

| Touches | How |
|---|---|
| `uav_safety/cmd_vel_mux` *(planned at system level; not SPF's dependency)* | When this mux eventually exists (built as part of emergency landing safety, see `SYSTEM_ARCHITECTURE.md` §4.2), SPF translation will flow through it like any other cmd_vel source. No SPF-specific behavior required. Standalone SPF demos (no landing in the loop) work without it. **Note**: the mux design must handle a type mismatch — `mp_node` publishes `geometry_msgs/TwistStamped` on `/uav/cmd_vel` while `emergency_landing_node_px4` publishes bare `geometry_msgs/Twist` on `/landing/cmd_vel`. The mux must either wrap the bare Twist into a TwistStamped on output, or the landing node must be patched. This is a SYSTEM_ARCHITECTURE.md concern. |
| `uav_safety/vslam_watchdog` *(planned at system level; not SPF's dependency)* | When built, will publish `/uav/emergency_trigger` on cuVSLAM tracking loss. SPF orchestrator subscribes for graceful abort. SPF still works without it — drone won't auto-abort on tracking loss, which is fine for sim and acceptable for early hardware tests. **Note**: the topic+field spec (`visual_slam/status` field `vo_state`) should be verified against the actual `isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus` definition before the watchdog is built. Spec assumption based on Isaac ROS release-3.2 exploration; cross-check at build time. |
| `uav_emergency_landing` *(separate repo at `/home/shantam/irobot/emergency_landing_sim`; not yet packaged into `planner_ws`)* | Landing's `/landing/enable: true` takes priority via cmd_vel mux (once mux exists). SPF must abort cleanly when this fires. Integration into `planner_ws` is tracked in `SYSTEM_ARCHITECTURE.md` §4.2. |
| **`uav_local_planner/waypoint_manager_node`** | **Major upgrade** — pose-aware FSM (ROTATING → TRANSLATING). Output topic upgraded `PointStamped` → `PoseStamped`. New `/uav/mission_phase` published. |
| **`uav_local_planner/mp_node`** | Subscribes to `/uav/mission_phase` and `/uav/current_waypoint` (now PoseStamped). Hovers (zero cmd_vel) during ROTATING. ~20 line addition. |
| **`uav_control/setpoint_publisher_node`** | Subscribes to `/uav/mission_phase` and `/uav/current_waypoint`. Emits trajectory_setpoint with yaw-only + NaN velocity during ROTATING. ~40 line addition. Single-point ownership of `/fmu/in/trajectory_setpoint` maintained. Optional: bump `cmd_timeout_s: 0.5 → 1.0` for more forgiving phase transitions. |
| `uav_vlm` (this package) | New. Hosts the spatial grounding extension and existing detection/semantic graph nodes from `/Cuvslam/`. |
| `uav_planner_interface` (action server) | Minor — must accept full PoseStamped (with yaw) as target. For SPF, can skip A* and emit a one-pose path directly. ~30 line change. |
| `uav_exploration` | Independent of SPF. Will benefit indirectly: exploration goals can now include yaw, and rotate-then-translate semantics get applied. No code change required for that benefit. |

---

## 9. Verification

Each phase has a discrete pass criterion:

1. **Phase SPF-A**: `ros2 launch uav_vlm vlm_api.launch.py` runs without errors against live RealSense. `/vlm/detections` publishes when feeding the API a familiar object.
2. **Phase SPF-B**: `ros2 topic pub /user/instruction std_msgs/String "data: 'fly to the door'"` produces a `/spf/target_pose` within a reasonable region.
3. **Phase SPF-C**: User instruction node logs received text correctly.
4. **Phase SPF-D**: Hand-published `/uav/global_path` with a single PoseStamped (90° yaw rotation from current) causes the drone to rotate first (visible in `/uav/mission_phase=ROTATING`), then translate. mp_node hovers cleanly during rotation phase. Regression: a same-yaw waypoint goes straight to TRANSLATING without rotating in place.
5. **Phase SPF-E**: Full SPF cycle — instruction → target pose → action call → rotation → translation → mission complete — observed in `/uav/global_planner_status` and `/uav/mission_phase` logs and RViz visualization.
6. **Phase SPF-F**: 10 instruction trials in `indoor_obstacle.sdf` with mixed objects (Gazebo sim on cluster or laptop). Target ≥ 70% success rate (where success = drone arrives within 0.5 m of intended object).
7. **Phase SPF-G**: Out of v1 scope. Verification criterion will be defined when closed-loop SPF is scoped for v2.

End-to-end demo target (v1, sim only): operator runs `ros2 topic pub /user/instruction std_msgs/String "data: 'fly to the red box and stop'"` against the Gazebo stack, drone executes successfully. Hardware deployment (SSH into Orin, real D415 + ToFs + Pixhawk) is a follow-on after sim validation passes.

---

## 10. Out of scope (v1)

- Closed-loop SPF (re-prompting VLM after each phase) — defer to v2
- Multi-step instructions ("first go to door, then to window") — defer; v1 is single-target
- VLM-driven exploration (separate from SPF; covered by `SEMANTIC_LAYER.md` and `uav_exploration`)
- VLM fine-tuning — using off-the-shelf models (GPT-4 Vision or NanoOWL)
- Gesture / voice input — text only in v1
- Mid-flight re-targeting from user — operator must wait for previous instruction to complete
- **`uav_safety` package** (`cmd_vel_mux`, `vslam_watchdog`, health adapters) — required for safe production deployment alongside emergency landing, but not SPF's responsibility. Tracked in `SYSTEM_ARCHITECTURE.md` §4.2.

---

## 11. References

- See, Point, Fly paper: https://arxiv.org/abs/2509.22653
- Existing VLM scripts: `/home/shantam/irobot/Cuvslam/`
- Topic contract: `SYSTEM_ARCHITECTURE.md`
- Semantic stack: `SEMANTIC_LAYER.md`
- Mux/safety: `SYSTEM_ARCHITECTURE.md` §2.5
