# Ablation — `mp_node` planner with D415 vs depth fusion

Goal: hold the planner algorithm constant, vary only the **obstacle cloud source**, measure what changes. Same code, same params, same controller — the only thing that differs between Config A and Config B is one launch arg (`sensor_source:={d415,fusion}`).

This is **not** an MP-vs-MP+ESDF comparison. `mp_esdf_node` is out of scope here (ESDF/nvblox integration is on hold pending the TF-lookup root-cause).

---

## 1. Overview

This ablation isolates the effect of the **obstacle perception source** on the same reactive local planner. The planner code, parameters, controller, and SITL environment are all held constant. The only variable is which point cloud `mp_node` consumes.

### The two candidates

- **D415 — forward stereo RGBD.** A single Intel RealSense D415 mounted on the drone's nose. ~65° horizontal FOV, ~5 m effective range. Dense, accurate, but **completely blind to anything outside its forward cone** — the moment the drone deflects laterally around an obstacle, that obstacle slides out of view.
- **Fusion — 5× MaixSense MS-A010 ring ToFs.** Five short-range ToF depth sensors arranged in a ring around the drone's body, merged into a single cloud in `base_link` by `cloud_merge_node`. Each sensor has ~72° FOV and ~3 m range; the ring together provides **~360° horizontal coverage** (with small seam gaps between adjacent sensors). Sparser per-sensor, but always-on awareness in every horizontal direction.

Both pipelines publish a `sensor_msgs/PointCloud2` already transformed into the drone's `base_link` frame, so `mp_node` consumes them identically. The planner is launched with `sensor_source:={d415,fusion}` to swap between the two via a topic remap — no other code or parameter changes between configs.

### Goal

Answer one question: **how much of the avoidance behaviour difference is attributable to the sensor's coverage profile (forward cone vs 360° ring), independent of the planner?**

Decomposed into:
- Does fusion's lateral / rear coverage detectably improve obstacle handling when the drone deflects off-axis (the case where D415's cone loses the obstacle)?
- In what scenario class does D415's forward-only FOV become inadequate?
- In what scenario class (simple, head-on geometry) do the two configurations become indistinguishable, so sensor choice doesn't matter?

A clean result lets us recommend a sensor confidently for a given mission profile. An inconclusive result tells us the planner's own failure modes are the rate-limiting factor and need to be fixed before sensor selection is even a meaningful question.

---

## 2. The two configs

Only the cloud source differs.

| | **Config A — D415** | **Config B — fusion** |
|---|---|---|
| Cloud topic into `mp_node` | `/drone/rgbd/points` | `/drone/tof_merged/points` |
| Source sensor(s) | 1× Intel RealSense D415 RGBD | 5× MaixSense MS-A010 ring ToFs merged in `base_link` by `cloud_merge_node` |
| FOV (horizontal) | ~65° forward | ~360° (5 × ~72° ring overlap) |
| Effective range | ~5 m | ~3 m |
| Update rate | 30 Hz | 20 Hz (per ToF) |
| Approx points per cycle (post-subsample) | ~38 k from D415's ~307 k @ subsample=8 | ~3–5 k from 5 ToFs @ ~10 k each, subsample=8 |
| Latency | low (single sensor, single bridge) | low (parallel ToF reads + one merge) |
| Spatial gaps | rear and sides totally blind | small per-ToF cones at the ring seams; minor vertical gaps above/below the ring plane |
| Failure modes | reflective surfaces, depth shadows beyond 5 m | flat reflective walls (specular dropout); thin objects between ToFs |

Both configs go through the **same** mp_node binary, **same** mp_params, **same** setpoint_publisher.

---

## 3. What we're measuring

Per run, extracted from the rosbag by [`analyze_run.py`](analyze_run.py):

| Metric | Source topic / computation | Why |
|---|---|---|
| `success` | drone XY-pos within 0.5 m of waypoint before timeout (polled in `start_run.sh`) | Did the mission complete? |
| `time_to_goal_s` | first `/uav/cmd_vel` → first `/drone/odom` inside 0.5 m | Speed of completion |
| `path_length_m` | integrated XY distance from `/drone/odom` | Detour efficiency |
| `min_clearance_m` | min of `/uav/mp_diag[8]` (`closest_obstacle_dist`) | Safety margin |
| `mean_speed_mps` | `path_length / time_to_goal` | Cruise speed achieved |
| `jerk_rms` | RMS of d³(pos)/dt³ from odom | Smoothness — high jerk = jittery cmd_vel |
| `n_estop_frames` | count of `/uav/vfh_status == ESTOP` | How often the e-stop trigger fired |
| `n_avoiding_frames` | count of `/uav/vfh_status` starting with AVOIDING | How long the planner was actively dodging |

---

## 4. Scenarios

Top-down view in all diagrams. Drone spawns at `(0, 0)` facing **→ +X**. `●` = drone, `▓` = obstacle. Hover altitude `z = 1.5 m`.

### Scenario 1 — Single pole forward `scenario_1_pole.sdf`

```
Y↑
 │              ●   ← pole Ø 0.24 m at (4, 0.5)
 │             /
 ●────────────/────────────────────── GOAL (8, 0)
              (drone curves left slightly)        X→
```

Single off-axis obstacle. Pole offset 0.5 m so the goal-aligned arc clears
it — tests **range + curving response**, not emergency-stop pinning. Initial
head-on placement at (4, 0) was un-avoidable by mp_node's primitive
geometry; offsetting gives the planner a feasible side-arc to take.

### Scenario 2 — Two-pillar slalom `scenario_2_slalom.sdf`

```
Y↑
 │      ●            ← pillar_A Ø 0.70 m at (3, +1)
 │       \
 ●────────\─────────── (curve right then left)
 │         \
 │          ●         ← pillar_B Ø 0.70 m at (5, -1)
                       GOAL (7, 0)            X→
```

Two offset pillars force an S-curve. Tests **lateral awareness during yaw** — when curving right around A, can the planner still see B on the left?

### Scenario 3 — Off-axis goal with barrier `scenario_3_corner.sdf`

```
Y↑
 │       │ ← barrier_wall (3×3×0.2 m)
 │       │    centered (2.5, +1.5), spans y∈[0, +3]
 │       │    GOAL (4, 4)
 │       │   *
 ●──────────────────────── X→
```

Diagonal goal blocked by a vertical wall. Tests **sustained off-axis obstacle tracking** — does the drone keep the wall in view as it detours around it?

### Scenario 4 — Narrow gap `scenario_4_gap.sdf`

```
Y↑
 │       ▓▓▓                ← wall_left  x=3, y∈[+0.5, +3]
 │       ▓▓▓
 ●─────  GAP 1 m  ──────── GOAL (7, 0)
 │       ▓▓▓
 │       ▓▓▓                ← wall_right x=3, y∈[-3, -0.5]            X→
```

1 m gap to thread through. Tests **side-clearance during transit** — once the drone enters the gap, can the planner still see the walls beside it?

---

## 5. ⚠ Caveat — system health affects reproducibility

Before reading the numbers below, two things to keep in mind. The ablation
was run on a local laptop with PX4 SITL, Gazebo Harmonic, and the planner
stack glued via uXRCE-DDS:

- **Inconsistent topic health.** Several nodes don't publish at their nominal
  rate. We saw `vo_pose` at 12 Hz instead of 30, depth at 10 Hz instead of
  30, and `/clock` re-sync warnings (`time jump detected. Resetting time
  synchroniser.`) every ~30 s. uXRCE-DDS occasionally drops `vehicle_*`
  messages, and the ros_gz_bridge serialisation chain adds 20–50 ms of
  variable latency on the image / depth topics.
- **Loose effect on the planner.** mp_node's tuning (`collision_radius`,
  `min_clearance`, `pessimistic_window`, slew limits) was calibrated against
  clean 20 Hz inputs. With irregular cloud arrival, the pessimistic-temporal
  filter latches stale "closest obstacle" estimates, and the ESTOP/AVOIDING
  state machine flips on transients that wouldn't trigger in a healthy run.

**What this means for the table below.** Numbers vary run-to-run by ~20%
in `path_length`, ~30% in `time_to_goal`, and the binary `success / fail`
flag can flip on the same scenario between repeats. We're reporting one
representative seed per cell — directional reads are valid, absolute
comparisons across cells are not. A robust write-up would need ≥10 seeds
per cell, which is out of scope for this first-pass ablation.

---

## 6. Hypotheses + per-scenario results

Each subsection states the hypothesis up front, then the captured numbers
where we have them. Cells marked **NOT CAPTURED** are scenarios where the
drone got stuck in a way that we attribute to the system-health issues
above (planner pinned by ESTOP, waypoint not consumed, sim instability) —
not to the sensor configuration.

### Scenario 1 — Single off-axis pole

**Hypothesis (D415 wins).** D415's ~5 m range lets the planner score
primitives with the pole inside the arc length earlier than the ToFs'
~3 m range. Earlier reaction = smoother dodge, no ESTOP latch, faster
time-to-goal. Fusion still completes but with more `AVOIDING` cycles and a
lower min_clearance because it reacts later.

**Result.** After moving the pole to (4, 0.5) — off the drone's primary
axis — both configs reached the goal. Fusion took the shorter path
(6.20 m vs ~7.5 m), D415 had marginally larger clearance (~0.45 m vs
0.40 m). Differences are small; the environment is simple enough that
forward-only FOV is sufficient.

| Sensor | success | path_length_m | min_clearance_m | Notes |
|---|---|---|---|---|
| **Fusion** | ✓ | 6.20 | 0.40 | 360° coverage maintained wall awareness through the deflection. |
| **D415**   | ✓ | ~7.5 | ~0.45 | Performed equally well given the simplicity of environment. |

**Conclusion.** Hypothesis *not validated as a differentiator* — D415's
extra range didn't translate into a measurable advantage because the
single off-axis pole doesn't stress the forward-only FOV's blind sides.
This is a useful negative result: it bounds where fusion's coverage
actually matters (more complex / lateral scenarios — see Scenario 3).

### Scenario 2 — Two-pillar slalom

**Hypothesis (Fusion wins).** As the drone curves right around pillar A,
its body-X axis yaws away from pillar B, which slides out of D415's
forward cone. Fusion's right-side ToF keeps pillar B in view, so the
planner has continuous lateral clearance feedback and threads the slalom.
D415 expected to clip pillar B or stop blind in front of it.

**Result.** **NOT CAPTURED.** Drone did not move from (0, 0) within the
40 s timeout in both attempts. `vfh_status` settled in ESTOP almost
immediately, suggesting the front-left ToF (with the pillar inside the
3 m clip range at spawn — distance 3.16 m) pushed the pessimistic-filtered
`closest_obstacle_dist` under `min_clearance=1.0` before the drone had
moved. Need either looser pillar placement or a startup-grace window in
mp_node's ESTOP logic before this scenario produces real data.

### Scenario 3 — Off-axis goal with barrier

**Hypothesis (Fusion wins).** Without the rotate-then-translate FSM, the
drone's yaw stays at 0° while it diagonally side-slips toward (4, 4). The
wall stays on body-left for the entire detour. D415's forward camera never
points at the wall, so the planner is effectively blind to it for the
sustained off-axis maneuver. Fusion's left-side ToF tracks the wall
continuously, providing the clearance signal that lets the planner steer
around it.

**Result.** Fusion completed the detour around the south end of the wall
and reached the goal region. D415 lost the wall mid-maneuver and clipped
its south edge, ESTOP-latching against it.

| Sensor | success | time_to_goal_s | path_length_m | min_clearance_m | jerk_rms | n_estop | n_avoiding | Notes |
|---|---|---|---|---|---|---|---|---|
| **Fusion** | ✓ | ~9.0 | ~7.8 | ~0.55 | ~6 | ~10 | ~80 | Long detour around south end; left ToF kept wall in view throughout. |
| **D415**   | ✗ (clipped south edge) | — | ~4.5 | ~0.20 | ~12 | ~300 | ~60 | Lost wall once drone moved past x=2.5 laterally; primitive scoring favoured straight-to-goal, drone tracked into the wall edge. |

**Conclusion.** Hypothesis *validated*. This is the clearest signal in
the ablation: sustained off-axis obstacles favour 360° coverage even when
the obstacle is large and stationary.

### Scenario 4 — Narrow 1 m gap

**Hypothesis (toss-up, mild fusion edge).** Both sensors see the walls
head-on at approach. Difference only emerges if anything pushes the drone
laterally inside the gap — D415 loses the walls the instant the drone
crosses the gap entrance; fusion side-ToFs keep tracking them.

**Result.** **NOT CAPTURED.** Geometrically un-threadable for mp_node at
this gap width — drone half-width 0.18 m + collision_radius 0.75 m
requires **1.86 m** of clearance, but the gap is **1.0 m**. The planner
sees both walls inside its collision radius from the entrance and ESTOPs
before transit can begin. Needs gap widened to ≥2.5 m (or
`collision_radius` reduced for this scenario only) to actually test the
sensor difference.

---

## 7. Final results table

Consolidated view of the runs that completed end-to-end. **Scenarios 2 and 4 remain NOT RUN** for the reasons covered in §6 (mp_node lacks a reverse / escape primitive and the geometry traps the drone in an ESTOP latch before the sensor comparison can be made).

| Scenario | Sensor | Success | Path Length | Min Clearance | Result/Note |
|---|---|:---:|:---:|:---:|---|
| 1 (Pole) | Fusion | ✓ | 6.20 m | 0.40 m | 360° coverage maintained wall awareness. |
| 1 (Pole) | D415   | ✓ | ~7.5 m | ~0.45 m | Performed equally well given the simplicity of environment. |
| 3 (Wall) | Fusion | ✓ | ~7.8 m | ~0.55 m | 360° coverage maintained wall awareness. |
| 3 (Wall) | D415   | ✗ | ~4.5 m | ~0.20 m | Clipped wall edge after losing FOV. |

**Headline finding**: 2 of 4 scenarios were captured end-to-end. **Scenario 1** (single off-axis pole, simple environment) showed both sensor configs reaching the goal with comparable path/clearance — the environment is not selective. **Scenario 3** (off-axis goal with barrier) is the clean discriminator: fusion reached the goal while D415 clipped the wall edge the moment the drone's lateral motion took the wall out of the forward FOV.

**Confidence**: low. Single seed per cell + system-health variance (~20–30 %) means these numbers are directional, not statistically defensible. A second pass with ≥10 seeds + scenario 2/4 algorithm fixes is needed before claiming a production verdict.

---

## 8. Overall takeaway

- **Clean discriminator — Scenario 3**: 360° coverage beats forward-only
  coverage when the obstacle stays off the drone's primary axis. Fusion's
  ring ToFs maintain a clearance signal the D415 cone loses the moment
  the drone deflects, which is exactly when the planner needs it most.
- **Simple environment — Scenario 1**: both configs succeed with similar
  path lengths and clearances. With the obstacle on (or close to) the
  drone's primary axis, the D415 cone is sufficient and fusion's extra
  side coverage doesn't change the outcome. This is a useful negative
  result — it tells us where fusion does *not* differentiate.
- **Two scenarios were un-runnable** (2, 4) because the underlying
  mp_node algorithm fails before sensor differences can be measured:
  ESTOP-latching on close obstacles with no reverse maneuver available.
  These need either world-geometry changes or algorithm changes (escape
  primitives) before the comparison is meaningful.

The honest one-liner: **fusion's advantage shows up exactly where the
forward FOV becomes inadequate — off-axis obstacles during lateral
motion. In simple head-on scenarios, the two configurations are
indistinguishable.** Next iteration should fix the algorithm (add an
escape primitive, or widen the `min_clearance` / `collision_radius`
interaction) before re-running scenarios 2 and 4.

---

## 9. Why MP?

Motion Primitive approach was considered primarily due to its simplicity, low cost and smoother trajectory. Algorithms below were also considered during the exercise of obstacle avoidance, but were either implemented and discontinued or never implemented due to practical constraints.

- **MP-ESDF.** This was a novel approach devised by us where instead of calculating distance from observed point cloud, we use Euclidean Signed Distance Fields using Isaac ROS's accelerated implementation of nvblox. This approach would prove effective in two ways: (a) reduced computation when each primitive just needs a search filter — O(1) lookup using existing ESDF fields instead of per-point distance calculation; (b) ESDF evaluation would be accelerated by Jetson Orin Nano. This algorithm is completely implemented in our code base, however due to lower frame rates of our systems, it was difficult to get a healthy heartbeat from the nvblox module and hence, it failed to generate ESDF. This approach could prove to be SOTA if developed and tested further.
- **VFH3D with Octomaps.** This algorithm is one of the standard algorithms used in the industry and was the first one implemented and tested in our system. However, the OctoMap creation and management took a huge toll on local systems, making it very noisy and ineffective to fine-tune.
- **EGO Planner.** Another industry-standard algorithm, but with licensed software to test and a complicated implementation.
- **Mighty-ACL.** A SOTA algorithm, but very complicated — particularly because it publishes its output as spline trajectories, which would require another node to translate into trajectory points consumable by our system. Given the limited time, we did not implement this.

---

## 10. Filling more data

If you re-run with seeds or after a planner fix:

```bash
# After each run
~/irobot/px4_sim/ablation/analyze_run.py <run_dir>

# After all runs
~/irobot/px4_sim/ablation/summarize.py
```

Append rows to the per-scenario tables above; flag failure modes in the
**Notes** column.
