# Planner Alternatives — Rejection Rationale

Companion to [`PLANNER_BENCHMARK_DRAFT.md`](PLANNER_BENCHMARK_DRAFT.md). That document describes the configurations we *will* benchmark (MP mapless vs MP+ESDF, with VFH3D+/ESDF as an optional Phase C). This document records the candidate planners we considered and *rejected*, and why — so future contributors don't have to re-derive these arguments.

The rejections below are framed entirely on technical, architectural, and integration-quality grounds. Calendar-time considerations are not used as rationale; we've tried to focus on properties of the algorithms and their distributions that would still hold even with unlimited engineering hours.

---

## 1. EGO-Planner-v2 (ZJU FAST Lab, 2022)

**Repository**: [`ZJU-FAST-Lab/ego-planner-swarm`](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), branch `ros2_version`.
**Algorithm class**: B-spline trajectory optimization with internal ESDF.
**License**: GPL-3.0.

### Why not

1. **License is copyleft.** Any code that compiles against EGO's headers or links to its libraries becomes a derivative work, obligated to release under GPL. For a project with potential commercial trajectory (inspection drone product), this is a permanent constraint on what we can ship.

2. **The architectural workaround exists but adds complexity that solves *only* a license problem.** A strict process+topic boundary keeps our code outside the GPL surface — but the message package (`quadrotor_msgs`) ships inside the EGO repository, and depending on its `.msg` files is in a legal grey zone. The clean fix is a relay node translating to a `uav_ego_msgs` package we own, which adds an entire piece of infrastructure that exists for legal hygiene rather than for the problem we're solving.

3. **Algorithmic load mismatch with deployment hardware.** EGO's B-spline optimization is designed for high-density obstacle environments and high-speed flight, validated on x86 NUCs or higher-spec embedded compute. On the Orin Nano (our deployment target), it's at the upper edge of what fits. We'd pay the engineering cost of integrating a sophisticated planner whose theoretical advantages are compute-bounded out of existence on our actual hardware.

4. **The ROS 2 port is stale.** The `ros2_version` branch is 40 commits behind upstream development, with no documented changelog of what was ported and what wasn't. Param-name renames, deprecated rclcpp APIs, and silent runtime bugs are inherited without an active upstream community to escalate to.

### What we'd be gaining

A research-quality trajectory optimizer with proven academic results, IF compute and license constraints didn't matter. Both matter for our use case.

---

## 2. MIGHTY (MIT-ACL, 2024)

**Repository**: [`mit-acl/mighty`](https://github.com/mit-acl/mighty).
**Algorithm class**: Hermite-spline trajectory optimization with convex safe-corridor decomposition.
**License**: BSD-3.

### Why not

1. **Dependency surface is enormous.** MIGHTY pulls in: `dynus_interfaces` (custom message package), `DecompROS2` (polyhedron decomposition library), `Livox-SDK2` and `livox_ros_driver2` (Livox-specific lidar driver — we have no Livox), a forked `gazebo_ros_pkgs`, the lab's `uav_simulator` (their own quadrotor dynamics), `realsense_gazebo_plugin`, `livox_laser_simulation_ros2`, and `acl-mapping`. We need essentially none of these.

2. **Tightly coupled to the lab's full simulation + autopilot stack.** Extracting "just the planner" requires patching their CMake to disable Livox-conditional builds, stripping out the simulation packages, vendoring `dynus_interfaces` separately, and verifying that none of the planner's internal calls reach into utility code that lives in the sim packages. The work is a non-reversible fork — every upstream improvement would have to be hand-merged across our pruning.

3. **Heavy and bespoke output format.** MIGHTY publishes `dynus_interfaces/DynTraj` — a Hermite-spline trajectory represented as polynomial coefficients. Our adapter needs to sample the spline at "now + lookahead," analytically differentiate the spline to recover velocity, convert to a `TwistStamped`, and throttle to the controller's rate. That's substantial adapter logic, and a non-trivial test surface for ensuring the velocity extraction is mathematically correct.

4. **Frame and TF coupling.** MIGHTY expects the sensor point cloud to be in world/map frame and does internal TF lookups to transform from sensor frames. Our `/drone/rgbd/points` is published in `rgbd_cam_link`. We'd need a TF-aware republisher on the input side, adding an additional infrastructure node solely to satisfy MIGHTY's frame assumptions.

5. **Algorithmic load mismatch with the use case.** Hermite-spline optimization with convex safe corridors via `DecompROS2` is engineered for high-density obstacle environments and aggressive trajectories. Indoor inspection at conservative speeds (our case) doesn't exercise the algorithm's strengths — we'd absorb its complexity without realizing its benefits.

### What we'd be gaining

The most recently published planner of the lot (2024) with a clean license. Strong fit for a different use case than ours.

---

## 3. DWA-3D (Bes, Dendarieta, Riazuelo, Montano — September 2024)

**Paper**: [arXiv:2409.05421](https://arxiv.org/abs/2409.05421).
**Algorithm class**: Velocity-space sampling (Dynamic Window Approach) extended to 3D.
**License**: N/A — paper only.

### Why not

1. **No public reference implementation.** The paper does not release source code. Integration requires re-implementing the algorithm from the paper text. The result of such an integration is "our implementation of DWA-3D," not "DWA-3D itself" — which contradicts the entire reason for bringing in an external comparator (to evaluate the published algorithm, not an interpretation of it).

2. **Map dependency conflicts with our direction.** DWA-3D consumes OctoMap as its persistent map source. We have decided OctoMap is out (CPU-bound, slow updates compared to nvblox). Adapting DWA-3D to nvblox ESDF means deviating from the paper's reference design — which compounds the "our interpretation" problem in point 1.

3. **Algorithmic class overlaps with MP.** DWA-3D samples velocity vectors `(vx, vy, vz, ω)` within a kinematically reachable dynamic window. MP samples curated motion-primitive arcs from a fixed library. These are isomorphic — every velocity sample produces a corresponding arc trajectory, every arc in the library corresponds to a velocity command. DWA-3D's contribution is essentially "DWA with denser velocity sampling and a 3D map," and MP is "DWA with a coarser arc library." The benchmark question that DWA-3D would help answer ("does a reactive sample-and-score planner improve when paired with a persistent map?") is **already covered by the MP+ESDF configuration** in our chosen scope.

4. **No categorically new planning paradigm.** Adding DWA-3D doesn't introduce a planning class we haven't already represented. It's a peer to MP, not a peer to MIGHTY or EGO. The information value of the additional comparator is bounded by how different its velocity-space sampling is from MP's primitive-library sampling — and in practice, both produce similar reachable sets at the granularities each samples at.

### What we'd be gaining

The most recent reactive planner from the literature, IF a faithful implementation existed. Without source, integration produces a measurement of our own engineering rather than of the published method.

---

## 4. VFH3D+ (existing implementation, demoted to optional)

**Status**: lives in [`uav_local_planner/src/vfh3d_node.cpp`](uav_local_planner/src/vfh3d_node.cpp), [`uav_local_planner/src/vfh3d.cpp`](uav_local_planner/src/vfh3d.cpp), [`uav_local_planner/include/uav_local_planner/vfh3d.hpp`](uav_local_planner/include/uav_local_planner/vfh3d.hpp).
**Algorithm class**: 3D polar-histogram reactive steering.
**License**: ours.

VFH3D+ is not categorically rejected — it remains available as an **optional Phase C** in the benchmark, gated on the outcome of Phase B (MP vs MP+ESDF). The arguments below are for why it's not the primary comparison axis.

### Why not the primary axis

1. **Hard-coupled to OctoMap, at the API level.** The implementation uses `octomap::OcTree*` C++ types directly throughout — see `octomap_msgs::msgToMap()` and the dynamic_cast in `vfh3d_node.cpp:56`, and the `const octomap::OcTree&` parameter signatures in `vfh3d.cpp:38` and `vfh3d.cpp:116`. The `OcTree::search()` semantics (returning a node pointer with binary occupancy and child accessors) have no direct analogue in nvblox's flat ESDF voxel grid. Swapping the map source is a real refactor of the inner loop, not just a topic remap.

2. **OctoMap is the practical bottleneck.** The algorithm itself is light, but it queries the map every cycle to build the polar histogram. OctoMap updates at 5-10 Hz on dense clouds (CPU-bound). The planner ends up bottlenecked at the map's rate, not its own algorithmic ceiling. Even after swapping to nvblox ESDF, the histogram-build step discretizes obstacle information into angular bins — discarding the signed-distance information that ESDF natively provides. The map upgrade unlocks update rate but not algorithmic richness.

3. **Structural algorithmic limitations that no map change can address:**
   - **No kinematic model.** VFH3D+ picks an angular bin, not a feasible trajectory. The output is a steering direction; the controller is responsible for translating it into a velocity. There is no notion of "this manoeuvre is feasible given my current velocity vector and acceleration limits."
   - **Vulnerable to local minima in concave geometries.** The histogram cannot distinguish "obstacle directly ahead" from "U-shaped dead end ahead." MP, by sampling curved arcs of finite length, at least propagates obstacle information forward along candidate trajectories.
   - **No projected-goal scoring.** VFH3D+ scores histogram bins by alignment to the *instantaneous* goal direction. MP scores motion primitives by both initial heading alignment *and* the projected end-position of the primitive relative to the goal — implicitly a one-step lookahead.

4. **Deliberately superseded by MP within this project.** MP was developed as the successor to VFH3D+ partly to address the limitations above. Promoting VFH3D+ back to the front of the comparison would frame the experiment as "did our migration to MP turn out to be a mistake?" — a valid question, but lower in priority than "does adding a persistent map help our current planner?"

### Why kept as a conditional Phase C

If Phase B (MP-mapless vs MP+ESDF) produces an inconclusive result — say the two configurations are within a few percent across all scenario metrics — then it becomes useful to know whether the lack of a clear winner is because *no reactive planner benefits from the map for our scenarios* (algorithm-agnostic) or specifically because *MP's scoring doesn't take advantage of the map's information* (algorithm-specific). VFH3D+ on ESDF answers this. It functions as a tie-breaker, not a primary candidate.

---

## What remains in the benchmark

| Configuration | Reason for inclusion |
|---|---|
| **MP, mapless (today)** | Established baseline. The configuration whose performance we want to characterize. |
| **MP + nvblox ESDF** | Same planner, with a persistent voxel map and signed-distance queries replacing the raw-cloud kdtree path. Single-variable A/B against the baseline. Whichever wins is immediately shippable as a production upgrade. |
| **VFH3D+ on ESDF** (optional Phase C) | Conditional tie-breaker if Phase B is inconclusive — to separate "map matters for any reactive planner" from "map matters specifically for MP." |

Both primary configurations live in this repository. Neither introduces a new external dependency, a license concern, an unmaintained third-party port, or a re-implementation-from-paper risk.

---

## Decision summary

The four rejected candidates fail on different axes:

| Candidate | Primary failure mode |
|---|---|
| EGO-Planner-v2 | License contamination + algorithmic load mismatch with Orin Nano |
| MIGHTY | Dependency hell + frame coupling + algorithmic load mismatch with use case |
| DWA-3D | No public source + redundant with MP+ESDF in algorithm class |
| VFH3D+ (as primary) | Algorithmic limitations precede the question of which map to use |

The benchmark therefore narrows to comparing an in-repo planner against itself with one carefully chosen variable changed. This is a less ambitious experiment than a 4-planner shootout, but it generates a result the team can act on either way: ship the winner.
