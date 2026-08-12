# Phase 6 Plan: Tightly-Coupled Fusion + ROS2

This is a design document, not implemented code. Phases 0-5 (see
[ROADMAP.md](ROADMAP.md)) are written but never build-verified; Phase 6 is
explicitly stretch scope and has not been started. This document exists so
that whoever picks up Phase 6 — me in a future session, or you — doesn't
have to re-derive the architecture from scratch, and can make informed
choices about what to build first.

Read [ROADMAP.md](ROADMAP.md) and [README.md](README.md) first if you
haven't — this document assumes familiarity with the existing
`frontend_vio`, `frontend_lidar`, and `backend` modules and their
conventions (SE3 pose semantics, the `PoseGraphEdge` measurement
convention, `ImuPreintegrator`'s current zero-bias formulation).

---

## 0. Two independent tracks

Phase 6 is really two unrelated projects that happen to share a ROADMAP
bullet:

- **Part A — Tightly-coupled LiDAR-VIO-IMU fusion**: a SLAM-math problem,
  entirely inside `slam_core`, no new dependencies.
- **Part B — ROS2 wrapping**: a systems/integration problem, almost
  entirely about message adapters, threading, and build tooling — it barely
  touches the math at all.

They can be done in either order, or in parallel by different people, or
one without the other. Part B works fine against the *current*
loosely-coupled backend; it does not require Part A. Section 4 gives a
recommended sequencing, but the two tracks are genuinely independent.

---

## 1. Current state (recap)

As of Phase 5:

- `frontend_vio::VioFrontend` produces a per-frame relative pose from
  stereo PnP, plus a preintegrated IMU delta (`ImuPreintegrator`) computed
  in parallel — **zero bias, no velocity state**, just `(delta_rotation,
  delta_velocity, delta_position, delta_time)` accumulated from raw
  gyro/accel with bias assumed exactly zero.
- `frontend_lidar::LidarFrontend` produces a per-frame relative pose from
  point-to-line/point-to-plane ICP.
- `backend::SlidingWindowOptimizer` fuses these **loosely**: each is an
  independent full pose estimate, turned into a `PoseGraphEdge` (a 6-DOF
  SE3 relative-pose constraint with a scalar-weighted information matrix)
  between two 6-DOF pose-only nodes. The IMU edge specifically only
  contributes its *rotation* (see `RotationOnlyInformation` in
  `optimizer.cpp`) — the preintegrated position is unused because, without
  a velocity state or gravity compensation, it isn't trustworthy.
- This is a real, working pose-graph fusion — but it is not what the SLAM
  literature calls "tightly coupled." In a tightly-coupled system, the IMU
  doesn't produce an independent pose estimate that gets weighted-averaged
  with vision/LiDAR after the fact; it contributes a genuine **factor**
  connecting joint state variables (pose *and* velocity *and* bias) inside
  the same optimization, and ideally the vision/LiDAR measurements
  themselves (raw pixel observations, raw point correspondences) are
  factors in that same graph too, rather than pre-solved poses.

Part A below describes what closing that gap actually requires.

---

## 2. Part A — Tightly-coupled LiDAR-VIO-IMU fusion

### 2.1 Two tiers, not one

"Tightly coupled" is not one feature — it's a spectrum, and the two ends
have very different costs:

- **Tier 1 (recommended target): full IMU factor.** Give each keyframe a
  15-DOF navigation state (pose, velocity, accel bias, gyro bias) instead
  of a bare 6-DOF pose, and let a proper bias-and-gravity-aware IMU factor
  connect consecutive states inside the optimizer — while *keeping* the
  existing VIO/LiDAR frontends exactly as they are (each still hands the
  backend a pre-solved relative pose, which becomes a `PoseGraphEdge`
  applied to the pose sub-block of the new 15-DOF state). This is the
  standard sense in which most "VIO" systems are tightly coupled with
  their IMU, and it's what lets you finally use the preintegrated
  *position*, not just rotation.
- **Tier 2 (larger, likely not worth it here): raw measurement factors.**
  Replace the VIO/LiDAR pose-edge inputs with *raw* measurements —
  2D pixel observations of 3D landmarks (reprojection factors) and raw
  point-to-plane/point-to-line residuals — computed jointly with the IMU
  factor in one optimization, so the backend (not each frontend
  independently) decides the best-fit pose, landmark positions, velocity,
  and bias all at once. This is what OKVIS/VINS-Fusion/LIO-SAM actually do
  end to end, and it's a substantially larger architectural change: the
  frontend/backend boundary this whole project is built around (frontends
  hand the backend a solved relative pose; the backend only ever sees
  6-DOF — now 15-DOF — poses and edges) goes away for the modules that
  make this change.

**Recommendation: build Tier 1 (sections 2.3-2.4 below) as the actual
Phase 6 deliverable. Treat Tier 2 (sections 2.5-2.6) as explicitly optional
and probably not worth doing** unless there's a specific reason to want a
research-grade system rather than a strong portfolio project — the
cost/benefit is bad: large rewrites of both frontends' public interfaces
and the backend's factor types, for an accuracy improvement that a
loosely-coupled-but-correct Tier-1 system usually doesn't need in practice.

### 2.2 Target state representation (Tier 1)

```cpp
// New: include/slam/backend/nav_state.hpp
struct NavState {
  Sophus::SE3d pose;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_accel{Eigen::Vector3d::Zero()};
};
```

15-DOF tangent, in this order: `(pose:6, velocity:3, bias_gyro:3,
bias_accel:3)`. Retraction: pose updates via `pose * Exp(dpose)` (Lie
group, same as today); velocity and both biases are plain vector spaces,
so their retraction is just `+=`.

### 2.3 Sub-phase 6A.1 — Full IMU factor

This is the mathematically hard part of Phase 6 — flag it as such going
in. The reference is Forster, Carlone, Dellaert, Scaramuzza, **"On-Manifold
Preintegration for Real-Time Visual-Inertial Odometry"** (IEEE T-RO, 2017).
Follow that paper's derivation closely when implementing this; don't
re-derive it from memory the way `ImuPreintegrator` reasonably could for
the much simpler zero-bias case.

**What changes vs. today's `ImuPreintegrator`:**

1. **Subtract a bias estimate before integrating.** Today's recursion
   integrates raw `gyro`/`accel` directly. The new version integrates
   `(gyro - bias_gyro_estimate)` and `(accel - bias_accel_estimate)`,
   where the bias estimate is the linearization point (the bias value at
   the *start* of the preintegration window, from the last optimization
   pass).
2. **Propagate bias-Jacobians alongside the preintegrated values.** The
   whole point of preintegration is *not* re-integrating raw IMU data
   every time the optimizer updates its bias estimate. That requires
   three more 3x3 matrices accumulated in the same recursion:
   `∂ΔR/∂bias_gyro`, `∂Δv/∂bias_gyro`, `∂Δv/∂bias_accel`,
   `∂Δp/∂bias_gyro`, `∂Δp/∂bias_accel`. With these, a small bias change
   `δb` can correct the preintegrated deltas via a first-order
   approximation (`ΔR(b+δb) ≈ ΔR(b) · Exp(J_g^ΔR · δb_g)`, and similarly
   linear corrections for `Δv`, `Δp`) instead of re-integration.
3. **Propagate measurement covariance alongside the preintegrated values**
   (a 9x9 covariance over the `(ΔR, Δv, Δp)` residual, via the same
   linearized noise-propagation recursion Forster et al. describe), so the
   factor's information matrix reflects actual sensor noise instead of the
   ad-hoc fixed scalar weight `RotationOnlyInformation` uses today.

**The factor itself**, connecting `NavState` at keyframe `i` and `j`
(gravity `g` is a fixed, known 3-vector — see 2.4 for where it comes
from):

```
r_ΔR = Log[ ΔR_ij(b_i)⁻¹ · R_i⁻¹ · R_j ]                              (3-dim)
r_Δv = R_i⁻¹ (v_j - v_i - g·Δt) - Δv_ij(b_i)                          (3-dim)
r_Δp = R_i⁻¹ (p_j - p_i - v_i·Δt - ½g·Δt²) - Δp_ij(b_i)               (3-dim)
r_b  = b_j - b_i                                                       (6-dim, separate factor: bias random walk)
```

**Jacobians — the recommendation that matters most in this section.**
`PoseGraph::Solve()` (Phase 3) uses numerical (central-difference)
Jacobians for exactly one reason: there's no compiler in the environment
this project is being written in to empirically catch a sign error in an
analytic derivation, and an SE3 adjoint Jacobian is easy to get subtly
wrong. That reasoning applies *more* strongly here — the bias-Jacobian
recursion above is meaningfully more intricate than a pose-graph adjoint.
Two honest options, in order of recommendation:

- **Numeric-with-reintegration (recommended first pass):** don't implement
  the analytic bias-Jacobian recursion at all. To evaluate
  `∂r/∂bias`, just *re-integrate* the raw IMU window with a perturbed
  bias and finite-difference the result. This is the whole optimization
  the bias-Jacobian trick exists to avoid, so it's slower — but at
  sliding-window scale (a handful of keyframes, each with maybe a few IMU
  samples given KITTI's 10Hz synced oxts) that cost is very likely
  irrelevant, and it sidesteps implementing SO3-right-Jacobian math that
  can't be verified without a build. Also numerically differentiate
  `∂r/∂pose_i, ∂r/∂velocity_i, ...` the same way `PoseGraph` does. Get
  this correct and tested first.
- **Analytic bias-Jacobians (later optimization):** implement Forster et
  al.'s exact recursion once the numeric version is verified against real
  test runs, and cross-check the analytic Jacobians *against* the working
  numeric ones in a unit test before trusting them (this is a good use of
  a "does the analytic version match finite differences" test regardless
  of which one ships).

**New files:**

```
include/slam/backend/nav_state.hpp          -- NavState struct (2.2)
include/slam/backend/imu_factor.hpp         -- bias-aware preintegration + residual
src/backend/imu_factor.cpp
include/slam/backend/nav_state_graph.hpp    -- see 2.7
src/backend/nav_state_graph.cpp
tests/backend/imu_factor_test.cpp
tests/backend/nav_state_graph_test.cpp
```

`ImuPreintegrator` (existing, zero-bias) should **not** be deleted or
folded into this — it's simpler, still correct for what it does, and
nothing stops both existing side by side (the zero-bias version could
remain the "quick preview" path, e.g. for a live viewer, while the
bias-aware version feeds the actual backend).

### 2.4 Sub-phase 6A.2 — Initialization

A tightly-coupled IMU factor is meaningless without a decent initial
estimate of velocity, gravity direction, and gyro bias — every real VIO
system has an explicit initialization phase before steady-state operation.
Reference: Qin, Li, Shen, **"VINS-Mono: A Robust and Versatile Monocular
Visual-Inertial State Estimator"** (IEEE T-RO, 2018) — specifically its
initialization procedure, not the rest of the paper (VINS-Mono is
monocular and spends most of its complexity on scale recovery, which does
**not apply here** — our VIO is stereo and our LiDAR is inherently metric,
so both already have real scale before initialization even starts. That's
a genuine simplification relative to the reference; note it, don't
re-implement scale recovery.).

Procedure, using a short window of already-available metric-scale poses
(from `VioFrontend` or `LidarFrontend`, whichever's being bootstrapped)
and their corresponding zero-bias-preintegrated IMU segments:

1. **Gyro bias**: linear least-squares using the *same* bias-Jacobian from
   6A.1 (a concrete reason 6A.1 has to exist first): linearize
   `Log(ΔR_i,i+1(b_g)⁻¹ · R_i⁻¹ R_{i+1}) ≈ Log(ΔR_i,i+1(0)⁻¹ · R_i⁻¹
   R_{i+1}) - J_g^ΔR · b_g` and solve the resulting normal equations for
   `b_g` across all consecutive pairs in the window.
2. **Gravity + velocities**: with gyro bias applied, stack the
   preintegrated `Δv`/`Δp` relations for every consecutive pair into one
   linear system with unknowns `[v_0, v_1, ..., v_N, g]` (accel bias still
   assumed zero at this stage — it's poorly observable without deliberate
   acceleration excitation, and is standardly left for the online
   optimizer to refine rather than solved for at init) and solve via
   least squares (`Eigen::JacobiSVD` is fine at this scale). Optionally
   refine `g` to have magnitude exactly 9.81 by reparametrizing to its
   2-DOF tangent plane and re-solving — a nice-to-have documented in
   VINS-Mono, not required for a first correct version.
3. **Accel bias**: initialize to zero; let the online optimizer refine it.

**New files:**

```
include/slam/backend/vio_initializer.hpp
src/backend/vio_initializer.cpp
tests/backend/vio_initializer_test.cpp   -- synthetic known g/v/bias, verify recovery
```

### 2.5 Sub-phase 6A.3 — Tightly-coupled visual factors (optional, likely skip)

Replace `VioFrontend`'s PnP-derived pose edge with raw reprojection
factors: each triangulated landmark observed in multiple keyframes
contributes a 2-DOF pixel-reprojection residual per observation, jointly
optimized against camera poses (now `NavState`s) and 3D landmark
positions in the *same* graph as the IMU factor. This means:

- `VioFrontend` stops producing a `FrameResult.relative_pose` and instead
  exposes raw 2D-3D observation correspondences.
- The backend needs a new factor type and landmark variables (a real
  bundle-adjustment factor, not a pose-graph edge) — this is the actual
  "full bundle adjustment with landmarks" scope that Phase 3's ROADMAP
  notes explicitly deferred (see ROADMAP.md's Phase 3 scope decisions:
  "Pose-graph, not landmark bundle adjustment").
- Landmark culling/management (which landmarks stay in the active window,
  when they get marginalized) becomes a real design problem it currently
  isn't.

This is a substantial rewrite of `VioFrontend`'s public contract and a new
class of backend factor and variable. Only worth doing if there's a
specific reason (e.g. wanting to demonstrate full bundle adjustment
specifically, since Phase 3 explicitly scoped that out).

### 2.6 Sub-phase 6A.4 — Tightly-coupled LiDAR factors (optional, likely skip)

Same idea for LiDAR: feed raw point-to-plane/point-to-line residuals
(`scan_matcher.hpp`'s math, already correct) directly into the joint
optimization instead of a pre-solved ICP pose. Reference: Shan, Englot,
**"LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and
Mapping"** (IROS, 2020) for how a real system structures this (it uses
IMU-predicted poses to *initialize* each scan match, which is a lighter,
worthwhile middle ground worth considering on its own even without going
all the way to joint optimization — see the note at the end of this
section). Same cost profile as 6A.3: `LidarFrontend`'s contract changes,
new factor type, correspondence data has to survive into the backend.

**A cheaper middle ground worth calling out separately**: even without
fully joint optimization, using the IMU-preintegrated relative rotation
(and, once 6A.1/6A.2 exist, position) to *seed* `MatchScans`'s
`initial_guess` parameter (currently always `Sophus::SE3d()` — identity)
would likely improve ICP convergence and robustness on sequences with fast
turns, at near-zero implementation cost. This is IMU-*aided* LiDAR
odometry, not tightly-coupled fusion, but it's a small, low-risk win — a
good candidate to do independently of the rest of this document if
someone wants a quick, contained improvement to Phase 2's frontend.

### 2.7 Solver architecture: don't unify `PoseGraph` and the new 15-DOF graph

`PoseGraph::Solve()` hardcodes 6-DOF blocks throughout its dense
linear-algebra bookkeeping (`H.block<6,6>(6*idx, ...)`, etc.). Generalizing
it to variable-dimension nodes (so 6-DOF pose-only history and 15-DOF
active nav-states could coexist in one graph) is real, non-trivial
refactoring for a benefit that isn't obviously worth it on a first pass.

**Recommendation: build a separate, self-contained `NavStateGraph` class**
(new file, `include/slam/backend/nav_state_graph.hpp`) that is 15-DOF
throughout, structurally a near-copy of `PoseGraph`'s Gauss-Newton loop
with bigger blocks — accept the code duplication rather than the
abstraction cost. `PoseGraph` keeps existing unchanged and remains useful
on its own (e.g. for anything not needing velocity/bias, or a
LiDAR-only/VIO-only pipeline that doesn't want IMU fusion at all).

To still reuse the existing, working VIO/LiDAR `PoseGraphEdge` machinery
inside `NavStateGraph` rather than duplicating *that* too: generalize
`PoseGraphEdge`'s residual/Jacobian computation to operate on "the pose
sub-block of whatever state a node holds" — since a `NavState`'s pose is
still a `Sophus::SE3d` at a known 15-dim tangent offset (0-5), a VIO/LiDAR
edge can zero-pad its 6-dim Jacobian into the right 15-dim columns.
Structurally: keep computing the exact same 6-dim residual/Jacobian
`PoseGraph`/`PoseGraphEdge` already compute correctly, and just place them
into the larger `NavStateGraph` block at the right offset — new
"placement" logic, not new math.

If, after actually building this, the duplication between `PoseGraph` and
`NavStateGraph` turns out to be painful to maintain, a generalization pass
unifying them into one variable-dimension solver is a reasonable follow-up
— but don't do that speculatively before the concrete need is felt.

### 2.8 Testing strategy for Part A

Follow the project's established pattern: synthetic, hand-verifiable
scenes, not real KITTI data, for unit tests.

- `imu_factor_test.cpp`: constant-acceleration / constant-turn-rate
  synthetic IMU sequences (same style as `imu_preintegrator_test.cpp`);
  verify bias-corrected preintegration against a hand-computed closed
  form; verify the bias-Jacobian correction by comparing "preintegrate
  with a perturbed bias directly" against "preintegrate once, then apply
  the Jacobian correction" — these two must agree to first order, and this
  comparison is a genuine, concrete correctness check regardless of
  whether the Jacobians end up analytic or numeric.
- `vio_initializer_test.cpp`: synthetic scenario with a *known* gravity
  vector, velocity sequence, and gyro bias; generate consistent synthetic
  poses + IMU from them; verify the linear solve recovers all three to
  tight tolerance — mirrors `scan_matcher_test.cpp`'s
  "construct-a-known-answer-and-check-recovery" pattern from Phase 2.
- `nav_state_graph_test.cpp`: mirror `pose_graph_test.cpp`'s two structural
  tests — (a) a minimal two-node convergence check, (b) a drift-correction
  scenario analogous to the hexagon loop-closure test: a chain of IMU
  factors alone accumulates a known bias/gravity-induced error, then
  adding a single accurate VIO/LiDAR pose edge should visibly correct it,
  directly demonstrating that fusion (not either sensor alone) is doing
  real work — the same "does this actually help" framing
  `LoopClosureCorrectsAccumulatedDrift` used in Phase 3.

### 2.9 Effort/risk summary — Part A

| Sub-phase | Size | Risk | Recommendation |
|---|---|---|---|
| 6A.1 Full IMU factor | Large | High (densest math in the project) | Do it; use numeric Jacobians first pass |
| 6A.2 Initialization | Medium | Medium (well-precedented, self-contained) | Do it; required to make 6A.1 usable |
| 6A.3 Tightly-coupled visual factors | Large | Medium (mostly engineering, not math risk) | Skip unless there's a specific reason |
| 6A.4 Tightly-coupled LiDAR factors | Large | Medium | Skip; consider the cheap IMU-seeded-ICP alternative (2.6) instead |

---

## 3. Part B — ROS2 wrapping

### 3.1 Goals and non-goals

**Goal**: replace `slam::sensors::KittiSequenceReader` (an offline,
indexed dataset reader) with a ROS2 node that drives the exact same
`VioFrontend` / `LidarFrontend` / `SlidingWindowOptimizer` /
`mapping::Map` pipeline from live topics or a rosbag, and publishes
results in standard ROS2 message types so `rviz2` and other ROS tooling
can consume them. This is precisely the seam `sensors/`'s plain-struct
boundary (`ImageFrame`, `ImuMeasurement`, `LidarScan`, `StereoFrame`) was
designed for from Phase 0 onward — see README.md's architecture section.

**Non-goals**: this document does not propose changing `slam_core`'s
algorithms for real-time performance (frame-rate budget, latency
guarantees) — it only covers wiring the existing pipeline to ROS2 I/O. It
also does not require Part A; it wraps whatever fusion the backend
currently does, loosely- or tightly-coupled.

### 3.2 Package layout

Keep `slam_core`'s existing vcpkg+CMake build entirely as-is (see 3.6 for
why) and add ROS2 as a **separate, external overlay**:

```
ros2_ws/
  src/
    slam_ros2/
      package.xml
      CMakeLists.txt              -- ament_cmake, find_package(slam CONFIG REQUIRED)
      include/slam_ros2/
        message_adapters.hpp      -- ROS msg <-> slam:: struct conversions
        slam_node.hpp
      src/
        message_adapters.cpp
        slam_node.cpp
        slam_node_main.cpp
      launch/
        slam.launch.py
      config/
        params.yaml                -- frame ids, calibration path, window size, etc.
      rviz/
        slam.rviz
      test/
        test_message_adapters.cpp  -- plain unit tests, no ROS runtime needed
```

### 3.3 Message adapters

Pure, ROS-runtime-independent conversion functions — these are the part of
Part B that's actually unit-testable the way the rest of this project is:

```cpp
// message_adapters.hpp
slam::ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg);
slam::ImuMeasurement ToImuMeasurement(const sensor_msgs::msg::Imu& msg);
slam::LidarScan ToLidarScan(const sensor_msgs::msg::PointCloud2& msg);
slam::StereoCalibration ToStereoCalibration(const sensor_msgs::msg::CameraInfo& left,
                                             const sensor_msgs::msg::CameraInfo& right);

nav_msgs::msg::Odometry ToOdometryMsg(const Sophus::SE3d& pose, const Eigen::Vector3d& velocity,
                                       const std::string& frame_id, rclcpp::Time stamp);
sensor_msgs::msg::PointCloud2 ToPointCloud2(const std::vector<Eigen::Vector3d>& points,
                                             const std::string& frame_id, rclcpp::Time stamp);
```

- `ToImageFrame`: needs `cv_bridge` (a real, unavoidable dependency —
  there's no reasonable way to get `sensor_msgs::Image` into an
  `cv::Mat` without it or reimplementing it).
- `ToLidarScan`: hand-parse `PointCloud2`'s `x,y,z,intensity` float32
  fields via `sensor_msgs::PointCloud2Iterator` (part of `sensor_msgs`
  core, not a new dependency) rather than pulling in PCL — consistent
  with this project's preference for minimal dependencies (no PCL
  anywhere else in the codebase either; see `frontend_lidar`'s own
  from-scratch k-d tree). PCL + `pcl_conversions` is the more common/
  idiomatic ROS approach if convention matters more than dependency count
  — a reasonable alternative choice, just not the one this document
  recommends.
- `ToStereoCalibration`: `CameraInfo`'s `K`/`P` fields map directly onto
  `CameraIntrinsics`/`StereoCalibration` the same way `calib.txt` parsing
  does in `kitti_dataset.cpp` — this *replaces* `LoadCalibration()` for
  the live case.

**Testing**: `test_message_adapters.cpp` hand-builds `sensor_msgs`
messages (no running ROS2 node/executor needed to construct a message
object) and checks the conversion, exactly like `kitti_dataset_test.cpp`
hand-builds synthetic KITTI files. This should be the "must-have" testing
deliverable for Part B; see 3.7 for why the rest is lower priority.

### 3.4 Node design

**Topics:**

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `~/left/image_raw` | `sensor_msgs/Image` | sub | matches KITTI `image_0` |
| `~/right/image_raw` | `sensor_msgs/Image` | sub | matches KITTI `image_1` |
| `~/left/camera_info` | `sensor_msgs/CameraInfo` | sub | replaces `calib.txt` |
| `~/imu` | `sensor_msgs/Imu` | sub | body-frame accel/gyro |
| `~/points` | `sensor_msgs/PointCloud2` | sub | raw LiDAR scan |
| `~/odometry` | `nav_msgs/Odometry` | pub | fused pose + velocity |
| `~/path` | `nav_msgs/Path` | pub | trajectory, for rviz2 |
| `~/map` | `sensor_msgs/PointCloud2` | pub | `mapping::Map` points |
| `/tf` | `tf2_msgs/TFMessage` | pub | see 3.5 |

QoS: sensor subscriptions use `rclcpp::SensorDataQoS()` (best-effort,
shallow queue — the ROS2 convention for high-rate sensor topics, and
correct here since a dropped frame under load is much better than an
ever-growing queue); publishers use the reliable default.

**Threading model**: message callbacks should do the absolute minimum
(convert + push onto a small bounded queue, e.g. capacity 2-4) and return
immediately; a **dedicated processing thread** pulls from that queue and
runs the actual `VioFrontend`/`LidarFrontend`/`SlidingWindowOptimizer`
pipeline. This is a standard pattern and worth stating explicitly because
getting it wrong is a classic, easy-to-miss ROS2 bug: if the SLAM pipeline
runs directly inside a subscription callback, it blocks the executor
thread, which can silently starve other callbacks (including the ones
feeding the next frame) rather than failing loudly.

**Time synchronization**: stereo left/right images should use
`message_filters::TimeSynchronizer` (or `ApproximateTime` — hardware
stereo rigs rarely have bit-identical timestamps) to pair them. For
combining stereo + LiDAR + IMU, `message_filters` gets unwieldy past two
or three topics at very different rates (IMU at ~100-200Hz vs. camera/
LiDAR at ~10Hz); the simpler, more robust pattern that also matches what
the frontends already assume (see `VioFrontend::ProcessImu` /
`ProcessStereoFrame`'s calling convention in `main_backend_demo.cpp`) is:
**process on stereo-pair arrival**, look up the nearest-in-time buffered
LiDAR scan, and feed every IMU message buffered since the last processed
frame into `ProcessImu` before calling `ProcessStereoFrame` — a small
ring-buffer per topic, not a generic multi-topic synchronizer.

### 3.5 Coordinate frame conventions — the sharpest edge in this whole document

This is the detail most likely to produce a SLAM system that runs,
compiles, publishes messages, and still looks wrong (trajectory sideways
or upside-down in `rviz2`) if skipped or rushed.

- **This project's internal convention** (inherited from KITTI's camera
  calibration, i.e. the standard OpenCV/computer-vision optical frame):
  X-right, Y-down, Z-forward. Every `Sophus::SE3d` produced by
  `VioFrontend`, `LidarFrontend`, and the backend is in this convention.
- **ROS2 REP-103** body frame convention: X-forward, Y-left, Z-up.
- **ROS2 REP-105** frame tree: `map` → `odom` → `base_link`, with sensor
  frames hung off `base_link` via static transforms.

Fix: a single fixed rotation `R` converts optical-frame axes to
base-frame axes. Deriving it directly (base_x = optical z [forward],
base_y = -optical x [left = negative right], base_z = -optical y
[up = negative down]):

```
        [ 0   0   1 ]
R_opt→base = [-1   0   0 ]
        [ 0  -1   0 ]
```

Apply it as a fixed similarity conjugation when publishing a pose:
`T_ros = T_R · T_optical · T_R⁻¹`, where `T_R = Sophus::SE3d(R,
Vector3d::Zero())` — this re-expresses both the body frame's axes *and*
the world/map frame's axes consistently (the simplest self-consistent
choice: apply the same fixed rotation to both sides). Do this once, in
`ToOdometryMsg`/the tf broadcaster, not scattered through the pipeline.

**Do not trust this formula blindly** — verify it empirically before
relying on it: publish a trajectory for a sequence with obvious forward
motion and confirm it actually moves along `+X` in `rviz2`, not sideways
or backward. This is exactly the kind of detail that looks fine in a code
review and is wrong in practice; a 30-second visual check catches it, code
reading alone might not.

### 3.6 Build system integration

Keep `slam_core` building via its existing vcpkg + plain CMake path,
completely unaware of ROS2 (no `ament_cmake`, no `rclcpp` dependency in
`slam_core` itself). Two ways to connect it to `ros2_ws`:

- **Recommended: library-first.** `cmake --install` the existing build to
  produce a standard CMake package (`slamConfig.cmake` + friends via
  `install(TARGETS slam_core EXPORT ...)`, which needs adding to the root
  `CMakeLists.txt` — not present today). `ros2_ws/src/slam_ros2`'s
  `ament_cmake` `CMakeLists.txt` then does a plain
  `find_package(slam CONFIG REQUIRED)` and links against it like any other
  external library. Clean separation: `slam_core` stays a portable,
  ROS-independent library (the entire point of the `sensors/` boundary
  design from Phase 0), and ROS2-specific code lives entirely in
  `slam_ros2`.
- **Alternative: colcon overlay.** Add a `package.xml` at the repo root
  and make the root `CMakeLists.txt` itself `ament_cmake`-compatible, so
  the whole repo builds via `colcon build`. Simpler for a single combined
  build, but undermines the "portable core library" framing this project
  has maintained since Phase 0 — not recommended.

This needs a small addition to the existing root `CMakeLists.txt`
regardless (an `install()` section for `slam_core` and its public
headers) — worth doing early in Part B even before writing any ROS2 code,
since it's useful independent of ROS2 (anyone linking this library from
another CMake project needs it too).

### 3.7 Testing strategy

- **Must-have**: `test_message_adapters.cpp` (3.3) — pure functions, no
  ROS2 runtime required, same testing philosophy as the rest of this
  project (synthetic fixtures, hand-verifiable expected values).
- **Nice-to-have, lower priority**: `launch_testing`-based integration
  tests that actually spin up `slam_node`, publish synthetic messages on
  its subscribed topics, and assert on what it publishes back. This is
  meaningfully more complex and fragile infrastructure (a running
  executor, timing-sensitive assertions) for a smaller marginal
  correctness gain than the adapter unit tests — reasonable to defer or
  skip, unlike the adapters themselves.

### 3.8 Effort/risk summary — Part B

| Piece | Size | Risk | Notes |
|---|---|---|---|
| Message adapters | Small-Medium | Low | Pure functions, directly testable |
| Node + threading + sync | Medium | Medium | Standard patterns, but easy to get the threading wrong |
| Frame conventions | Small (code), high stakes | **High** | Small amount of code, very easy to get subtly wrong; verify visually |
| Build integration | Small | Low | One `install()` block + a standard `ament_cmake` package |
| Testing | Small (adapters) / Medium (launch_testing) | Low / Medium | Prioritize adapter tests |

---

## 4. Recommended sequencing

These are independent tracks (Section 0), so "sequencing" is really "what
to prioritize," not a hard dependency chain:

1. **Prerequisite, still outstanding as of this document**: an actual
   build + test run of Phases 0-5. Everything in this project so far has
   been reasoned through without a compiler; Phase 6 adds more of the same
   kind of risk on top of an already-unverified base. Do this first,
   regardless of which Phase 6 track comes next.
2. **If prioritizing SLAM depth**: 6A.1 → 6A.2 (Section 2). This is the
   more technically demanding, more "hard SLAM math" track, and directly
   upgrades the one piece of the project (`RotationOnlyInformation`) that
   was explicitly flagged as a partial/simplified fusion.
3. **If prioritizing systems/robotics integration breadth**: Part B
   (Section 3). Independent of 6A, and arguably more relevant if the goal
   is demonstrating "this could run on a real robot" rather than
   "this recovers state-of-the-art accuracy."
4. **Skip unless there's a specific reason**: 6A.3, 6A.4 (Section 2.5-2.6).
   Large rewrites of the frontend/backend boundary for benefit that's
   unlikely to matter for a portfolio-scale project already showing
   competence via 6A.1/6A.2 and Part B.

---

## 5. Risk register

| Risk | Where | Mitigation |
|---|---|---|
| Bias-Jacobian derivation error | 6A.1 | Use numeric Jacobians first; cross-check any later analytic version against them |
| Initialization divergence (bad gravity/velocity estimate poisons everything downstream) | 6A.2 | Synthetic-scenario unit tests with known ground truth before trusting it on real data |
| `PoseGraph`/`NavStateGraph` duplication becomes a maintenance burden | 2.7 | Accept it deliberately for now; only unify if the pain is actually felt, not speculatively |
| ROS2 executor starvation from blocking callbacks | 3.4 | Dedicated processing thread + bounded queue, not inline processing in callbacks |
| Frame convention bug (trajectory sideways/upside-down) | 3.5 | Explicit fixed-rotation formula given above; verify visually, don't trust by inspection alone |
| `slam_core` accidentally gains a ROS2/rclcpp dependency, breaking its portability | 3.6 | Keep `ament_cmake`/`rclcpp` entirely inside `slam_ros2`; `slam_core`'s CMakeLists never mentions ROS |
| Real-time performance (this document doesn't address it) | Part B generally | Out of scope here; would need actual profiling against real sensor rates once built |

---

## 6. Definition of done

**6A.1 (Full IMU factor)**: `ImuFactor` computes bias-corrected
preintegration + (numeric, per the recommendation) Jacobians; unit tests
verify bias-correction against direct re-integration; residual is zero for
noise-free synthetic data satisfying the IMU kinematics exactly.

**6A.2 (Initialization)**: `VioInitializer` recovers known synthetic
gravity/velocity/gyro-bias to tight tolerance; documented as *not*
recovering scale (already known from metric VIO/LiDAR).

**6A.3/6A.4**: not planned; revisit only with a specific motivating reason.

**Part B**: `slam_node` builds against an installed `slam_core` via
`ament_cmake`; message adapter unit tests pass; a real rosbag or live
sensor run produces a visually-correct (forward-moving, right-side-up)
trajectory in `rviz2`; `README.md` gets a "Running on ROS2" section
alongside the existing KITTI-demo instructions.

---

## 7. References

- Forster, Carlone, Dellaert, Scaramuzza. "On-Manifold Preintegration for
  Real-Time Visual-Inertial Odometry." *IEEE Transactions on Robotics*,
  2017. — IMU preintegration with bias Jacobians (6A.1).
- Qin, Li, Shen. "VINS-Mono: A Robust and Versatile Monocular
  Visual-Inertial State Estimator." *IEEE Transactions on Robotics*, 2018.
  — Initialization procedure (6A.2); ignore its monocular scale-recovery
  content, not applicable here.
- Shan, Englot. "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via
  Smoothing and Mapping." *IROS*, 2020. — Reference architecture for
  tightly-coupled LiDAR-IMU fusion (6A.4) and IMU-seeded scan matching
  (the cheaper alternative noted in 2.6).
- REP-103 (Standard Units of Measure and Coordinate Conventions) and
  REP-105 (Coordinate Frames for Mobile Platforms) — ROS2 conventions
  referenced in 3.5.
- This project's own `ROADMAP.md`, particularly Phase 3's "Scope decisions
  made here" section, which this document extends rather than repeats.
