# Phase 6 Design: Tightly-Coupled Fusion + ROS2

Design reference for Phase 6 (see [ROADMAP.md](ROADMAP.md) for current
status). Assumes familiarity with the `frontend_vio`, `frontend_lidar`,
and `backend` modules and their conventions (SE3 pose semantics, the
`PoseGraphEdge` measurement convention, `ImuPreintegrator`'s zero-bias
formulation).

## 0. Two independent tracks

- **Part A — tightly-coupled LiDAR-VIO-IMU fusion**: a SLAM-math problem,
  entirely inside `slam_core`, no new dependencies.
- **Part B — ROS2 wrapping**: a systems/integration problem, mostly
  message adapters, threading, and build tooling.

Both tracks are implemented in this repo. `SlamNode` (Part B) runs the
Part A backend (`TightlyCoupledOptimizer`) rather than the loosely-coupled
one — a same-repo integration choice; the tracks remain separable in
general (a fork could revert `SlamNode` to `SlidingWindowOptimizer`
without touching Part A).

## 1. Baseline before this phase

- `frontend_vio::VioFrontend` produces a per-frame relative pose from
  stereo PnP, plus a preintegrated IMU delta (`ImuPreintegrator`) —
  zero bias, no velocity state.
- `frontend_lidar::LidarFrontend` produces a per-frame relative pose from
  point-to-line/point-to-plane ICP.
- `backend::SlidingWindowOptimizer` fuses these loosely: each is an
  independent pose estimate turned into a `PoseGraphEdge` between two
  6-DOF pose-only nodes. The IMU edge contributes rotation only
  (`RotationOnlyInformation`) — without a velocity state or gravity
  compensation, the preintegrated position isn't trustworthy.

This is a real, working pose-graph fusion, but not what the literature
calls "tightly coupled": the IMU doesn't contribute a factor connecting
joint pose/velocity/bias state inside the optimization, it produces an
independent estimate that gets weighted-averaged with vision/LiDAR after
the fact.

## 2. Part A — tightly-coupled fusion

### 2.1 Two tiers

- **Tier 1 (target): full IMU factor.** Give each keyframe a 15-DOF
  navigation state (pose, velocity, accel bias, gyro bias) and let a
  bias-and-gravity-aware IMU factor connect consecutive states inside the
  optimizer, while keeping the VIO/LiDAR frontends unchanged — each still
  hands the backend a pre-solved relative pose, applied as a
  `PoseGraphEdge` to the pose sub-block of the 15-DOF state. This is the
  standard sense in which most VIO systems are tightly coupled with their
  IMU, and it's what makes the preintegrated *position* usable.
- **Tier 2 (not planned): raw measurement factors.** Replace the
  VIO/LiDAR pose-edge inputs with raw measurements — reprojection factors
  for triangulated landmarks, raw point-to-plane/point-to-line residuals —
  jointly optimized with the IMU factor in one graph (OKVIS/VINS-Fusion/
  LIO-SAM style). This removes the frontend/backend pose-edge boundary the
  project is built around, for accuracy gains a loosely-coupled-but-correct
  Tier-1 system usually doesn't need. Large rewrites of both frontends'
  public interfaces and the backend's factor types for marginal benefit;
  not recommended.

### 2.2 State representation

```cpp
// include/slam/backend/nav_state.hpp
struct NavState {
  Sophus::SE3d pose;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_accel{Eigen::Vector3d::Zero()};
};
```

15-DOF tangent, ordered `(pose:6, velocity:3, bias_gyro:3, bias_accel:3)`.
Pose retracts via `pose * Exp(dpose)`; velocity and both biases are plain
vector spaces (`+=`).

### 2.3 IMU factor

Reference: Forster, Carlone, Dellaert, Scaramuzza, "On-Manifold
Preintegration for Real-Time Visual-Inertial Odometry" (IEEE T-RO, 2017).

Changes versus the existing zero-bias `ImuPreintegrator`:

1. Integrate `(gyro - bias_gyro_estimate)` and `(accel -
   bias_accel_estimate)` instead of raw measurements, where the bias
   estimate is the linearization point from the last optimization pass.
2. Propagate bias-Jacobians (`∂ΔR/∂bias_gyro`, `∂Δv/∂bias_gyro`,
   `∂Δv/∂bias_accel`, `∂Δp/∂bias_gyro`, `∂Δp/∂bias_accel`) alongside the
   preintegrated values, so a small bias change can correct the
   preintegrated deltas via a first-order approximation instead of
   re-integrating raw data.
3. Propagate measurement covariance over `(ΔR, Δv, Δp)` via the same
   linearized recursion, so the factor's information matrix reflects
   actual sensor noise instead of a fixed scalar weight.

Residuals connecting `NavState` at keyframes `i` and `j` (gravity `g`
fixed and known — see 2.4):

```
r_ΔR = Log[ ΔR_ij(b_i)⁻¹ · R_i⁻¹ · R_j ]                              (3-dim)
r_Δv = R_i⁻¹ (v_j - v_i - g·Δt) - Δv_ij(b_i)                          (3-dim)
r_Δp = R_i⁻¹ (p_j - p_i - v_i·Δt - ½g·Δt²) - Δp_ij(b_i)               (3-dim)
r_b  = b_j - b_i                                     (6-dim, separate bias random-walk factor)
```

**Jacobians:** `PoseGraph::Solve()` (Phase 3) uses numerical
(central-difference) Jacobians rather than a hand-derived SE3 adjoint,
since an analytic derivation is easy to get subtly wrong without a
compiler to catch a sign error. The bias-Jacobian recursion above is more
intricate, so the same reasoning applies here: the implementation
re-integrates the raw IMU window with a perturbed bias and
finite-differences the result for `∂r/∂bias` (and does the same for
`∂r/∂pose`, `∂r/∂velocity`), rather than the analytic recursion. At
sliding-window scale (a handful of keyframes, each with a few IMU samples
given KITTI's 10Hz synced oxts) the extra cost is negligible. An analytic
version, cross-checked against the numeric one in a unit test, is a
reasonable later optimization but not required.

New files: `include/slam/backend/nav_state.hpp`,
`include/slam/backend/imu_factor.hpp` + `src/backend/imu_factor.cpp`,
`include/slam/backend/nav_state_graph.hpp` + `src/backend/nav_state_graph.cpp`,
`tests/backend/imu_factor_test.cpp`, `tests/backend/nav_state_graph_test.cpp`.

`ImuPreintegrator` (zero-bias) is unchanged and coexists with the
bias-aware version — e.g. as a quick-preview path for a live viewer, while
the bias-aware factor feeds the actual backend.

### 2.4 Initialization

A tightly-coupled IMU factor needs a decent initial estimate of velocity,
gravity direction, and gyro bias before steady-state operation. Reference:
Qin, Li, Shen, "VINS-Mono: A Robust and Versatile Monocular
Visual-Inertial State Estimator" (IEEE T-RO, 2018) — its initialization
procedure only; VINS-Mono's monocular scale recovery doesn't apply here,
since this project's stereo VIO and LiDAR are already metric.

Procedure, over a short window of already-available metric-scale poses
and their zero-bias-preintegrated IMU segments:

1. **Gyro bias**: linear least-squares using the bias-Jacobian from 2.3 —
   linearize `Log(ΔR_i,i+1(b_g)⁻¹ · R_i⁻¹ R_{i+1}) ≈ Log(ΔR_i,i+1(0)⁻¹ ·
   R_i⁻¹ R_{i+1}) - J_g^ΔR · b_g` and solve for `b_g` across all
   consecutive pairs in the window.
2. **Gravity + velocities**: with gyro bias applied, stack the
   preintegrated `Δv`/`Δp` relations for every consecutive pair into one
   linear system with unknowns `[v_0, ..., v_N, g]` (accel bias left at
   zero — poorly observable without deliberate acceleration excitation,
   left for the online optimizer to refine) and solve via least squares.
3. **Accel bias**: initialized to zero, refined online.

New files: `include/slam/backend/vio_initializer.hpp` +
`src/backend/vio_initializer.cpp`,
`tests/backend/vio_initializer_test.cpp` (synthetic known g/v/bias,
verify recovery).

### 2.5 Tightly-coupled visual factors — not planned

Would replace `VioFrontend`'s PnP-derived pose edge with raw reprojection
factors per landmark observation, jointly optimized against camera poses
and 3D landmark positions — the "full bundle adjustment with landmarks"
scope Phase 3 explicitly deferred (see ROADMAP.md's Phase 3 scope notes).
Requires `VioFrontend` to expose raw 2D-3D correspondences instead of a
solved pose, a new backend factor/variable type, and landmark
window-management. Large rewrite of `VioFrontend`'s public contract for
benefit that isn't needed here.

### 2.6 Tightly-coupled LiDAR factors — not planned

Same idea for LiDAR: feed raw point-to-plane/point-to-line residuals
directly into the joint optimization instead of a pre-solved ICP pose.
Reference: Shan, Englot, "LIO-SAM: Tightly-coupled Lidar Inertial
Odometry via Smoothing and Mapping" (IROS, 2020). Same cost profile as
2.5.

A cheaper alternative worth doing independently: use the
IMU-preintegrated relative rotation/position to seed `MatchScans`'s
`initial_guess` (currently always identity), which would likely improve
ICP convergence on fast turns at near-zero implementation cost —
IMU-*aided* LiDAR odometry, not tightly-coupled fusion.

### 2.7 Solver architecture

`PoseGraph::Solve()` hardcodes 6-DOF blocks throughout its dense
linear-algebra bookkeeping (`H.block<6,6>(6*idx, ...)`). Generalizing it
to variable-dimension nodes so 6-DOF and 15-DOF states could coexist in
one graph is non-trivial refactoring for unclear benefit on a first pass.

Chosen instead: a separate, self-contained `NavStateGraph`
(`include/slam/backend/nav_state_graph.hpp`), 15-DOF throughout,
structurally a near-copy of `PoseGraph`'s Gauss-Newton loop with larger
blocks. `PoseGraph` is unchanged and remains useful on its own (anything
not needing velocity/bias).

To reuse the existing `PoseGraphEdge` machinery inside `NavStateGraph`:
its residual/Jacobian computation operates on "the pose sub-block of
whatever state a node holds" — a `NavState`'s pose sits at a known
15-dim tangent offset (0-5), so a VIO/LiDAR edge zero-pads its 6-dim
Jacobian into the right columns. This is new placement logic, not new
math; the 6-dim residual/Jacobian computation itself is unchanged.

If the duplication between `PoseGraph` and `NavStateGraph` becomes a
maintenance burden, a generalization pass unifying them is a reasonable
follow-up — not attempted speculatively ahead of that need.

### 2.8 Testing strategy

Synthetic, hand-verifiable scenes, consistent with the rest of the
project's test style:

- `imu_factor_test.cpp`: constant-acceleration/constant-turn-rate
  synthetic IMU sequences; bias-corrected preintegration checked against
  a hand-computed closed form; bias-Jacobian correction checked by
  comparing "preintegrate with a perturbed bias directly" against
  "preintegrate once, then apply the Jacobian correction" (must agree to
  first order).
- `vio_initializer_test.cpp`: synthetic scenario with known gravity,
  velocity sequence, and gyro bias; verify the linear solve recovers all
  three to tight tolerance.
- `nav_state_graph_test.cpp`: a minimal two-node convergence check, and a
  drift-correction scenario (a chain of IMU factors alone accumulates a
  known bias/gravity-induced error; adding one accurate pose edge should
  visibly correct it) — demonstrating fusion does real work, the same
  framing as Phase 3's `LoopClosureCorrectsAccumulatedDrift`.

### 2.9 Effort/risk summary

| Sub-phase | Size | Risk | Status |
|---|---|---|---|
| Full IMU factor | Large | High (densest math in the project) | Done — numeric Jacobians |
| Initialization | Medium | Medium | Done |
| Tightly-coupled visual factors | Large | Medium | Not planned |
| Tightly-coupled LiDAR factors | Large | Medium | Not planned; IMU-seeded ICP (2.6) is the cheap alternative |

## 3. Part B — ROS2 wrapping

### 3.1 Goals and non-goals

**Goal**: replace `slam::sensors::KittiSequenceReader` (offline, indexed)
with a ROS2 node that drives the same `VioFrontend` / `LidarFrontend` /
optimizer / `mapping::Map` pipeline from live topics or a rosbag, and
publishes results in standard message types for `rviz2` and other ROS
tooling. This is the seam `sensors/`'s plain-struct boundary
(`ImageFrame`, `ImuMeasurement`, `LidarScan`, `StereoFrame`) was designed
for — see README.md's architecture section.

**Non-goals**: real-time performance tuning (frame-rate budget, latency
guarantees) — this covers wiring the existing pipeline to ROS2 I/O only.
Does not require Part A; wraps whatever fusion the backend currently does.

### 3.2 Package layout

`slam_core` keeps its existing vcpkg + CMake build, unaware of ROS2; ROS2
is a separate, external overlay:

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
      launch/slam.launch.py
      config/params.yaml           -- frame ids, calibration path, window size
      rviz/slam.rviz
      test/test_message_adapters.cpp  -- plain unit tests, no ROS runtime needed
```

### 3.3 Message adapters

Pure, ROS-runtime-independent conversion functions — the directly
unit-testable part of Part B:

```cpp
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

- `ToImageFrame` uses `cv_bridge` — unavoidable for `sensor_msgs::Image`
  → `cv::Mat`.
- `ToLidarScan` hand-parses `PointCloud2`'s `x,y,z,intensity` fields via
  `sensor_msgs::PointCloud2Iterator` rather than pulling in PCL,
  consistent with the project's from-scratch approach elsewhere (e.g.
  `frontend_lidar`'s k-d tree). PCL + `pcl_conversions` is the more
  idiomatic ROS approach if convention matters more than dependency
  count.
- `ToStereoCalibration`: `CameraInfo`'s `K`/`P` fields map onto
  `CameraIntrinsics`/`StereoCalibration` the same way `calib.txt` parsing
  does in `kitti_dataset.cpp` — replaces `LoadCalibration()` for the live
  case.

`test_message_adapters.cpp` hand-builds `sensor_msgs` messages (no
running node/executor needed) and checks the conversion, the same pattern
`kitti_dataset_test.cpp` uses for synthetic KITTI files.

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
shallow queue — a dropped frame under load is preferable to an
ever-growing queue); publishers use the reliable default.

**Threading**: message callbacks convert and push onto a small bounded
queue (capacity 2-4) and return immediately; a dedicated processing
thread pulls from the queue and runs the pipeline. Running the pipeline
directly inside a subscription callback blocks the executor thread and
can silently starve other callbacks, including the ones feeding the next
frame.

**Time synchronization**: stereo left/right images use
`message_filters::TimeSynchronizer` (or `ApproximateTime`, since hardware
stereo rigs rarely have bit-identical timestamps). For combining stereo +
LiDAR + IMU, `message_filters` gets unwieldy past two or three topics at
very different rates (IMU ~100-200Hz vs. camera/LiDAR ~10Hz); instead:
process on stereo-pair arrival, look up the nearest-in-time buffered
LiDAR scan, and feed every IMU message buffered since the last processed
frame into `ProcessImu` before calling `ProcessStereoFrame` — a small
ring-buffer per topic, matching `VioFrontend`'s existing calling
convention (see `main_backend_demo.cpp`).

### 3.5 Coordinate frame conventions

The detail most likely to produce a system that runs, compiles,
publishes messages, and still looks wrong (trajectory sideways or
upside-down in `rviz2`).

- This project's internal convention (from KITTI's camera calibration,
  the standard OpenCV/computer-vision optical frame): X-right, Y-down,
  Z-forward. Every `Sophus::SE3d` produced by `VioFrontend`,
  `LidarFrontend`, and the backend is in this convention.
- ROS2 REP-103 body frame: X-forward, Y-left, Z-up.
- ROS2 REP-105 frame tree: `map` → `odom` → `base_link`, sensor frames
  hung off `base_link` via static transforms.

Fix: a fixed rotation `R` converts optical-frame axes to base-frame axes
(base_x = optical z [forward], base_y = -optical x [left], base_z =
-optical y [up]):

```
        [ 0   0   1 ]
R_opt→base = [-1   0   0 ]
        [ 0  -1   0 ]
```

Applied as a fixed similarity conjugation when publishing a pose:
`T_ros = T_R · T_optical · T_R⁻¹`, where `T_R = Sophus::SE3d(R,
Vector3d::Zero())`, done once in `ToOdometryMsg`/the tf broadcaster.

Verify empirically before relying on it: publish a trajectory for a
sequence with obvious forward motion and confirm it moves along `+X` in
`rviz2`.

### 3.6 Build system integration

Keep `slam_core` building via its existing vcpkg + plain CMake path,
unaware of ROS2. Recommended approach — library-first: `cmake --install`
the existing build to produce a standard CMake package
(`slamConfig.cmake` via `install(TARGETS slam_core EXPORT ...)` added to
the root `CMakeLists.txt`); `slam_ros2`'s `ament_cmake` `CMakeLists.txt`
then does a plain `find_package(slam CONFIG REQUIRED)`. Keeps
`slam_core` a portable, ROS-independent library — the entire point of the
`sensors/` boundary design.

Alternative (not recommended): a colcon overlay with the root
`CMakeLists.txt` itself made `ament_cmake`-compatible. Simpler for a
single combined build, but undermines the portable-core-library framing.

### 3.7 Testing strategy

- **Must-have**: `test_message_adapters.cpp` (3.3) — pure functions, no
  ROS2 runtime required.
- **Lower priority**: `launch_testing`-based integration tests that spin
  up `slam_node`, publish synthetic messages, and assert on what it
  publishes back. More complex, fragile infrastructure (running executor,
  timing-sensitive assertions) for a smaller marginal correctness gain
  than the adapter unit tests.

### 3.8 Effort/risk summary

| Piece | Size | Risk | Notes |
|---|---|---|---|
| Message adapters | Small-Medium | Low | Pure functions, directly testable |
| Node + threading + sync | Medium | Medium | Standard patterns; easy to get threading wrong |
| Frame conventions | Small (code), high stakes | High | Easy to get subtly wrong; verify visually |
| Build integration | Small | Low | One `install()` block + a standard `ament_cmake` package |
| Testing | Small (adapters) / Medium (launch_testing) | Low / Medium | Prioritize adapter tests |

## 4. Risk register

| Risk | Where | Mitigation |
|---|---|---|
| Bias-Jacobian derivation error | 2.3 | Numeric Jacobians; cross-check any analytic version against them |
| Initialization divergence (bad gravity/velocity estimate poisons downstream state) | 2.4 | Synthetic-scenario unit tests with known ground truth |
| `PoseGraph`/`NavStateGraph` duplication becomes a maintenance burden | 2.7 | Accepted deliberately; unify only if the pain is actually felt |
| ROS2 executor starvation from blocking callbacks | 3.4 | Dedicated processing thread + bounded queue |
| Frame convention bug (trajectory sideways/upside-down) | 3.5 | Fixed-rotation formula above; verify visually |
| `slam_core` gains a ROS2/rclcpp dependency, breaking portability | 3.6 | `ament_cmake`/`rclcpp` stay entirely inside `slam_ros2` |
| Real-time performance unaddressed | Part B generally | Out of scope here; needs profiling against real sensor rates once built |

## 5. Status

Implementation status (what's built, tested, and what remains
build-unverified) is tracked in [ROADMAP.md](ROADMAP.md)'s Phase 6
section, not duplicated here. In summary: 6A.1 (IMU factor), 6A.2
(initialization), and Part B (ROS2 wrapping) are implemented and unit
tested; 6A.3/6A.4 are not planned (2.5, 2.6). The largest open item is an
actual `colcon build` against a real ROS2 install and a live/rosbag run
confirming a visually-correct trajectory in `rviz2` — `slam_node.cpp`'s
exact `rclcpp`/`message_filters`/`tf2_ros`/`cv_bridge` API usage has not
been checked against a real install.

## 6. References

- Forster, Carlone, Dellaert, Scaramuzza. "On-Manifold Preintegration for
  Real-Time Visual-Inertial Odometry." *IEEE Transactions on Robotics*,
  2017. — IMU preintegration with bias Jacobians (2.3).
- Qin, Li, Shen. "VINS-Mono: A Robust and Versatile Monocular
  Visual-Inertial State Estimator." *IEEE Transactions on Robotics*, 2018.
  — Initialization procedure (2.4); its monocular scale-recovery content
  doesn't apply here.
- Shan, Englot. "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via
  Smoothing and Mapping." *IROS*, 2020. — Reference architecture for
  tightly-coupled LiDAR-IMU fusion (2.6) and IMU-seeded scan matching.
- REP-103 (Standard Units of Measure and Coordinate Conventions) and
  REP-105 (Coordinate Frames for Mobile Platforms) — ROS2 conventions
  referenced in 3.5.
- This project's own `ROADMAP.md`, particularly Phase 3's scope-decision
  notes on pose-graph vs. landmark bundle adjustment.
