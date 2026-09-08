# Roadmap

Foundation project for LiDAR + visual-inertial SLAM, built module by
module so each phase leaves a runnable, testable increment. See
[README.md](README.md) for architecture and build instructions.

**Optimizer policy:** the backend (sliding-window bundle adjustment,
pose-graph optimization) is hand-written using Eigen/Sophus — no GTSAM,
g2o, or Ceres. If it's numerically unstable, the fix is better math
(correct Jacobians, robust kernels, damping/trust-region tuning), not
swapping in a library.

**Build status:** the `Dockerfile` at the repo root
(`docker build --target runtime .`) builds `slam_core` via vcpkg and runs
the full test suite as part of the image build — a successful build is
the source of truth for whether the project compiles and passes tests.

## Phase 0 — Scaffolding
- [x] Repo structure, CMake + vcpkg manifest, CI (GitHub Actions, Linux + Windows)
- [x] `slam::common` data types (`ImageFrame`, `ImuMeasurement`, `LidarScan`)
- [x] KITTI odometry sequence reader (images, Velodyne scans, ground-truth poses) + unit tests
- [ ] Download a KITTI odometry sequence locally and run `slam_kitti_demo` against it

## Phase 1 — VIO front-end
- [x] Feature detection + KLT tracking across frames (`FeatureTracker`)
- [x] Stereo triangulation + PnP relative pose estimation (`stereo_geometry`) —
      stereo (not monocular 5-point) so the recovered trajectory has real
      metric scale, no separate scale-recovery step needed
- [x] IMU preintegration between keyframes (`ImuPreintegrator`), unit-tested
- [x] Raw (unoptimized) VIO trajectory on a KITTI sequence (`slam_vio_demo`)

**IMU (rotation-only fusion):** `KittiOxtsReader`
(`slam/sensors/kitti_raw_imu.hpp`) reads a raw KITTI drive's `oxts/`
folder; `LookupRawDriveMapping` maps odometry sequences 00-10 to their
source raw drive + start frame (the official devkit's sequence-to-raw
table). `slam_backend_demo` wires it in when given a raw dataset root as
a third argument, feeding real body-frame accel/gyro
(`ax,ay,az`/`wx,wy,wz` — not the roll/pitch-leveled `af,al,au`/`wf,wl,wu`
fields) into `VioFrontend::ProcessImu` each frame.

The backend (`SlidingWindowOptimizer::AddKeyframe`) consumes the
resulting preintegrated delta as a pose-graph edge — rotation only. Gyro
integration over one KITTI frame interval (~0.1s) is trustworthy without
a velocity state or bias correction; the preintegrated position is not
(it assumes zero initial velocity and isn't gravity-compensated). The IMU
edge's information matrix therefore zeros out its translation rows
(`RotationOnlyInformation`), relying on `Sophus::SE3d::log()`'s rotation
component being independent of translation. Full-state IMU fusion
(velocity + bias as optimizable state) is the tightly-coupled backend
described under Phase 6.

KITTI's raw *synced* oxts data is one sample per image frame, so this is
a single preintegration step per keyframe interval, not multi-sample
high-rate preintegration.

## Phase 2 — LiDAR front-end
- [x] Point-cloud downsampling (`VoxelDownsample`, hash-grid centroid merge)
      and ground filtering (`RemoveGround`, RANSAC plane fit) — both from
      scratch, no PCL
- [x] Edge and planar feature extraction (LOAM-style curvature ranking,
      `ExtractFeatures`), with ring index estimated from vertical angle
      since KITTI's `.bin` files don't carry it
- [x] Scan-to-scan point-to-line/point-to-plane Gauss-Newton ICP
      (`MatchScans`), correspondence search via a hand-written k-d tree
      (`KdTree3d`) — no PCL/g2o/Ceres
- [x] Raw LiDAR odometry trajectory on the same sequence (`slam_lidar_demo`)

**Design note:** `RemoveGround` is a standalone, tested utility,
deliberately not wired into `ExtractFeatures` — the ground plane is
itself a strong planar-feature source for ICP (constrains z/roll/pitch),
so stripping it before feature extraction would discard useful
constraints. It's used by mapping instead (Phase 4), for
obstacle/free-space separation.

## Phase 3 — Backend
- [x] Hand-written Gauss-Newton pose-graph optimization (`PoseGraph`) over
      the SE3 manifold, dense normal equations, no g2o/GTSAM/Ceres
- [x] Shared pose graph consuming both front-ends (`SlidingWindowOptimizer`)
      — each keyframe gets a VIO edge and/or a LiDAR edge, weighted by
      inlier/correspondence count, plus a rotation-only IMU edge when real
      IMU data is available, jointly optimized
- [x] Sliding window: keyframes older than the window are frozen
      (`PoseGraph::FixNode`) rather than dropped; `OptimizeGlobally()` does
      a one-shot full-graph re-solve (e.g. at the end of a sequence run)
- [x] Loop closure detection + pose-graph correction (`DetectLoopClosure`):
      geometric-proximity candidates verified by LiDAR scan registration
- [x] `slam_backend_demo`: runs VIO + LiDAR + backend together over a KITTI
      sequence

**Scope decisions (read before extending this phase):**
- **Pose-graph, not landmark bundle adjustment.** "Bundle adjustment" in
  the literature usually means jointly optimizing camera poses *and* 3D
  landmark positions via reprojection error, with landmarks marginalized
  out via a Schur complement. This phase optimizes keyframe poses only,
  constrained by each frontend's per-frame relative-pose estimate — a
  real, useful technique (what many lightweight SLAM backends do), but a
  smaller scope than full landmark BA. Landmark-level optimization is a
  candidate follow-up, not assumed here.
- **Frozen, not marginalized, window boundary.** Dropping a keyframe from
  the active window by fixing its pose (`FixNode`) discards the
  information a proper marginalization would fold into a prior on its
  neighbor. Simpler and still correct — the sliding window is more
  conservative about revising old history than a fully marginalized
  backend would be.
- **Numerical, not analytic, edge Jacobians.** `PoseGraph::Solve()` uses
  central finite differences instead of hand-deriving the SE3 adjoint
  Jacobian of the edge residual. Both are legitimate from-scratch
  techniques; numerical differentiation avoids the risk of an
  undetected sign error in an adjoint derivation, and the cost is
  negligible at pose-graph scale.
- **Geometric loop closure, not appearance-based.** `DetectLoopClosure`
  uses trajectory proximity + LiDAR scan-matching verification, not a
  visual bag-of-words / Scan Context style place recognizer. An
  appearance-based recognizer is a substantially larger undertaking, left
  for future work.

## Phase 4 — Mapping + visualization
- [x] Sparse landmark map (VIO) and voxel/point-cloud map (LiDAR)
      (`mapping::Map`) — both built on the same `PointCloudMap` incremental
      voxel accumulator at different granularity; see its doc comment for
      why that's one structure, not two
- [x] `slam_mapping_demo`: two-pass map build (collect per-frame local
      points, run `OptimizeGlobally()`, then transform+merge with final
      poses) exported to a standard ASCII PLY file — viewable in
      MeshLab/CloudCompare/Blender without any of this project's own code
- [x] Pangolin live trajectory + map viewer (`slam::viz::PangolinViewer`,
      `slam_viz_demo`) — opt-in only (`-DSLAM_BUILD_VIZ=ON` +
      `--preset with-viz` for the vcpkg `viz` feature), not part of the
      default build

**On the viewer:** `PangolinViewer` is graphics/windowing code, unlike
the rest of this project (residual formulas, Jacobians, RANSAC geometry),
which is derivable and testable by hand. It follows the standard Pangolin
usage pattern (the same structure ORB-SLAM2's viewer uses). Build and run
it to confirm it renders correctly before relying on it. It's gated
behind `SLAM_BUILD_VIZ` (default off) and the vcpkg `viz` feature
specifically so it never affects the default build or CI.
`slam_mapping_demo`'s PLY export has no such caveat — plain, testable
file I/O with unit test coverage.

**LiDAR map density:** `slam_mapping_demo`/`slam_viz_demo` build the
LiDAR map from `LidarFrontend::LastFeatures()` (the edge/planar points
already computed for odometry), not the full raw scan — holding a full
KITTI sequence's raw point clouds across an entire run is impractical. A
full-density map fed from raw scans is a reasonable future enhancement.

## Phase 5 — Evaluation
- [x] ATE scoring (`ComputeAte`) — RMSE after closed-form Kabsch/Horn rigid
      (no-scale) alignment, the standard literature metric
- [x] RPE scoring (`ComputeKittiOdometryError`) — KITTI's own official
      odometry evaluation protocol (100-800m ground-truth-path segments,
      avg translation % + rotation deg/100m), so numbers are comparable to
      published results
- [x] `slam_eval_demo`: runs VIO-only, LiDAR-only, fused-incremental, and
      fused-globally-optimized trajectories over a sequence and prints all
      four metrics side by side
- [ ] Comparison table vs. published LOAM/ORB-SLAM3 numbers in the README
      — requires a downloaded sequence and a real `slam_eval_demo` run;
      see README.md's Evaluation section for the template.

## Phase 6 — Tightly-coupled fusion + ROS2

Full design in [PHASE6_PLAN.md](PHASE6_PLAN.md).

- [x] **Full IMU factor.** `backend::NavState` (15-DOF: pose + velocity +
      gyro/accel bias), `backend::ImuPreintegration` (bias-aware
      preintegration; bias changes are handled by re-integrating the raw
      buffer via `BiasCorrected()`, not an analytic bias-Jacobian — see
      the class doc comment and PHASE6_PLAN.md §2.3), and
      `ComputeImuFactorResidual` (9-dim motion + 6-dim bias-random-walk
      residual). Unit-tested against hand-derived closed-form synthetic
      motion.
- [x] **`NavStateGraph` solver.** A separate 15-DOF Gauss-Newton solver
      (deliberately not unified with `PoseGraph` — PHASE6_PLAN.md §2.7)
      that fuses `NavPoseEdge` (VIO/LiDAR-style relative-pose constraints
      on the pose sub-block) and `NavImuEdge` (the IMU factor above).
      `HighWeightPoseEdgeDominatesOverImuEdge` verifies fusion changes the
      outcome, the same test pattern as Phase 3's loop closure.
- [x] **Initialization.** `backend::InitializeVio`: gyro bias via a small
      numeric Gauss-Newton solve, then gravity + per-keyframe velocity via
      one linear least-squares solve — no scale unknown to solve for
      (this project's odometry is already metric). Recovers known
      synthetic gravity/velocity/bias to tight tolerance.
- [x] **Wired into a running pipeline.** `backend::TightlyCoupledOptimizer`
      (`tightly_coupled_optimizer.hpp/.cpp`) is a `NavStateGraph`-based
      sibling to `SlidingWindowOptimizer`: every keyframe gets pose edges
      (VIO/LiDAR/loop-closure) immediately, while raw IMU is buffered per
      interval (`AddImuMeasurement`) until `VioInitializer` succeeds over
      a trailing window of keyframes — at which point gravity and that
      window's velocity/bias are seeded, `NavImuEdge`s are added
      retroactively, and every keyframe after that gets a live IMU factor
      alongside its pose edges (`IsInitialized()` reports which stage is
      active; if IMU data never arrives or every window is degenerate, it
      runs pose-only — a safe fallback, not an error). `slam_tightly_coupled_demo`
      runs this end to end on a real KITTI sequence; `slam_eval_demo`,
      given a raw KITTI root, adds tightly-coupled rows to its comparison
      table. `SlamNode` (Part B) also runs this backend — see below. A
      separate class from `PoseGraph`/`SlidingWindowOptimizer`, per
      PHASE6_PLAN.md §2.7.
- [ ] Tightly-coupled visual factors — not planned; see PHASE6_PLAN.md §2.5
- [ ] Tightly-coupled LiDAR factors — not planned; see PHASE6_PLAN.md §2.6
- [x] **ROS2 wrapping** (`ros2_ws/src/slam_ros2/`): message adapters
      (`message_adapters.hpp/.cpp`) converting ROS2 messages to/from
      `slam_core`'s structs, including the optical→REP-103
      frame-convention fix (PHASE6_PLAN.md §3.5); `SlamNode` wraps
      VIO/LiDAR/backend/mapping as a live node (buffered callbacks +
      dedicated processing thread, per §3.4); `slam_core` has a CMake
      package export so `slam_ros2` can `find_package` it without
      `slam_core` depending on ROS2 (§3.6). `SlamNode` runs
      `TightlyCoupledOptimizer` (not `SlidingWindowOptimizer`) — buffered
      IMU messages feed both `VioFrontend::ProcessImu` and
      `TightlyCoupledOptimizer::AddImuMeasurement`, and `/odometry`'s
      twist carries the real fused velocity once initialization succeeds
      (zero before that).

**Confidence within ROS2 wrapping:** `message_adapters.*` are pure
functions, unit-tested with hand-built messages
(`test_message_adapters.cpp`), including a correctness check of the
frame-conversion matrix against its own defining property — comparable
confidence to the rest of this project. `slam_node.*` combines multiple
ROS2 library surfaces (`rclcpp`, `message_filters`, `tf2_ros`,
`cv_bridge`) into a running node; it's the least build-verified file in
this repository — build and run it against a real ROS2 install before
trusting it.
