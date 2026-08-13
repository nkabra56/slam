# Roadmap

Foundation project for LiDAR + visual-inertial SLAM, built module by module so
each phase leaves a runnable, testable increment. See [README.md](README.md)
for architecture and build instructions.

**Optimizer policy:** the Phase 3 backend (sliding-window bundle adjustment,
pose-graph optimization) is hand-written using Eigen/Sophus — no GTSAM, g2o,
or Ceres fallback. If it's numerically unstable, the fix is better math
(correct Jacobians, robust kernels, damping/trust-region tuning), not
swapping in a library.

**On "never build-verified":** every phase below was written without a
compiler in the loop, and that phrase repeats throughout this document as
an honest caveat, not a rhetorical one. The `Dockerfile` at the repo root
(`docker build --target runtime .`) is meant to finally close that gap —
it builds `slam_core` via vcpkg and runs the full test suite as part of
the image build, so a successful `docker build` is real verification. It
hasn't been run here either (no Docker install in this environment), so
until someone runs it, "unverified" still applies to everything, Docker
tooling included — see the Dockerfile's own header comment.

## Phase 0 — Scaffolding
- [x] Repo structure, CMake + vcpkg manifest, CI (GitHub Actions, Linux + Windows)
- [x] `slam::common` data types (`ImageFrame`, `ImuMeasurement`, `LidarScan`)
- [x] KITTI odometry sequence reader (images, Velodyne scans, ground-truth poses) + unit tests
- [ ] Download a KITTI odometry sequence locally and run `slam_kitti_demo` against it

## Phase 1 — VIO front-end
- [x] Feature detection + KLT tracking across frames (`FeatureTracker`)
- [x] Stereo triangulation + PnP relative pose estimation (`stereo_geometry`) —
      used stereo (not monocular 5-point) so the recovered trajectory has
      real metric scale, no separate scale-recovery step needed
- [x] IMU preintegration between keyframes (`ImuPreintegrator`), algorithm
      implemented and unit-tested
- [x] Raw (unoptimized) VIO trajectory on a KITTI sequence (`slam_vio_demo`)

**IMU gap — closed (rotation only):** real KITTI IMU now flows end to end.
`KittiOxtsReader` (`slam/sensors/kitti_raw_imu.hpp`) reads a raw KITTI
drive's `oxts/` folder; `LookupRawDriveMapping` maps odometry sequences
00-10 to their source raw drive + start frame (the official devkit's
sequence-to-raw table, cross-checked against two independent mirrors of
`devkit/readme.txt`). `slam_backend_demo` wires it in when given a raw
dataset root as a third argument, feeding real body-frame accel/gyro
(`ax,ay,az`/`wx,wy,wz` — *not* the roll/pitch-leveled `af,al,au`/`wf,wl,wu`
fields, a different frame the devkit documents separately) into
`VioFrontend::ProcessImu` each frame.

The backend (`SlidingWindowOptimizer::AddKeyframe`) consumes the resulting
preintegrated delta as a genuine pose-graph edge — but **rotation only**.
Gyro integration over one KITTI frame interval (~0.1s) is trustworthy
without a velocity state or bias correction; the preintegrated *position*
is not (it assumes zero initial velocity and isn't gravity-compensated, both
of which require carrying velocity/bias as optimizable per-keyframe state —
real, larger scope). So the IMU edge's information matrix zeros out its
translation rows entirely (see `RotationOnlyInformation`), leveraging the
fact that `Sophus::SE3d::log()`'s rotation component is independent of
translation — a clean, honest partial fusion, not the tightly-coupled
full-state IMU factor that remains Phase 6 stretch scope.

KITTI's raw *synced* oxts data is one sample per image frame (not a
separate high-rate IMU stream), so this is a single preintegration step per
keyframe interval, not multi-sample high-rate preintegration.

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

**Design note:** `RemoveGround` is a standalone, tested utility, deliberately
*not* wired into the `ExtractFeatures` pipeline — the ground plane is itself
a strong planar-feature source for ICP (constrains z/roll/pitch), so
stripping it before feature extraction would throw away useful constraints.
It's meant for Phase 4 mapping (obstacle/free-space separation) instead.

## Phase 3 — Backend
- [x] Hand-written Gauss-Newton pose-graph optimization (`PoseGraph`) over
      the SE3 manifold, dense normal equations, no g2o/GTSAM/Ceres
- [x] Shared pose graph consuming both front-ends (`SlidingWindowOptimizer`)
      — each keyframe gets a VIO edge and/or a LiDAR edge, weighted by
      inlier/correspondence count, plus a rotation-only IMU edge when real
      IMU data is available (see Phase 1's IMU section), jointly optimized
- [x] Sliding window: keyframes older than the window are frozen
      (`PoseGraph::FixNode`) rather than dropped; `OptimizeGlobally()` does
      a one-shot full-graph re-solve (e.g. at the end of a sequence run)
- [x] Loop closure detection + pose-graph correction (`DetectLoopClosure`):
      geometric-proximity candidates verified by LiDAR scan registration
- [x] `slam_backend_demo`: runs VIO + LiDAR + backend together over a KITTI
      sequence

**Scope decisions made here (read before extending this phase):**
- **Pose-graph, not landmark bundle adjustment.** "Bundle adjustment" in the
  literature usually means jointly optimizing camera poses *and* 3D landmark
  positions via reprojection error, with landmarks marginalized out via a
  Schur complement. This phase optimizes keyframe poses only, constrained by
  each frontend's per-frame relative-pose estimate — a real, useful
  technique (this is what many lightweight SLAM backends do), but a smaller
  scope than full landmark BA. Landmark-level optimization is a candidate
  follow-up, not assumed here.
- **Frozen, not marginalized, window boundary.** Dropping a keyframe from
  the active window by just fixing its pose (`FixNode`) discards the
  information a proper marginalization would fold into a prior on its
  neighbor. This is simpler and still correct — it just means the sliding
  window is more conservative about revising old history than a fully
  marginalized backend would be.
- **Numerical, not analytic, edge Jacobians.** `PoseGraph::Solve()` uses
  central finite differences instead of hand-deriving the SE3 adjoint
  Jacobian of the edge residual. Both are legitimate from-scratch
  techniques; numerical differentiation was chosen specifically because
  there's no compiler in this environment to empirically catch a sign error
  in an adjoint derivation, and the cost is negligible at pose-graph scale.
- **Geometric loop closure, not appearance-based.** `DetectLoopClosure` uses
  trajectory proximity + LiDAR scan-matching verification, not a visual
  bag-of-words / Scan Context style place recognizer. A real appearance-based
  recognizer is a substantially larger undertaking, left for future work.

## Phase 4 — Mapping + visualization
- [x] Sparse landmark map (VIO) and voxel/point-cloud map (LiDAR)
      (`mapping::Map`) — both built on the same `PointCloudMap` incremental
      voxel accumulator at different granularity; see its doc comment for
      why that's one structure, not two
- [x] `slam_mapping_demo`: two-pass map build (collect per-frame local
      points, run `OptimizeGlobally()`, *then* transform+merge with final
      poses) exported to a standard ASCII PLY file — viewable in
      MeshLab/CloudCompare/Blender without any of this project's own code
- [x] Pangolin live trajectory + map viewer (`slam::viz::PangolinViewer`,
      `slam_viz_demo`) — **opt-in only** (`-DSLAM_BUILD_VIZ=ON` +
      `--preset with-viz` for the vcpkg `viz` feature), not part of the
      default build

**Read before trusting the viewer:** `PangolinViewer` is graphics/windowing
code, not math — everything else in this project was derivable and
reasoned through by hand (residual formulas, Jacobians, RANSAC geometry);
whether an OpenGL window actually renders correctly is not something that
can be verified the same way, and this project has no display to test
against. It follows the standard, extremely common Pangolin usage pattern
(the same structure ORB-SLAM2's viewer and most Pangolin tutorials use), so
it's a genuine attempt, not a stub — but build and run it yourself before
relying on it. It's gated behind `SLAM_BUILD_VIZ` (default off) and the
vcpkg `viz` feature specifically so it can never affect the default build
or CI, which do not exercise it. `slam_mapping_demo`'s PLY export has no
such caveat — it's plain, testable file I/O with unit tests covering it.

**LiDAR map density note:** `slam_mapping_demo`/`slam_viz_demo` build the
LiDAR map from `LidarFrontend::LastFeatures()` (the edge/planar points
already computed for odometry), not the full raw scan — a full KITTI
sequence's raw point clouds would be far too much to hold across an entire
run. A full-density map fed from raw scans is a reasonable enhancement, not
attempted here.

## Phase 5 — Evaluation
- [x] ATE scoring (`ComputeAte`) — RMSE after closed-form Kabsch/Horn rigid
      (no-scale) alignment, the standard literature metric
- [x] RPE scoring (`ComputeKittiOdometryError`) — KITTI's own official
      odometry evaluation protocol (100-800m ground-truth-path segments,
      avg translation % + rotation deg/100m), not a simplified stand-in,
      specifically so numbers are comparable to published results
- [x] `slam_eval_demo`: runs VIO-only, LiDAR-only, fused-incremental, and
      fused-globally-optimized trajectories over a sequence and prints all
      four metrics side by side
- [ ] **Comparison table vs. published LOAM/ORB-SLAM3 numbers in the
      README — not done, and not fakeable.** This project has never been
      build-verified (no compiler in the environment it was written in),
      so there are no real numbers to report yet. Filling this in requires
      an actual build + a real `slam_eval_demo` run against a downloaded
      sequence. Putting invented numbers here would be worse than an empty
      table. See README.md's Evaluation section for the template and how
      to populate it once you've run it.

## Phase 6 — Stretch: fusion + ROS2

Full design in [PHASE6_PLAN.md](PHASE6_PLAN.md) — read it before extending
this phase; the summary below tracks status against that plan's sub-phases,
it doesn't replace it.

- [x] **6A.1 — Full IMU factor.** `backend::NavState` (15-DOF: pose +
      velocity + gyro/accel bias), `backend::ImuPreintegration`
      (bias-aware preintegration; bias-estimate changes are handled by
      re-integrating the raw buffer via `BiasCorrected()`, not an analytic
      bias-Jacobian — see the class doc comment and PHASE6_PLAN.md §2.3
      for why), and `ComputeImuFactorResidual` (the 9-dim motion + 6-dim
      bias-random-walk residual). Unit-tested against hand-derived
      closed-form synthetic motion.
- [x] **6A.1 (solver) — `NavStateGraph`.** A separate 15-DOF Gauss-Newton
      solver (deliberately not unified with `PoseGraph` — PHASE6_PLAN.md
      §2.7) that fuses `NavPoseEdge` (VIO/LiDAR-style relative-pose
      constraints, placed on the pose sub-block) and `NavImuEdge` (the
      IMU factor above). `HighWeightPoseEdgeDominatesOverImuEdge` is the
      "does fusion actually change the outcome" test, same spirit as
      Phase 3's loop-closure test.
- [x] **6A.2 — Initialization.** `backend::InitializeVio`: gyro bias via a
      small numeric Gauss-Newton, then gravity + per-keyframe velocity via
      one linear least-squares solve — no scale unknown to solve for,
      unlike monocular VIO initialization, since this project's odometry
      is already metric. Recovers known synthetic gravity/velocity/bias to
      tight tolerance.
- [ ] **Not yet done: wiring into the actual pipeline.** `NavStateGraph`/
      `InitializeVio` exist and are tested as standalone components, but
      nothing in `SlidingWindowOptimizer` or the demo apps uses them yet —
      the existing pipeline still runs its original loosely-coupled
      fusion (`PoseGraph` + the rotation-only IMU edge from the Phase 1/3
      IMU-gap closure). Wiring a demo app to actually run tightly-coupled
      fusion end to end is the natural next increment, not yet started.
- [ ] 6A.3 — Tightly-coupled visual factors (not planned; see
      PHASE6_PLAN.md §2.5 for why this is recommended against)
- [ ] 6A.4 — Tightly-coupled LiDAR factors (not planned; see §2.6)
- [x] **Part B — ROS2 wrapping**, in `ros2_ws/src/slam_ros2/`: message
      adapters (`message_adapters.hpp/.cpp`) converting ROS2 messages
      to/from `slam_core`'s structs, including the optical→REP-103
      frame-convention fix (PHASE6_PLAN.md §3.5); `SlamNode` wrapping
      VIO/LiDAR/backend/mapping as a live node (buffered callbacks +
      dedicated processing thread, per §3.4); `slam_core` now has a real
      `install()`/CMake-package export so `slam_ros2` can `find_package`
      it without `slam_core` knowing ROS2 exists (§3.6).

  **Read before trusting this one.** Every other line of code in this
  project — including 6A's IMU math above — could be reasoned through and
  cross-checked by hand: residual formulas verified against their own
  zero conditions, Jacobians checked by re-deriving twice, a rotation
  matrix checked against its own defining property. ROS2's message field
  names and the exact API shapes of `rclcpp`/`message_filters`/`tf2_ros`/
  `cv_bridge` are not something derivable that way — they're either right
  or wrong by convention, and this was written from documented API
  knowledge with no way to check it against a real ROS2 install. Two
  tiers of confidence within Part B itself:
  - `message_adapters.*` — pure functions, unit-tested with hand-built
    messages (`test_message_adapters.cpp`), including a real correctness
    check of the frame-conversion matrix against its own defining
    property. Comparable confidence to the rest of this project.
  - `slam_node.*` — combines multiple ROS2 library surfaces into a
    running node. A genuine, complete attempt at PHASE6_PLAN.md §3.4's
    design, not a stub — but the least-verified file in this repository.
    Build and run it against a real ROS2 install before trusting it.
