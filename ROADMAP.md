# Roadmap

Foundation project for LiDAR + visual-inertial SLAM, built module by module so
each phase leaves a runnable, testable increment. See [README.md](README.md)
for architecture and build instructions.

**Optimizer policy:** the Phase 3 backend (sliding-window bundle adjustment,
pose-graph optimization) is hand-written using Eigen/Sophus — no GTSAM, g2o,
or Ceres fallback. If it's numerically unstable, the fix is better math
(correct Jacobians, robust kernels, damping/trust-region tuning), not
swapping in a library.

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
- [ ] Tightly-coupled LiDAR-VIO fusion in the backend
- [ ] Wrap `slam::sensors` as ROS2 nodes for live/bag playback instead of the KITTI reader
