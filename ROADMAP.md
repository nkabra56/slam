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
- [ ] Feature detection + KLT tracking across frames
- [ ] 5-point / PnP relative pose estimation
- [ ] IMU preintegration between keyframes
- [ ] Raw (unoptimized) VIO trajectory on a KITTI sequence

## Phase 2 — LiDAR front-end
- [ ] Point-cloud downsampling / ground filtering
- [ ] Edge and planar feature extraction (LOAM-style)
- [ ] Scan-to-scan ICP
- [ ] Raw LiDAR odometry trajectory on the same sequence

## Phase 3 — Backend
- [ ] Hand-written sliding-window bundle adjustment (Gauss-Newton / Levenberg-Marquardt)
- [ ] Shared pose-graph optimization consuming both front-ends
- [ ] Loop closure detection (place recognition) and pose-graph correction

## Phase 4 — Mapping + visualization
- [ ] Sparse landmark map (VIO)
- [ ] Voxel / point-cloud map (LiDAR)
- [ ] Pangolin live trajectory + map viewer

## Phase 5 — Evaluation
- [ ] ATE / RPE scoring against KITTI ground truth
- [ ] Comparison table vs. published LOAM / ORB-SLAM3 numbers in the README

## Phase 6 — Stretch: fusion + ROS2
- [ ] Tightly-coupled LiDAR-VIO fusion in the backend
- [ ] Wrap `slam::sensors` as ROS2 nodes for live/bag playback instead of the KITTI reader
