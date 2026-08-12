# slam

A LiDAR + visual-inertial SLAM system built from scratch in C++, with Python
bindings, meant as the foundation for a larger robotics project rather than a
one-off demo. Front-ends and the backend optimizer are hand-implemented
instead of wrapping an existing SLAM framework. See [ROADMAP.md](ROADMAP.md)
for the phase-by-phase build plan and current status.

## Architecture

```
sensors/         timestamped data types + KITTI dataset reader
                 (this boundary becomes a ROS2 node boundary later on)
frontend_vio/    feature tracking + IMU preintegration -> raw VIO trajectory
frontend_lidar/  point-cloud features + scan matching  -> raw LiDAR odometry
backend/         hand-written sliding-window BA + pose-graph optimization,
                 shared by both frontends, with loop closure
mapping/         sparse landmark map (VIO) + voxel map (LiDAR), PLY export
viz/             Pangolin live trajectory/map viewer (opt-in, SLAM_BUILD_VIZ)
bindings/        pybind11 Python bindings over slam_core
eval/            ATE + KITTI-protocol RPE scoring against ground truth
```

Every module boundary is a plain data struct (`ImageFrame`, `ImuMeasurement`,
`LidarScan`, ...) defined in `include/slam/common/types.hpp`, and each stage
is configured rather than hardcoded, so a stage can later be swapped for a
live sensor or a ROS2 node without changing the algorithm code around it.

## Dataset

[KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) —
free for research/non-commercial use, and the only common dataset with
synchronized stereo camera + Velodyne LiDAR + ground-truth poses in the same
sequences, so both front-ends develop and evaluate against one dataset.

Download the *grayscale*, *velodyne laser data*, and *ground truth poses*
downloads from the KITTI site, then lay them out as:

```
data/
  sequences/
    00/
      image_0/
      image_1/
      velodyne/
      times.txt
    ...
  poses/
    00.txt
    ...
```

`data/` is gitignored — nothing under it is committed to the repo.

### IMU (optional, sequences 00-10 only)

The odometry benchmark download above has no IMU. Real IMU comes from the
separate [KITTI raw data](https://www.cvlibs.net/datasets/kitti/raw_data.php)
download's `oxts/` folder. Download the *[synced+rectified data]* for the
raw drive corresponding to the odometry sequence you want (see
`LookupRawDriveMapping` in `slam/sensors/kitti_raw_imu.hpp` for the
sequence-to-drive table), and lay it out as:

```
data/
  raw/
    2011_10_03/
      2011_10_03_drive_0027_sync/
        oxts/
          data/
          timestamps.txt
    ...
```

Pass that `raw/` root as `slam_backend_demo`'s third argument to enable it.

## Building

Requires a [vcpkg](https://vcpkg.io) install with the `VCPKG_ROOT` environment
variable set.

```powershell
cmake --preset default
cmake --build build
ctest --test-dir build
```

Run the KITTI smoke test against a downloaded sequence:

```bash
./build/apps/slam_kitti_demo data/sequences/00 data/poses/00.txt
```

Run the Phase 1 stereo-VIO frontend and print its raw trajectory:

```bash
./build/apps/slam_vio_demo data/sequences/00 data/poses/00.txt
```

Run the Phase 2 LiDAR frontend and print its raw trajectory:

```bash
./build/apps/slam_lidar_demo data/sequences/00 data/poses/00.txt
```

Run both frontends fused through the Phase 3 backend (prints the
per-frame trajectory, then a final loop-closure-corrected global pass).
The third argument (a raw KITTI dataset root) is optional and enables
real IMU rotation fusion — see "IMU" above:

```bash
./build/apps/slam_backend_demo data/sequences/00 data/poses/00.txt [data/raw]
```

Build the final map (VIO landmarks + LiDAR features, using the
loop-closure-corrected global poses) and export it to PLY, viewable in
MeshLab/CloudCompare/Blender:

```bash
./build/apps/slam_mapping_demo data/sequences/00 map.ply data/poses/00.txt
```

The Pangolin live viewer is opt-in — build it with the `with-viz` preset
(requires OpenGL/X11 dev libraries on Linux, e.g. `libgl1-mesa-dev`):

```powershell
cmake --preset with-viz
cmake --build build
```
```bash
./build/apps/slam_viz_demo data/sequences/00 data/poses/00.txt
```

## Evaluation

`slam_eval_demo` runs all four trajectory variants (VIO-only, LiDAR-only,
backend-fused incrementally, backend-fused after a final global
loop-closure-corrected optimization) over one sequence and reports, for
each: ATE RMSE (Sturm et al.'s standard rigid-alignment metric) and the
*official* KITTI odometry protocol's average translation error (%) and
rotation error (deg/100m) over 100-800m ground-truth path segments —
deliberately the same protocol papers report, not a simplified stand-in,
so the numbers are actually comparable to published results:

```bash
./build/apps/slam_eval_demo data/sequences/00 data/poses/00.txt
```

**This table is a template, not a result** — this project has never been
build-verified (no compiler in the environment it was written in), so
there are no real numbers to publish yet. Run the command above on a
downloaded sequence and fill in the top rows yourself; compare against
whatever published numbers you look up for that same sequence from the
[KITTI odometry leaderboard](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
or the LOAM/ORB-SLAM3 papers directly — don't trust a pasted-in number here
that wasn't actually measured.

| Method                     | ATE (m) | Trans. error (%) | Rot. error (deg/100m) |
|----------------------------|---------|-------------------|------------------------|
| VIO-only                   | —       | —                 | —                      |
| LiDAR-only                 | —       | —                 | —                      |
| Fused (incremental)        | —       | —                 | —                      |
| Fused (global opt.)        | —       | —                 | —                      |
| LOAM (published)           | —       | —                 | —                      |
| ORB-SLAM3 (published)      | —       | —                 | —                      |

## Status

Phases 0-5 are complete: build/CI setup, core data types, a KITTI sequence
reader (images/LiDAR/calibration/ground truth), a stereo visual-inertial
frontend (KLT tracking, stereo triangulation, PnP relative pose, IMU
preintegration wired to real KITTI IMU data), a LiDAR frontend (LOAM-style
edge/planar feature extraction, hand-written k-d tree, point-to-line/
point-to-plane Gauss-Newton scan matching), a backend (hand-written SE3
pose-graph Gauss-Newton optimizer fusing VIO, LiDAR, and IMU-rotation edges
in a sliding window, plus geometric loop closure), mapping (a
voxel-accumulated point-cloud map exported to PLY, plus an opt-in Pangolin
live viewer), and evaluation tooling (ATE + the official KITTI RPE
protocol) — though its comparison table is still an empty template, since
filling it in honestly requires a real build. See [ROADMAP.md](ROADMAP.md)
for the scope decisions behind each phase's design and what's next (Phase
6, stretch: tightly-coupled fusion + ROS2).

## License

MIT — see [LICENSE](LICENSE).
