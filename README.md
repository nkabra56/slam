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
mapping/         sparse landmark map (VIO) + voxel map (LiDAR)
viz/             Pangolin live trajectory/map viewer
bindings/        pybind11 Python bindings over slam_core
eval/            ATE/RPE scoring against KITTI ground truth
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

## Status

Phase 0 and Phase 1 are complete: build/CI setup, core data types, a KITTI
sequence reader (images/LiDAR/calibration/ground truth), and a stereo
visual-inertial frontend (KLT feature tracking, stereo triangulation, PnP
relative pose, IMU preintegration) producing a raw chained trajectory. See
[ROADMAP.md](ROADMAP.md) for the known IMU-data-wiring gap and what's next.

## License

MIT — see [LICENSE](LICENSE).
