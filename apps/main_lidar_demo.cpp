#include <filesystem>
#include <iostream>
#include <optional>

#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/sensors/kitti_dataset.hpp"

// Runs the LiDAR frontend over a KITTI sequence's Velodyne scans and
// prints the raw (unoptimized) chained trajectory.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: slam_lidar_demo <sequence_dir> [poses_file]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 3) {
    poses_file = std::filesystem::path{argv[2]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    slam::frontend_lidar::LidarFrontend lidar({}, {}, reader.LoadLidarToCamera());

    Sophus::SE3d world_pose;  // identity at frame 0

    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      const slam::LidarScan scan = reader.LoadLidarScan(i);
      const auto result = lidar.ProcessScan(scan);
      if (result.has_pose) {
        world_pose = world_pose * result.relative_pose.inverse();
      }

      const Eigen::Vector3d t = world_pose.translation();
      std::cout << "frame " << i << ": edges=" << result.num_edge_correspondences
                << " planars=" << result.num_planar_correspondences << " pos=[" << t.x() << ", "
                << t.y() << ", " << t.z() << "]";
      if (reader.HasGroundTruth()) {
        const Eigen::Vector3d gt = reader.GroundTruthPoseAt(i).translation();
        std::cout << " gt=[" << gt.x() << ", " << gt.y() << ", " << gt.z() << "]";
      }
      std::cout << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
