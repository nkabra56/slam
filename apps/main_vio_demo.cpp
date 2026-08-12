#include <filesystem>
#include <iostream>
#include <optional>

#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/sensors/kitti_dataset.hpp"

// Runs the Phase 1 stereo-VIO frontend over a KITTI sequence and prints the
// raw (unoptimized) chained trajectory. IMU preintegration deltas are
// computed but not yet fused into the pose -- see ROADMAP.md Phase 3.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: slam_vio_demo <sequence_dir> [poses_file]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 3) {
    poses_file = std::filesystem::path{argv[2]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    const slam::StereoCalibration calibration = reader.LoadCalibration();
    slam::frontend_vio::VioFrontend vio(calibration);

    Sophus::SE3d world_pose;  // identity at frame 0

    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      slam::StereoFrame frame;
      frame.timestamp = reader.TimestampAt(i);
      frame.left = reader.LoadImage(i, 0).image;
      frame.right = reader.LoadImage(i, 1).image;

      const auto result = vio.ProcessStereoFrame(frame);
      if (result.has_pose) {
        // result.relative_pose maps the previous camera frame into the
        // current one, so chain by its inverse to accumulate world pose.
        world_pose = world_pose * result.relative_pose.inverse();
      }

      const Eigen::Vector3d t = world_pose.translation();
      std::cout << "frame " << i << ": inliers=" << result.num_inliers << " pos=[" << t.x() << ", "
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
