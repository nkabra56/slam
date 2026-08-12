#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "slam/common/types.hpp"

namespace slam::sensors {

// Reads a single KITTI odometry sequence directory, e.g. data/sequences/00,
// with an optional ground-truth poses file (data/poses/00.txt) available for
// sequences 00-10. Grayscale stereo (image_0/image_1) and Velodyne
// (velodyne/) are supported; IMU/GPS (oxts/) parsing lands in a later phase
// alongside the VIO frontend.
class KittiSequenceReader {
 public:
  explicit KittiSequenceReader(
      const std::filesystem::path& sequence_dir,
      std::optional<std::filesystem::path> poses_file = std::nullopt);

  std::size_t NumFrames() const;
  TimestampSec TimestampAt(std::size_t index) const;

  // camera 0 = left grayscale, 1 = right grayscale.
  ImageFrame LoadImage(std::size_t index, int camera = 0) const;
  LidarScan LoadLidarScan(std::size_t index) const;

  bool HasGroundTruth() const;
  Eigen::Isometry3d GroundTruthPoseAt(std::size_t index) const;

 private:
  std::filesystem::path sequence_dir_;
  std::vector<TimestampSec> timestamps_;
  std::vector<Eigen::Isometry3d> ground_truth_poses_;
};

}  // namespace slam::sensors
