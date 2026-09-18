#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "slam/common/types.hpp"

namespace slam::sensors {

// Reads a KITTI odometry sequence directory (e.g. data/sequences/00) with an
// optional ground-truth poses file. Stereo + Velodyne only; see kitti_raw_imu.hpp for IMU.
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

  // Reads calib.txt (P0/P1 projection matrices) for the left/right
  // grayscale pair used by LoadImage(index, 0)/LoadImage(index, 1).
  StereoCalibration LoadCalibration() const;

  // Reads calib.txt's `Tr:` line: Velodyne points -> camera 0 (the frame the
  // ground-truth poses are in). Throws if absent -- the gray-images archive's
  // calib.txt has only P0-P3; `Tr` is in the separate data_odometry_calib.zip.
  Eigen::Isometry3d LoadLidarToCamera() const;

 private:
  std::filesystem::path sequence_dir_;
  std::vector<TimestampSec> timestamps_;
  std::vector<Eigen::Isometry3d> ground_truth_poses_;
};

}  // namespace slam::sensors
