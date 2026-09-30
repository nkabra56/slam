#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "slam/common/types.hpp"

namespace slam::sensors {

// Maps a KITTI odometry sequence (00-10) to its source raw-data drive and
// the frame range it was cut from, per the official odometry devkit.
struct KittiRawDriveMapping {
  std::string date;   // e.g. "2011_10_03"
  std::string drive;  // e.g. "2011_10_03_drive_0027"
  std::size_t start_frame{};
};

// Returns nullopt for sequences without a known raw-data mapping (only 00-10
// have one).
std::optional<KittiRawDriveMapping> LookupRawDriveMapping(const std::string& sequence);

// Composes `raw_root/<date>/calib_imu_to_velo.txt` (IMU -> Velodyne) with
// `lidar_to_camera` (calib.txt's `Tr`) into IMU -> camera 0. Throws if the
// calibration file is missing (it ships in KITTI's per-date raw calib zip).
Eigen::Isometry3d LoadImuToCamera(const std::filesystem::path& raw_root, const std::string& date,
                                   const Eigen::Isometry3d& lidar_to_camera);

// Reads IMU from oxts/. Uses body-frame ax,ay,az/wx,wy,wz, not the
// gravity-leveled af,al,au/wf,wl,wu fields. Measurements are rotated by `imu_to_camera`
// into the camera frame (lever arm ignored); pass Identity() explicitly in
// tests that don't exercise rotation -- there is no default.
class KittiOxtsReader {
 public:
  explicit KittiOxtsReader(const std::filesystem::path& oxts_dir, std::size_t start_frame,
                            Eigen::Isometry3d imu_to_camera);

  std::size_t NumMeasurements() const;
  ImuMeasurement MeasurementAt(std::size_t index) const;

 private:
  std::filesystem::path oxts_dir_;
  std::size_t start_frame_;
  Eigen::Matrix3d imu_to_camera_rotation_;
  std::vector<TimestampSec> timestamps_;
};

}  // namespace slam::sensors
