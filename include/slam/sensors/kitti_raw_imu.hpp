#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

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

// Reads IMU from oxts/. Uses body-frame ax,ay,az/wx,wy,wz, not the
// gravity-leveled af,al,au/wf,wl,wu fields.
class KittiOxtsReader {
 public:
  explicit KittiOxtsReader(const std::filesystem::path& oxts_dir, std::size_t start_frame = 0);

  std::size_t NumMeasurements() const;
  ImuMeasurement MeasurementAt(std::size_t index) const;

 private:
  std::filesystem::path oxts_dir_;
  std::size_t start_frame_;
  std::vector<TimestampSec> timestamps_;
};

}  // namespace slam::sensors
