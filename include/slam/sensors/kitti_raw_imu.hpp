#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "slam/common/types.hpp"

namespace slam::sensors {

// Maps a KITTI odometry sequence (00-10, the only ones with ground truth)
// to its source raw-data drive and the frame range the sequence was cut
// from within that drive. Sourced from the official odometry devkit's
// sequence-to-raw mapping table (cross-checked against two independent
// mirrors of devkit/readme.txt) -- this correspondence is a fixed,
// published dataset artifact, not something that changes.
struct KittiRawDriveMapping {
  std::string date;   // e.g. "2011_10_03"
  std::string drive;  // e.g. "2011_10_03_drive_0027"
  std::size_t start_frame{};
};

// Returns std::nullopt for sequences without a known raw-data mapping
// (only 00-10 do; KITTI's odometry test sequences 11-21 have no published
// ground truth or raw-data correspondence).
std::optional<KittiRawDriveMapping> LookupRawDriveMapping(const std::string& sequence);

// Reads IMU measurements from a raw KITTI drive's oxts/ folder, one
// measurement per synced frame -- KITTI's "synced+rectified" raw drives
// have exactly one oxts sample per image/Velodyne frame, aligned by index.
//
// Uses the body-frame accel/gyro fields (ax,ay,az / wx,wy,wz: "in
// direction of vehicle's x/y/z axis", i.e. front/left/top of the vehicle
// chassis) rather than af,al,au / wf,wl,wu, which the OXTS devkit
// documents as roll/pitch-*leveled* forward/left/up axes -- a different,
// gravity-referenced frame that would violate ImuPreintegrator's rigid
// body-frame model.
//
// `oxts_dir` is the oxts/ folder inside a separately downloaded KITTI raw
// data tree, e.g. .../2011_10_03/2011_10_03_drive_0027_sync/oxts/. This is
// a different download from the odometry benchmark data -- see README.md.
// `start_frame` (from LookupRawDriveMapping) is the offset into that raw
// drive where the odometry sequence begins; index 0 here means that frame,
// and timestamps are zeroed there too, matching times.txt's convention of
// starting at 0 for the odometry sequence's first frame.
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
