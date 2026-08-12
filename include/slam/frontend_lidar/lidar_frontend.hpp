#pragma once

#include "slam/common/types.hpp"

namespace slam::frontend_lidar {

// Extracts edge/planar features from raw point clouds and matches
// consecutive scans to produce frame-to-frame pose estimates.
//
// TODO(phase-2): point-cloud downsampling/ground filtering, LOAM-style
// edge/planar feature extraction, and scan-to-scan ICP.
class LidarFrontend {
 public:
  void ProcessScan(const LidarScan& scan);
};

}  // namespace slam::frontend_lidar
