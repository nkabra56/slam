#pragma once

#include <sophus/se3.hpp>

#include "slam/common/types.hpp"
#include "slam/frontend_lidar/scan_features.hpp"
#include "slam/frontend_lidar/scan_matcher.hpp"

namespace slam::frontend_lidar {

// Frame-to-frame LiDAR odometry: extracts LOAM-style edge/planar features
// from each scan (ExtractFeatures), lightly voxel-downsamples them, and
// registers against the previous scan's features with the hand-written
// point-to-line/point-to-plane Gauss-Newton solve in scan_matcher.hpp.
class LidarFrontend {
 public:
  struct FrameResult {
    bool has_pose{false};
    Sophus::SE3d relative_pose;  // previous scan frame -> current scan frame
    int num_edge_correspondences{0};
    int num_planar_correspondences{0};
  };

  explicit LidarFrontend(FeatureExtractionParams feature_params = {},
                          ScanMatcherParams matcher_params = {});

  FrameResult ProcessScan(const LidarScan& scan);

  // Features extracted from the most recently processed scan (post
  // voxel-downsample) -- exposed so callers (e.g. the backend's loop
  // closure) can reuse them without re-running feature extraction.
  const ScanFeatures& LastFeatures() const { return previous_features_; }

 private:
  FeatureExtractionParams feature_params_;
  ScanMatcherParams matcher_params_;
  ScanFeatures previous_features_;
  bool has_previous_scan_{false};
};

}  // namespace slam::frontend_lidar
