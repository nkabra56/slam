#pragma once

#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "slam/common/types.hpp"
#include "slam/frontend_lidar/scan_features.hpp"
#include "slam/frontend_lidar/scan_matcher.hpp"

namespace slam::frontend_lidar {

// Frame-to-frame LiDAR odometry: extracts LOAM features per scan, then
// registers against the previous scan's features via scan_matcher.hpp.
class LidarFrontend {
 public:
  struct FrameResult {
    bool has_pose{false};
    Sophus::SE3d relative_pose;  // previous scan frame -> current scan frame
    int num_edge_correspondences{0};
    int num_planar_correspondences{0};
  };

  // `lidar_to_camera` moves extracted features into the camera frame that
  // VIO and ground truth use, so poses and LastFeatures() are in that frame.
  // Identity leaves everything in the LiDAR's own frame.
  explicit LidarFrontend(FeatureExtractionParams feature_params = {},
                          ScanMatcherParams matcher_params = {},
                          Eigen::Isometry3d lidar_to_camera = Eigen::Isometry3d::Identity());

  FrameResult ProcessScan(const LidarScan& scan);

  // Features from the most recently processed scan, post voxel-downsample.
  const ScanFeatures& LastFeatures() const { return previous_features_; }

 private:
  FeatureExtractionParams feature_params_;
  ScanMatcherParams matcher_params_;
  Eigen::Isometry3d lidar_to_camera_;
  ScanFeatures previous_features_;
  bool has_previous_scan_{false};
  // Constant-velocity seed for MatchScans's initial guess.
  Sophus::SE3d last_relative_pose_;
};

}  // namespace slam::frontend_lidar
