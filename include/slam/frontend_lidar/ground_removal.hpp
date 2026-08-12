#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>

namespace slam::frontend_lidar {

struct GroundRemovalParams {
  int max_iterations = 200;
  double distance_threshold_m = 0.2;
  double min_inlier_ratio = 0.15;
  // Reject planes whose normal isn't roughly vertical in the sensor frame
  // (Velodyne mounted level on the vehicle roof for KITTI).
  double max_normal_tilt_deg = 30.0;
};

struct GroundRemovalResult {
  Eigen::Vector4d plane{Eigen::Vector4d::Zero()};  // a*x+b*y+c*z+d=0, (a,b,c) unit normal
  std::vector<Eigen::Vector3d> ground_points;
  std::vector<Eigen::Vector3d> non_ground_points;
};

// RANSAC plane fit + segmentation, from scratch (no PCL). Returns
// std::nullopt if no plane clears min_inlier_ratio within max_iterations,
// or the best plane found isn't roughly horizontal.
//
// Note: this is a standalone utility, not wired into the LOAM feature
// pipeline (scan_features.hpp) -- the ground plane is itself a good source
// of planar features for scan matching, so removing it there would throw
// away useful constraints. It's meant for later use in Phase 4 mapping
// (obstacle/free-space separation).
std::optional<GroundRemovalResult> RemoveGround(const std::vector<Eigen::Vector3d>& points,
                                                 const GroundRemovalParams& params = {});

}  // namespace slam::frontend_lidar
