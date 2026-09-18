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

// RANSAC plane fit + segmentation. Returns nullopt if no plane clears
// min_inlier_ratio or is roughly horizontal.
std::optional<GroundRemovalResult> RemoveGround(const std::vector<Eigen::Vector3d>& points,
                                                 const GroundRemovalParams& params = {});

}  // namespace slam::frontend_lidar
