#pragma once

#include <cstddef>
#include <vector>

#include <sophus/se3.hpp>

namespace slam::eval {

// RMSE translation error after best-fit rigid (SE3, no scale) Kabsch/Horn
// alignment. Requires equal-length inputs and >= 3 poses; throws otherwise.
struct AbsoluteTrajectoryError {
  double rmse_m{};
  Sophus::SE3d alignment;  // the SE3 that best maps estimated -> ground_truth
};

AbsoluteTrajectoryError ComputeAte(const std::vector<Sophus::SE3d>& estimated,
                                    const std::vector<Sophus::SE3d>& ground_truth);

// KITTI's odometry protocol: avg translation/rotation error over
// {100,...,800}m segments, no alignment. 0 segments if too short.
struct KittiOdometryError {
  double avg_translation_error_percent{};
  double avg_rotation_error_deg_per_100m{};
  std::size_t num_segments_evaluated{};
};

KittiOdometryError ComputeKittiOdometryError(const std::vector<Sophus::SE3d>& estimated,
                                              const std::vector<Sophus::SE3d>& ground_truth);

}  // namespace slam::eval
