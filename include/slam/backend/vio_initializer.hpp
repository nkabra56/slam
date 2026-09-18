#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/backend/imu_factor.hpp"

namespace slam::backend {

struct VioInitializationResult {
  Eigen::Vector3d gravity{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> velocities;  // one per input pose, same order/count
  Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
  // Left at zero -- poorly observable without deliberate excitation; the
  // online optimizer refines it.
};

// Recovers gravity, gyro bias, and per-keyframe velocity from a pose/IMU
// window. Returns nullopt for too few poses, a size mismatch, or implausible gravity.
std::optional<VioInitializationResult> InitializeVio(
    const std::vector<Sophus::SE3d>& poses, const std::vector<ImuPreintegration>& preintegrations);

}  // namespace slam::backend
