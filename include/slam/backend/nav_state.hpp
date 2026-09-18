#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace slam::backend {

// 15-DOF nav state: pose + velocity + gyro/accel bias. Pose retracts via
// the Lie group exponential, the rest by simple addition.
struct NavState {
  Sophus::SE3d pose;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_accel{Eigen::Vector3d::Zero()};
};

NavState Retract(const NavState& state, const Eigen::Matrix<double, 15, 1>& delta);

}  // namespace slam::backend
