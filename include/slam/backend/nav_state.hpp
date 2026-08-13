#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace slam::backend {

// 15-DOF navigation state: pose (SE3) + velocity + gyro/accel bias.
// Perturbation/retraction tangent order: (pose:6, velocity:3,
// bias_gyro:3, bias_accel:3). Pose retracts via the Lie group
// exponential; velocity and both biases are vector spaces and retract by
// simple addition. See PHASE6_PLAN.md section 2.2.
struct NavState {
  Sophus::SE3d pose;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_accel{Eigen::Vector3d::Zero()};
};

NavState Retract(const NavState& state, const Eigen::Matrix<double, 15, 1>& delta);

}  // namespace slam::backend
