#include "slam/backend/nav_state.hpp"

namespace slam::backend {

NavState Retract(const NavState& state, const Eigen::Matrix<double, 15, 1>& delta) {
  NavState result = state;
  result.pose = state.pose * Sophus::SE3d::exp(delta.segment<6>(0));
  result.velocity = state.velocity + delta.segment<3>(6);
  result.bias_gyro = state.bias_gyro + delta.segment<3>(9);
  result.bias_accel = state.bias_accel + delta.segment<3>(12);
  return result;
}

}  // namespace slam::backend
