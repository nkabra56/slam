#include "slam/backend/nav_state.hpp"

#include <gtest/gtest.h>

namespace slam::backend {
namespace {

TEST(Retract, AppliesZeroDeltaAsIdentity) {
  NavState state;
  state.velocity = Eigen::Vector3d(1, 2, 3);
  const NavState result = Retract(state, Eigen::Matrix<double, 15, 1>::Zero());
  EXPECT_TRUE(result.pose.translation().isApprox(state.pose.translation()));
  EXPECT_TRUE(result.velocity.isApprox(state.velocity));
}

TEST(Retract, UpdatesEachBlockIndependently) {
  const NavState state;
  Eigen::Matrix<double, 15, 1> delta = Eigen::Matrix<double, 15, 1>::Zero();
  delta.segment<3>(0) = Eigen::Vector3d(1.0, 0.0, 0.0);   // pose translation part
  delta.segment<3>(6) = Eigen::Vector3d(0.0, 2.0, 0.0);   // velocity
  delta.segment<3>(9) = Eigen::Vector3d(0.0, 0.0, 0.01);  // gyro bias
  delta.segment<3>(12) = Eigen::Vector3d(0.02, 0.0, 0.0);  // accel bias

  const NavState result = Retract(state, delta);
  EXPECT_NEAR(result.pose.translation().x(), 1.0, 1e-9);
  EXPECT_NEAR(result.velocity.y(), 2.0, 1e-9);
  EXPECT_NEAR(result.bias_gyro.z(), 0.01, 1e-9);
  EXPECT_NEAR(result.bias_accel.x(), 0.02, 1e-9);
}

}  // namespace
}  // namespace slam::backend
