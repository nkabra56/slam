#include "slam/frontend_vio/imu_preintegrator.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::frontend_vio {
namespace {

constexpr double kQuarterTurn = 1.5707963267948966;  // pi/2

TEST(ImuPreintegrator, IntegratesConstantAcceleration) {
  ImuPreintegrator preintegrator;

  ImuMeasurement first;
  first.timestamp = 0.0;
  first.linear_acceleration = Eigen::Vector3d(1.0, 0.0, 0.0);
  preintegrator.Integrate(first);

  ImuMeasurement second;
  second.timestamp = 1.0;
  preintegrator.Integrate(second);

  const auto& result = preintegrator.result();
  EXPECT_NEAR(result.delta_time, 1.0, 1e-9);
  EXPECT_NEAR(result.delta_velocity.x(), 1.0, 1e-9);
  EXPECT_NEAR(result.delta_position.x(), 0.5, 1e-9);
  EXPECT_TRUE(result.delta_rotation.matrix().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(ImuPreintegrator, IntegratesConstantAngularVelocity) {
  ImuPreintegrator preintegrator;

  ImuMeasurement first;
  first.timestamp = 0.0;
  first.angular_velocity = Eigen::Vector3d(0.0, 0.0, kQuarterTurn);
  preintegrator.Integrate(first);

  ImuMeasurement second;
  second.timestamp = 1.0;
  preintegrator.Integrate(second);

  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(kQuarterTurn, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(preintegrator.result().delta_rotation.matrix().isApprox(expected, 1e-9));
}

TEST(ImuPreintegrator, ResetClearsAccumulatedState) {
  ImuPreintegrator preintegrator;

  ImuMeasurement m;
  m.timestamp = 0.0;
  m.linear_acceleration = Eigen::Vector3d(2.0, 0.0, 0.0);
  preintegrator.Integrate(m);
  m.timestamp = 1.0;
  preintegrator.Integrate(m);

  preintegrator.Reset();

  const auto& result = preintegrator.result();
  EXPECT_EQ(result.delta_time, 0.0);
  EXPECT_TRUE(result.delta_velocity.isZero());
  EXPECT_TRUE(result.delta_position.isZero());
}

}  // namespace
}  // namespace slam::frontend_vio
