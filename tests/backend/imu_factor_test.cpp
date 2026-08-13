#include "slam/backend/imu_factor.hpp"

#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

TEST(ImuPreintegration, BiasCorrectedMatchesDirectIntegrationWithSameBias) {
  std::vector<ImuMeasurement> measurements(3);
  measurements[0].timestamp = 0.0;
  measurements[0].linear_acceleration = Eigen::Vector3d(0.3, -0.1, 0.2);
  measurements[0].angular_velocity = Eigen::Vector3d(0.05, 0.02, -0.03);
  measurements[1].timestamp = 0.1;
  measurements[1].linear_acceleration = Eigen::Vector3d(0.25, -0.05, 0.15);
  measurements[1].angular_velocity = Eigen::Vector3d(0.04, 0.01, -0.02);
  measurements[2].timestamp = 0.2;

  const ImuBias bias{Eigen::Vector3d(0.01, -0.01, 0.005), Eigen::Vector3d(0.02, 0.0, -0.01)};

  ImuPreintegration direct(bias);
  for (const auto& m : measurements) direct.Integrate(m);

  ImuPreintegration zero_bias;
  for (const auto& m : measurements) zero_bias.Integrate(m);
  const ImuPreintegration via_correction = zero_bias.BiasCorrected(bias);

  EXPECT_TRUE(
      direct.DeltaRotation().matrix().isApprox(via_correction.DeltaRotation().matrix(), 1e-12));
  EXPECT_TRUE(direct.DeltaVelocity().isApprox(via_correction.DeltaVelocity(), 1e-12));
  EXPECT_TRUE(direct.DeltaPosition().isApprox(via_correction.DeltaPosition(), 1e-12));
  EXPECT_NEAR(direct.DeltaTime(), via_correction.DeltaTime(), 1e-12);
}

TEST(ImuPreintegration, ZeroBiasMatchesFrontendVioPreintegratorFormula) {
  // Sanity check that this class's recursion is the same one
  // frontend_vio::ImuPreintegrator uses (just parameterized by a bias),
  // via the same closed-form single-step check that class's own tests use.
  ImuPreintegration preintegration;
  ImuMeasurement first;
  first.timestamp = 0.0;
  first.linear_acceleration = Eigen::Vector3d(1.0, 0.0, 0.0);
  preintegration.Integrate(first);
  ImuMeasurement second;
  second.timestamp = 1.0;
  preintegration.Integrate(second);

  EXPECT_NEAR(preintegration.DeltaTime(), 1.0, 1e-9);
  EXPECT_NEAR(preintegration.DeltaVelocity().x(), 1.0, 1e-9);
  EXPECT_NEAR(preintegration.DeltaPosition().x(), 0.5, 1e-9);
}

// Builds a NavState pair (state_i, state_j) that exactly satisfies the IMU
// factor's noise-free kinematics for a given synthetic preintegration and
// gravity -- i.e. the inverse of ComputeImuFactorResidual's formulas.
struct SyntheticImuSegment {
  ImuPreintegration preintegration;
  NavState state_i;
  NavState state_j;
  Eigen::Vector3d gravity;
};

SyntheticImuSegment MakeSyntheticImuSegment() {
  SyntheticImuSegment segment;
  segment.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  ImuMeasurement m1;
  m1.timestamp = 0.0;
  m1.linear_acceleration = Eigen::Vector3d(0.8, -0.3, 0.1);
  m1.angular_velocity = Eigen::Vector3d(0.0, 0.0, 0.2);
  segment.preintegration.Integrate(m1);
  ImuMeasurement m2;
  m2.timestamp = 0.1;
  segment.preintegration.Integrate(m2);

  segment.state_i.pose =
      Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
  segment.state_i.velocity = Eigen::Vector3d(0.3, 0.0, 0.0);

  const double dt = segment.preintegration.DeltaTime();
  const Eigen::Matrix3d R_i = segment.state_i.pose.rotationMatrix();

  segment.state_j.pose = Sophus::SE3d(
      segment.state_i.pose.so3() * segment.preintegration.DeltaRotation(),
      segment.state_i.pose.translation() + segment.state_i.velocity * dt +
          0.5 * segment.gravity * dt * dt + R_i * segment.preintegration.DeltaPosition());
  segment.state_j.velocity =
      segment.state_i.velocity + segment.gravity * dt + R_i * segment.preintegration.DeltaVelocity();

  return segment;
}

TEST(ComputeImuFactorResidual, IsZeroForConsistentSyntheticMotion) {
  const SyntheticImuSegment segment = MakeSyntheticImuSegment();

  const auto residual =
      ComputeImuFactorResidual(segment.state_i, segment.state_j, segment.preintegration, segment.gravity);

  EXPECT_LT(residual.motion.norm(), 1e-9);
  EXPECT_LT(residual.bias_random_walk.norm(), 1e-9);
}

TEST(ComputeImuFactorResidual, IsNonZeroWhenStatesAreInconsistent) {
  const SyntheticImuSegment segment = MakeSyntheticImuSegment();

  NavState perturbed_j = segment.state_j;
  perturbed_j.pose = Sophus::SE3d(perturbed_j.pose.so3(),
                                   perturbed_j.pose.translation() + Eigen::Vector3d(1.0, 0.0, 0.0));

  const auto residual =
      ComputeImuFactorResidual(segment.state_i, perturbed_j, segment.preintegration, segment.gravity);

  EXPECT_GT(residual.motion.norm(), 0.5);
}

TEST(ComputeImuFactorResidual, BiasRandomWalkReflectsBiasDifference) {
  NavState state_i;
  NavState state_j;
  state_j.bias_gyro = Eigen::Vector3d(0.01, 0.0, 0.0);
  state_j.bias_accel = Eigen::Vector3d(0.0, 0.02, 0.0);

  ImuPreintegration empty;
  const auto residual = ComputeImuFactorResidual(state_i, state_j, empty, Eigen::Vector3d(0, 0, -9.81));

  EXPECT_NEAR(residual.bias_random_walk(0), 0.01, 1e-12);
  EXPECT_NEAR(residual.bias_random_walk(4), 0.02, 1e-12);
}

}  // namespace
}  // namespace slam::backend
