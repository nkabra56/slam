#include "slam/backend/vio_initializer.hpp"

#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

// Builds a short synthetic trajectory with KNOWN gravity, per-keyframe
// velocity, and gyro bias: generates ground-truth poses from the TRUE
// (unbiased) IMU motion, and feeds InitializeVio only the BIASED
// ("measured") preintegrations a real sensor would have produced --
// mirroring exactly what a real bias-corrupted IMU vs. bias-free
// ground-truth trajectory relationship looks like.
TEST(InitializeVio, RecoversKnownGravityVelocityAndGyroBias) {
  const Eigen::Vector3d true_gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d true_gyro_bias(0.02, -0.01, 0.015);

  const int n = 8;  // keyframes
  const double dt = 0.1;

  std::vector<Sophus::SE3d> poses(n);
  std::vector<Eigen::Vector3d> true_velocities(n);
  poses[0] = Sophus::SE3d();
  true_velocities[0] = Eigen::Vector3d(1.0, 0.2, 0.0);

  std::vector<ImuPreintegration> preintegrations;
  preintegrations.reserve(n - 1);

  for (int i = 0; i < n - 1; ++i) {
    const Eigen::Vector3d true_gyro(0.0, 0.0, 0.3);
    const Eigen::Vector3d true_accel(0.5, -0.2, 9.81 * 0.02);  // mild body-frame accel

    // "Measured" (biased) preintegration -- the only thing InitializeVio
    // ever sees.
    ImuPreintegration measured;
    ImuMeasurement m1;
    m1.timestamp = 0.0;
    m1.angular_velocity = true_gyro + true_gyro_bias;
    m1.linear_acceleration = true_accel;  // accel bias assumed zero, as InitializeVio itself assumes
    measured.Integrate(m1);
    ImuMeasurement m2;
    m2.timestamp = dt;
    measured.Integrate(m2);
    preintegrations.push_back(measured);

    // Ground truth from the TRUE (unbiased) motion, exactly satisfying
    // the IMU kinematics with the known gravity -- same closed-form
    // construction as imu_factor_test.cpp's SyntheticImuSegment.
    ImuPreintegration truth;
    ImuMeasurement t1 = m1;
    t1.angular_velocity = true_gyro;
    truth.Integrate(t1);
    truth.Integrate(m2);

    const Eigen::Matrix3d R_i = poses[i].rotationMatrix();
    poses[i + 1] = Sophus::SE3d(
        poses[i].so3() * truth.DeltaRotation(),
        poses[i].translation() + true_velocities[i] * dt + 0.5 * true_gravity * dt * dt +
            R_i * truth.DeltaPosition());
    true_velocities[i + 1] = true_velocities[i] + true_gravity * dt + R_i * truth.DeltaVelocity();
  }

  const auto result = InitializeVio(poses, preintegrations);
  ASSERT_TRUE(result.has_value());

  EXPECT_NEAR((result->gravity - true_gravity).norm(), 0.0, 1e-3);
  EXPECT_NEAR((result->bias_gyro - true_gyro_bias).norm(), 0.0, 1e-3);
  ASSERT_EQ(result->velocities.size(), static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    EXPECT_NEAR((result->velocities[i] - true_velocities[i]).norm(), 0.0, 1e-2);
  }
}

TEST(InitializeVio, ReturnsNulloptWithTooFewPoses) {
  const std::vector<Sophus::SE3d> poses(2);
  const std::vector<ImuPreintegration> preintegrations(1);
  EXPECT_FALSE(InitializeVio(poses, preintegrations).has_value());
}

TEST(InitializeVio, ReturnsNulloptOnSizeMismatch) {
  const std::vector<Sophus::SE3d> poses(4);
  const std::vector<ImuPreintegration> preintegrations(1);  // should be 3
  EXPECT_FALSE(InitializeVio(poses, preintegrations).has_value());
}

}  // namespace
}  // namespace slam::backend
