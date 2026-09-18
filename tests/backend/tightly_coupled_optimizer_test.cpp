#include "slam/backend/tightly_coupled_optimizer.hpp"

#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

// Builds one keyframe transition's ground truth and the BIASED raw IMU
// samples a real sensor would report for it.
struct SyntheticStep {
  Sophus::SE3d pose_j;
  Eigen::Vector3d velocity_j;
  ImuPreintegration measured;  // biased -- what AddImuMeasurement receives
};

SyntheticStep MakeSyntheticStep(const Sophus::SE3d& pose_i, const Eigen::Vector3d& velocity_i,
                                 const Eigen::Vector3d& gravity, const Eigen::Vector3d& true_gyro,
                                 const Eigen::Vector3d& true_accel, const Eigen::Vector3d& gyro_bias,
                                 double t0, double dt) {
  ImuPreintegration truth;
  ImuMeasurement m1;
  m1.timestamp = t0;
  m1.angular_velocity = true_gyro;
  m1.linear_acceleration = true_accel;
  truth.Integrate(m1);
  ImuMeasurement m2;
  m2.timestamp = t0 + dt;
  truth.Integrate(m2);

  const Eigen::Matrix3d R_i = pose_i.rotationMatrix();

  SyntheticStep step;
  step.pose_j = Sophus::SE3d(pose_i.so3() * truth.DeltaRotation(),
                              pose_i.translation() + velocity_i * dt + 0.5 * gravity * dt * dt +
                                  R_i * truth.DeltaPosition());
  step.velocity_j = velocity_i + gravity * dt + R_i * truth.DeltaVelocity();

  ImuMeasurement biased1 = m1;
  biased1.angular_velocity = true_gyro + gyro_bias;
  step.measured.Integrate(biased1);
  step.measured.Integrate(m2);

  return step;
}

TEST(TightlyCoupledOptimizer, BeforeInitializationBehavesAsPoseOnlyGraph) {
  TightlyCoupledOptimizer::Params params;
  params.init_window_keyframes = 10;  // never reached in this test
  TightlyCoupledOptimizer optimizer(params);

  // Camera moves by `step`; frontends report p_new = relative_pose * p_prev, its inverse.
  const Sophus::SE3d step(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));
  TightlyCoupledOptimizer::EdgeMeasurement vio_edge;
  vio_edge.valid = true;
  vio_edge.relative_pose = step.inverse();
  vio_edge.num_matches = 50;
  TightlyCoupledOptimizer::EdgeMeasurement no_edge;

  optimizer.AddKeyframe(no_edge, no_edge);
  for (int i = 0; i < 5; ++i) {
    optimizer.AddKeyframe(vio_edge, no_edge);
  }

  EXPECT_FALSE(optimizer.IsInitialized());
  EXPECT_EQ(optimizer.NumKeyframes(), 6u);
  const Eigen::Vector3d final_position = optimizer.StateOf(5).pose.translation();
  EXPECT_NEAR(final_position.x(), 5.0, 1e-6);
  // No IMU factor yet -- velocity should stay at its zero seed.
  EXPECT_NEAR(optimizer.StateOf(5).velocity.norm(), 0.0, 1e-9);
}

// Distinct rotating steps: identical steps commute and would hide composition-order errors.
std::vector<Sophus::SE3d> MakeCameraMotions() {
  std::vector<Sophus::SE3d> motions;
  for (int i = 0; i < 6; ++i) {
    const double yaw = 0.05 * (i + 1) * (i % 2 == 0 ? 1.0 : -1.0);
    motions.emplace_back(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())),
                         Eigen::Vector3d(0.03 * i, 0.01 * i, 1.0 + 0.1 * i));
  }
  return motions;
}

// VIO and LiDAR edges take separate code paths; each alone, and both together,
// must reproduce the demos' composition T_new = T_prev * relative_pose.inverse().
TEST(TightlyCoupledOptimizer, VioAndLidarEdgesChainLikeTheDemosComposition) {
  struct Config {
    bool vio;
    bool lidar;
  };
  for (const Config config : {Config{true, false}, Config{false, true}, Config{true, true}}) {
    SCOPED_TRACE(::testing::Message() << "vio=" << config.vio << " lidar=" << config.lidar);
    TightlyCoupledOptimizer::Params params;
    params.init_window_keyframes = 100;  // never reached: pure pose graph
    TightlyCoupledOptimizer optimizer(params);
    TightlyCoupledOptimizer::EdgeMeasurement no_edge;
    optimizer.AddKeyframe(no_edge, no_edge);

    Sophus::SE3d expected;
    for (const Sophus::SE3d& camera_motion : MakeCameraMotions()) {
      TightlyCoupledOptimizer::EdgeMeasurement edge;
      edge.valid = true;
      edge.relative_pose = camera_motion.inverse();
      edge.num_matches = 100;
      const NavNodeId id = optimizer.AddKeyframe(config.vio ? edge : no_edge, config.lidar ? edge : no_edge);
      expected = expected * edge.relative_pose.inverse();
      EXPECT_LT((optimizer.StateOf(id).pose.inverse() * expected).log().norm(), 1e-6)
          << "keyframe " << id;
    }
  }
}

TEST(TightlyCoupledOptimizer, InitializesFromSyntheticImuWindowAndRecoversVelocityAndGravity) {
  const Eigen::Vector3d true_gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d true_gyro_bias(0.02, -0.01, 0.015);
  const Eigen::Vector3d true_gyro(0.0, 0.0, 0.3);
  const Eigen::Vector3d true_accel(0.5, -0.2, 9.81 * 0.02);
  constexpr double dt = 0.1;
  constexpr std::size_t kWindow = 10;

  TightlyCoupledOptimizer::Params params;
  params.init_window_keyframes = kWindow;
  TightlyCoupledOptimizer optimizer(params);

  TightlyCoupledOptimizer::EdgeMeasurement no_edge;
  optimizer.AddKeyframe(no_edge, no_edge);  // keyframe 0, identity seed

  Sophus::SE3d pose = Sophus::SE3d();
  Eigen::Vector3d velocity(1.0, 0.2, 0.0);

  for (std::size_t i = 0; i + 1 < kWindow; ++i) {
    const SyntheticStep synthetic_step = MakeSyntheticStep(pose, velocity, true_gravity, true_gyro,
                                                             true_accel, true_gyro_bias, i * dt, dt);

    // Feed the (biased) raw IMU samples for this interval.
    ImuMeasurement m1;
    m1.timestamp = i * dt;
    m1.angular_velocity = true_gyro + true_gyro_bias;
    m1.linear_acceleration = true_accel;
    optimizer.AddImuMeasurement(m1);
    ImuMeasurement m2;
    m2.timestamp = i * dt + dt;
    optimizer.AddImuMeasurement(m2);

    // Noiseless VIO edge for the true motion, in the frontends' convention
    // (the inverse of pose_i^-1 * pose_j), so seeding matches truth precisely.
    TightlyCoupledOptimizer::EdgeMeasurement vio_edge;
    vio_edge.valid = true;
    vio_edge.relative_pose = synthetic_step.pose_j.inverse() * pose;
    vio_edge.num_matches = 100;
    optimizer.AddKeyframe(vio_edge, no_edge);

    pose = synthetic_step.pose_j;
    velocity = synthetic_step.velocity_j;
  }

  ASSERT_TRUE(optimizer.IsInitialized());
  EXPECT_NEAR((optimizer.Gravity() - true_gravity).norm(), 0.0, 1e-2);
  EXPECT_NEAR((optimizer.StateOf(kWindow - 1).velocity - velocity).norm(), 0.0, 1e-1);
  EXPECT_LT((optimizer.StateOf(kWindow - 1).pose.translation() - pose.translation()).norm(), 5e-2);
}

// Once initialized, a confident IMU factor should pull the estimate toward
// IMU-consistent motion when that transition's pose edge is weak.
TEST(TightlyCoupledOptimizer, ImuFactorCorrectsAWeakPoseEdgeAfterInitialization) {
  const Eigen::Vector3d true_gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d true_gyro(0.0, 0.0, 0.0);
  const Eigen::Vector3d true_accel(1.0, 0.0, 9.81);  // ~constant forward accel, body frame
  constexpr double dt = 0.1;
  constexpr std::size_t kWindow = 5;

  TightlyCoupledOptimizer::Params params;
  params.init_window_keyframes = kWindow;
  params.imu_motion_weight = 1e4;  // confident IMU factor
  TightlyCoupledOptimizer optimizer(params);

  TightlyCoupledOptimizer::EdgeMeasurement no_edge;
  optimizer.AddKeyframe(no_edge, no_edge);

  Sophus::SE3d pose = Sophus::SE3d();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

  for (std::size_t i = 0; i + 1 < kWindow; ++i) {
    const SyntheticStep synthetic_step = MakeSyntheticStep(
        pose, velocity, true_gravity, true_gyro, true_accel, Eigen::Vector3d::Zero(), i * dt, dt);

    ImuMeasurement m1;
    m1.timestamp = i * dt;
    m1.angular_velocity = true_gyro;
    m1.linear_acceleration = true_accel;
    optimizer.AddImuMeasurement(m1);
    ImuMeasurement m2;
    m2.timestamp = i * dt + dt;
    optimizer.AddImuMeasurement(m2);

    TightlyCoupledOptimizer::EdgeMeasurement vio_edge;
    vio_edge.valid = true;
    vio_edge.relative_pose = synthetic_step.pose_j.inverse() * pose;
    vio_edge.num_matches = 100;
    optimizer.AddKeyframe(vio_edge, no_edge);

    pose = synthetic_step.pose_j;
    velocity = synthetic_step.velocity_j;
  }
  ASSERT_TRUE(optimizer.IsInitialized());

  // One more transition: accurate IMU motion, but a deliberately wrong,
  // low-confidence VIO edge (as if that frame's visual tracking degraded).
  const SyntheticStep final_step =
      MakeSyntheticStep(pose, velocity, true_gravity, true_gyro, true_accel,
                         Eigen::Vector3d::Zero(), (kWindow - 1) * dt, dt);

  ImuMeasurement m1;
  m1.timestamp = (kWindow - 1) * dt;
  m1.angular_velocity = true_gyro;
  m1.linear_acceleration = true_accel;
  optimizer.AddImuMeasurement(m1);
  ImuMeasurement m2;
  m2.timestamp = kWindow * dt;
  optimizer.AddImuMeasurement(m2);

  TightlyCoupledOptimizer::EdgeMeasurement bad_vio_edge;
  bad_vio_edge.valid = true;
  Sophus::SE3d wrong_motion = pose.inverse() * final_step.pose_j;
  wrong_motion.translation() += Eigen::Vector3d(0.0, 2.0, 0.0);  // way off, e.g. bad match
  bad_vio_edge.relative_pose = wrong_motion.inverse();
  bad_vio_edge.num_matches = 1;  // low weight

  const NavNodeId last_id = optimizer.AddKeyframe(bad_vio_edge, no_edge);

  // What the bad VIO edge alone (no IMU factor) would have pulled toward.
  const Eigen::Vector3d bad_edge_only_target = (pose * wrong_motion).translation();

  const double dist_to_imu_truth =
      (optimizer.StateOf(last_id).pose.translation() - final_step.pose_j.translation()).norm();
  const double dist_to_bad_edge_target =
      (optimizer.StateOf(last_id).pose.translation() - bad_edge_only_target).norm();

  EXPECT_LT(dist_to_imu_truth, dist_to_bad_edge_target);
  EXPECT_LT(dist_to_imu_truth, 0.3);
}

}  // namespace
}  // namespace slam::backend
