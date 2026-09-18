#include "slam/backend/optimizer.hpp"

#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

// Frontends report p_new = relative_pose * p_prev, the inverse of the camera's
// own motion, so these tests feed step.inverse() for a camera that moved by `step`.
TEST(SlidingWindowOptimizer, ChainsConsistentKeyframesToExpectedTrajectory) {
  SlidingWindowOptimizer optimizer;

  const Sophus::SE3d step(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));

  SlidingWindowOptimizer::EdgeMeasurement vio_edge;
  vio_edge.valid = true;
  vio_edge.relative_pose = step.inverse();
  vio_edge.num_matches = 50;

  SlidingWindowOptimizer::EdgeMeasurement lidar_edge;
  lidar_edge.valid = true;
  lidar_edge.relative_pose = step.inverse();
  lidar_edge.num_matches = 80;

  SlidingWindowOptimizer::EdgeMeasurement no_edge;  // invalid, for the first keyframe

  optimizer.AddKeyframe(no_edge, no_edge);
  for (int i = 0; i < 5; ++i) {
    optimizer.AddKeyframe(vio_edge, lidar_edge);
  }

  EXPECT_EQ(optimizer.NumKeyframes(), 6u);
  const Eigen::Vector3d final_position = optimizer.PoseOf(5).translation();
  EXPECT_NEAR(final_position.x(), 5.0, 1e-6);
  EXPECT_NEAR(final_position.y(), 0.0, 1e-6);
  EXPECT_NEAR(final_position.z(), 0.0, 1e-6);
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

// The demos chain frontend output as T_new = T_prev * relative_pose.inverse();
// the optimizer must agree, including for rotating motion.
TEST(SlidingWindowOptimizer, MatchesTheDemosChainedCompositionOfFrontendPoses) {
  SlidingWindowOptimizer optimizer;
  SlidingWindowOptimizer::EdgeMeasurement no_edge;
  optimizer.AddKeyframe(no_edge, no_edge);

  Sophus::SE3d expected;
  std::size_t keyframe = 0;
  for (const Sophus::SE3d& camera_motion : MakeCameraMotions()) {
    SlidingWindowOptimizer::EdgeMeasurement edge;
    edge.valid = true;
    edge.relative_pose = camera_motion.inverse();
    edge.num_matches = 100;
    optimizer.AddKeyframe(edge, edge);
    expected = expected * edge.relative_pose.inverse();
    EXPECT_LT((optimizer.PoseOf(++keyframe).inverse() * expected).log().norm(), 1e-6)
        << "keyframe " << keyframe;
  }
  EXPECT_GT(optimizer.PoseOf(keyframe).translation().z(), 5.0);  // forward, not backward
}

TEST(SlidingWindowOptimizer, FreezesKeyframesOutsideWindowWithoutBreakingTheChain) {
  SlidingWindowOptimizer::Params params;
  params.window_size = 3;
  SlidingWindowOptimizer optimizer(params);

  const Sophus::SE3d step(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));
  SlidingWindowOptimizer::EdgeMeasurement edge;
  edge.valid = true;
  edge.relative_pose = step.inverse();
  edge.num_matches = 50;
  SlidingWindowOptimizer::EdgeMeasurement no_edge;

  optimizer.AddKeyframe(no_edge, no_edge);
  for (int i = 0; i < 8; ++i) {
    optimizer.AddKeyframe(edge, no_edge);
  }

  EXPECT_EQ(optimizer.NumKeyframes(), 9u);
  EXPECT_NEAR(optimizer.PoseOf(8).translation().x(), 8.0, 1e-6);
}

TEST(SlidingWindowOptimizer, ImuEdgeConstrainsRotationOnlyNotTranslation) {
  SlidingWindowOptimizer optimizer;
  SlidingWindowOptimizer::EdgeMeasurement no_edge;  // vio/lidar both invalid throughout

  optimizer.AddKeyframe(no_edge, no_edge);  // keyframe 0

  slam::frontend_vio::ImuPreintegrationResult imu_delta;
  imu_delta.delta_time = 0.1;
  imu_delta.delta_rotation =
      Sophus::SO3d(Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY())));
  // delta_position is deliberately left at its default zero -- irrelevant
  // here since AddKeyframe only consumes delta_rotation from this struct.

  optimizer.AddKeyframe(no_edge, no_edge, imu_delta);  // keyframe 1, IMU-only edge

  const Sophus::SE3d& pose1 = optimizer.PoseOf(1);
  const Eigen::Matrix3d expected_rotation =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  EXPECT_TRUE(pose1.rotationMatrix().isApprox(expected_rotation, 1e-6));
  // No vio/lidar edge and zero-weighted translation rows on the IMU edge:
  // translation should stay at its (identity) seed, unconstrained.
  EXPECT_NEAR(pose1.translation().norm(), 0.0, 1e-6);
}

TEST(SlidingWindowOptimizer, OptimizeGloballyIsSafeWithNoLoopClosures) {
  SlidingWindowOptimizer optimizer;
  SlidingWindowOptimizer::EdgeMeasurement no_edge;
  optimizer.AddKeyframe(no_edge, no_edge);

  const Sophus::SE3d step(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));
  SlidingWindowOptimizer::EdgeMeasurement edge;
  edge.valid = true;
  edge.relative_pose = step.inverse();
  edge.num_matches = 50;
  optimizer.AddKeyframe(edge, no_edge);

  EXPECT_EQ(optimizer.NumLoopClosures(), 0);
  optimizer.OptimizeGlobally();
  EXPECT_NEAR(optimizer.PoseOf(1).translation().x(), 1.0, 1e-6);
}

}  // namespace
}  // namespace slam::backend
