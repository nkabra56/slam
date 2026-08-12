#include "slam/backend/optimizer.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

TEST(SlidingWindowOptimizer, ChainsConsistentKeyframesToExpectedTrajectory) {
  SlidingWindowOptimizer optimizer;

  const Sophus::SE3d step(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));

  SlidingWindowOptimizer::EdgeMeasurement vio_edge;
  vio_edge.valid = true;
  vio_edge.relative_pose = step;
  vio_edge.num_matches = 50;

  SlidingWindowOptimizer::EdgeMeasurement lidar_edge;
  lidar_edge.valid = true;
  lidar_edge.relative_pose = step;
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

TEST(SlidingWindowOptimizer, FreezesKeyframesOutsideWindowWithoutBreakingTheChain) {
  SlidingWindowOptimizer::Params params;
  params.window_size = 3;
  SlidingWindowOptimizer optimizer(params);

  const Sophus::SE3d step(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));
  SlidingWindowOptimizer::EdgeMeasurement edge;
  edge.valid = true;
  edge.relative_pose = step;
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
  edge.relative_pose = step;
  edge.num_matches = 50;
  optimizer.AddKeyframe(edge, no_edge);

  EXPECT_EQ(optimizer.NumLoopClosures(), 0);
  optimizer.OptimizeGlobally();
  EXPECT_NEAR(optimizer.PoseOf(1).translation().x(), 1.0, 1e-6);
}

}  // namespace
}  // namespace slam::backend
