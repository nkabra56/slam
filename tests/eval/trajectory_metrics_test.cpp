#include "slam/eval/trajectory_metrics.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::eval {
namespace {

TEST(ComputeAte, ZeroErrorForIdenticalTrajectories) {
  std::vector<Sophus::SE3d> poses;
  for (int i = 0; i < 5; ++i) {
    poses.push_back(
        Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(i, 0.0, 0.0)));
  }
  const auto result = ComputeAte(poses, poses);
  EXPECT_NEAR(result.rmse_m, 0.0, 1e-9);
}

TEST(ComputeAte, RecoversZeroErrorUnderKnownRigidOffset) {
  std::vector<Sophus::SE3d> ground_truth;
  for (int i = 0; i < 6; ++i) {
    ground_truth.push_back(Sophus::SE3d(Eigen::Quaterniond::Identity(),
                                         Eigen::Vector3d(i, 0.3 * std::sin(i), 0.0)));
  }

  const Sophus::SE3d offset(Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ())),
                             Eigen::Vector3d(5.0, -3.0, 1.0));

  std::vector<Sophus::SE3d> estimated;
  estimated.reserve(ground_truth.size());
  for (const auto& p : ground_truth) estimated.push_back(offset.inverse() * p);

  const auto result = ComputeAte(estimated, ground_truth);
  EXPECT_LT(result.rmse_m, 1e-6);
}

TEST(ComputeAte, ThrowsOnMismatchedLengths) {
  const std::vector<Sophus::SE3d> a(3);
  const std::vector<Sophus::SE3d> b(2);
  EXPECT_THROW(ComputeAte(a, b), std::invalid_argument);
}

TEST(ComputeAte, ThrowsWithTooFewPoses) {
  const std::vector<Sophus::SE3d> a(2);
  EXPECT_THROW(ComputeAte(a, a), std::invalid_argument);
}

TEST(ComputeKittiOdometryError, ZeroErrorForIdenticalTrajectories) {
  std::vector<Sophus::SE3d> poses;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  for (int i = 0; i < 20; ++i) {
    poses.push_back(Sophus::SE3d(Eigen::Quaterniond::Identity(), position));
    position += Eigen::Vector3d(10.0, 0.0, 0.0);  // 10m steps -> 190m total path
  }
  const auto result = ComputeKittiOdometryError(poses, poses);
  EXPECT_GT(result.num_segments_evaluated, 0u);
  EXPECT_NEAR(result.avg_translation_error_percent, 0.0, 1e-9);
  EXPECT_NEAR(result.avg_rotation_error_deg_per_100m, 0.0, 1e-9);
}

TEST(ComputeKittiOdometryError, DetectsScaleError) {
  // gt moves 10m/step, estimate moves 11m/step -- a clean, hand-verifiable
  // 10% scale error: over any 100m ground-truth segment (10 steps), the
  // true relative translation is 100m and the estimated one is 110m, so
  // the error is exactly 10m / 100m = 10%.
  std::vector<Sophus::SE3d> ground_truth;
  std::vector<Sophus::SE3d> estimated;
  Eigen::Vector3d gt_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d est_position = Eigen::Vector3d::Zero();
  for (int i = 0; i < 20; ++i) {
    ground_truth.push_back(Sophus::SE3d(Eigen::Quaterniond::Identity(), gt_position));
    estimated.push_back(Sophus::SE3d(Eigen::Quaterniond::Identity(), est_position));
    gt_position += Eigen::Vector3d(10.0, 0.0, 0.0);
    est_position += Eigen::Vector3d(11.0, 0.0, 0.0);
  }

  const auto result = ComputeKittiOdometryError(estimated, ground_truth);
  ASSERT_GT(result.num_segments_evaluated, 0u);
  EXPECT_NEAR(result.avg_translation_error_percent, 10.0, 0.5);
  EXPECT_NEAR(result.avg_rotation_error_deg_per_100m, 0.0, 1e-6);
}

TEST(ComputeKittiOdometryError, ReturnsZeroSegmentsWhenTrajectoryTooShort) {
  std::vector<Sophus::SE3d> poses;
  for (int i = 0; i < 3; ++i) {
    poses.push_back(Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(i, 0.0, 0.0)));
  }
  const auto result = ComputeKittiOdometryError(poses, poses);
  EXPECT_EQ(result.num_segments_evaluated, 0u);
}

TEST(ComputeKittiOdometryError, ThrowsOnMismatchedLengths) {
  const std::vector<Sophus::SE3d> a(3);
  const std::vector<Sophus::SE3d> b(2);
  EXPECT_THROW(ComputeKittiOdometryError(a, b), std::invalid_argument);
}

}  // namespace
}  // namespace slam::eval
