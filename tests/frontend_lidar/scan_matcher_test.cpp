#include "slam/frontend_lidar/scan_matcher.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::frontend_lidar {
namespace {

// z=0 exactly (normal is (0,0,1)); (x,y) jitter avoids collinear triples.
std::vector<Eigen::Vector3d> MakeJitteredPlane(int nx, int ny, double spacing) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double x =
          -0.5 * (nx - 1) * spacing + i * spacing + 0.13 * std::sin(2.7 * i + 1.1 * j);
      const double y =
          -0.5 * (ny - 1) * spacing + j * spacing + 0.11 * std::cos(1.9 * i + 3.3 * j);
      points.emplace_back(x, y, 0.0);
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> MakeTwoOrthogonalLines() {
  std::vector<Eigen::Vector3d> points;
  for (double z = 0.2; z <= 3.0; z += 0.2) {
    points.emplace_back(3.0, 3.0, z);
  }
  for (double y = -3.0; y <= 3.0; y += 0.3) {
    points.emplace_back(-3.0, y, 1.5);
  }
  return points;
}

TEST(MatchScans, RecoversKnownSmallRigidTransform) {
  ScanFeatures target;
  target.planar_points = MakeJitteredPlane(12, 12, 0.6);
  target.edge_points = MakeTwoOrthogonalLines();

  const Sophus::SE3d ground_truth(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d(0.3, 0.6, 0.7).normalized())),
      Eigen::Vector3d(0.15, -0.1, 0.05));

  ScanFeatures source;
  source.planar_points.reserve(target.planar_points.size());
  for (const auto& p : target.planar_points) {
    source.planar_points.push_back(ground_truth.inverse() * p);
  }
  source.edge_points.reserve(target.edge_points.size());
  for (const auto& p : target.edge_points) {
    source.edge_points.push_back(ground_truth.inverse() * p);
  }

  const auto match = MatchScans(source, target);
  ASSERT_TRUE(match.has_value());

  const Sophus::SE3d error = ground_truth.inverse() * match->pose;
  EXPECT_LT(error.log().norm(), 5e-3);
  EXPECT_GT(match->num_planar_correspondences, 50);
  EXPECT_GT(match->num_edge_correspondences, 10);
}

// Matching in another coordinate frame (both scans moved by E) must give the
// conjugated relative pose E*T*E^-1 -- why LidarFrontend can apply its
// LiDAR-to-camera extrinsic to features before matching.
TEST(MatchScans, IsEquivariantUnderACommonChangeOfFrame) {
  ScanFeatures target;
  target.planar_points = MakeJitteredPlane(12, 12, 0.6);
  target.edge_points = MakeTwoOrthogonalLines();

  const Sophus::SE3d ground_truth(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d(0.3, 0.6, 0.7).normalized())),
      Eigen::Vector3d(0.15, -0.1, 0.05));
  const Sophus::SE3d frame_change(
      Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(1.0, 2.0, 3.0).normalized())),
      Eigen::Vector3d(0.1, -0.3, 0.7));

  ScanFeatures source;
  for (const auto& p : target.planar_points) source.planar_points.push_back(ground_truth.inverse() * p);
  for (const auto& p : target.edge_points) source.edge_points.push_back(ground_truth.inverse() * p);

  ScanFeatures moved_source;
  ScanFeatures moved_target;
  for (const auto& p : source.planar_points) moved_source.planar_points.push_back(frame_change * p);
  for (const auto& p : source.edge_points) moved_source.edge_points.push_back(frame_change * p);
  for (const auto& p : target.planar_points) moved_target.planar_points.push_back(frame_change * p);
  for (const auto& p : target.edge_points) moved_target.edge_points.push_back(frame_change * p);

  const auto original = MatchScans(source, target);
  const auto moved = MatchScans(moved_source, moved_target);
  ASSERT_TRUE(original.has_value());
  ASSERT_TRUE(moved.has_value());

  const Sophus::SE3d expected = frame_change * original->pose * frame_change.inverse();
  EXPECT_LT((expected.inverse() * moved->pose).log().norm(), 1e-2);
}

// Real per-frame motion at highway speed (~1.5m/0.1s) exceeds the 1m default
// correspondence radius. An identity guess should fail to recover it, while a
// constant-velocity-style guess near the truth should -- this is why
// LidarFrontend seeds MatchScans with the previous frame's motion.
TEST(MatchScans, LargeTranslationNeedsANearbyInitialGuess) {
  ScanFeatures target;
  target.planar_points = MakeJitteredPlane(12, 12, 0.6);
  target.edge_points = MakeTwoOrthogonalLines();

  const Sophus::SE3d ground_truth(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 1.5));

  ScanFeatures source;
  source.planar_points.reserve(target.planar_points.size());
  for (const auto& p : target.planar_points) {
    source.planar_points.push_back(ground_truth.inverse() * p);
  }
  source.edge_points.reserve(target.edge_points.size());
  for (const auto& p : target.edge_points) {
    source.edge_points.push_back(ground_truth.inverse() * p);
  }

  const auto from_identity = MatchScans(source, target);
  if (from_identity.has_value()) {
    const Sophus::SE3d error = ground_truth.inverse() * from_identity->pose;
    EXPECT_GT(error.log().norm(), 0.5) << "expected identity-seeded matching to fail to converge";
  }

  const Sophus::SE3d nearby_guess(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 1.4));
  const auto from_prediction = MatchScans(source, target, nearby_guess);
  ASSERT_TRUE(from_prediction.has_value());
  const Sophus::SE3d error = ground_truth.inverse() * from_prediction->pose;
  EXPECT_LT(error.log().norm(), 5e-3);
}

TEST(MatchScans, ReturnsNulloptWithInsufficientFeatures) {
  ScanFeatures source;
  source.planar_points = {{0, 0, 0}};
  ScanFeatures target;
  target.planar_points = {{0, 0, 0}};

  EXPECT_FALSE(MatchScans(source, target).has_value());
}

}  // namespace
}  // namespace slam::frontend_lidar
