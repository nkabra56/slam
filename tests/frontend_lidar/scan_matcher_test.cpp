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

TEST(MatchScans, ReturnsNulloptWithInsufficientFeatures) {
  ScanFeatures source;
  source.planar_points = {{0, 0, 0}};
  ScanFeatures target;
  target.planar_points = {{0, 0, 0}};

  EXPECT_FALSE(MatchScans(source, target).has_value());
}

}  // namespace
}  // namespace slam::frontend_lidar
