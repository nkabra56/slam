#include "slam/frontend_lidar/ground_removal.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace slam::frontend_lidar {
namespace {

std::vector<Eigen::Vector3d> MakeGroundPlaneWithWall() {
  std::vector<Eigen::Vector3d> points;
  for (double x = -5.0; x < 5.0; x += 0.2) {
    for (double y = -5.0; y < 5.0; y += 0.2) {
      points.emplace_back(x, y, 0.0);
    }
  }
  for (double x = -2.0; x < 2.0; x += 0.2) {
    for (double z = 0.5; z < 3.0; z += 0.2) {
      points.emplace_back(x, 6.0, z);
    }
  }
  return points;
}

TEST(RemoveGround, SeparatesGroundFromWall) {
  const auto points = MakeGroundPlaneWithWall();
  const auto result = RemoveGround(points);
  ASSERT_TRUE(result.has_value());

  EXPECT_NEAR(std::abs(result->plane.z()), 1.0, 1e-2);

  for (const auto& p : result->ground_points) {
    EXPECT_NEAR(p.z(), 0.0, 0.25);
  }
  EXPECT_FALSE(result->non_ground_points.empty());
  for (const auto& p : result->non_ground_points) {
    EXPECT_GT(p.z(), 0.3);
  }
}

TEST(RemoveGround, ReturnsNulloptWithTooFewPoints) {
  const std::vector<Eigen::Vector3d> points = {{0, 0, 0}, {1, 0, 0}};
  EXPECT_FALSE(RemoveGround(points).has_value());
}

}  // namespace
}  // namespace slam::frontend_lidar
