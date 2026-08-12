#include "slam/frontend_lidar/voxel_grid.hpp"

#include <gtest/gtest.h>

namespace slam::frontend_lidar {
namespace {

TEST(VoxelDownsample, MergesPointsInSameVoxel) {
  const std::vector<Eigen::Vector3d> points = {
      {0.05, 0.05, 0.05}, {0.06, 0.04, 0.05}, {5.0, 5.0, 5.0}};
  const auto result = VoxelDownsample(points, 1.0);
  ASSERT_EQ(result.size(), 2u);
}

TEST(VoxelDownsample, CentroidIsAverageOfMergedPoints) {
  const std::vector<Eigen::Vector3d> points = {{0.1, 0.1, 0.1}, {0.3, 0.3, 0.3}};
  const auto result = VoxelDownsample(points, 1.0);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x(), 0.2, 1e-9);
  EXPECT_NEAR(result[0].y(), 0.2, 1e-9);
  EXPECT_NEAR(result[0].z(), 0.2, 1e-9);
}

TEST(VoxelDownsample, HandlesEmptyInput) {
  const auto result = VoxelDownsample({}, 1.0);
  EXPECT_TRUE(result.empty());
}

}  // namespace
}  // namespace slam::frontend_lidar
