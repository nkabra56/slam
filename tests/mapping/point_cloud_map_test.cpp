#include "slam/mapping/point_cloud_map.hpp"

#include <gtest/gtest.h>

namespace slam::mapping {
namespace {

TEST(PointCloudMap, MergesPointsInSameVoxel) {
  PointCloudMap map(1.0);
  map.Insert({{0.1, 0.1, 0.1}, {0.2, 0.2, 0.2}, {5.0, 5.0, 5.0}});
  EXPECT_EQ(map.NumPoints(), 2u);
}

TEST(PointCloudMap, AccumulatesAcrossMultipleInsertCalls) {
  PointCloudMap map(1.0);
  map.Insert({{0.1, 0.1, 0.1}});
  map.Insert({{0.3, 0.3, 0.3}});
  ASSERT_EQ(map.NumPoints(), 1u);
  const auto points = map.Points();
  EXPECT_NEAR(points[0].x(), 0.2, 1e-9);
  EXPECT_NEAR(points[0].y(), 0.2, 1e-9);
  EXPECT_NEAR(points[0].z(), 0.2, 1e-9);
}

TEST(PointCloudMap, EmptyMapHasNoPoints) {
  PointCloudMap map(1.0);
  EXPECT_EQ(map.NumPoints(), 0u);
  EXPECT_TRUE(map.Points().empty());
}

TEST(PointCloudMap, DistinctVoxelsStayDistinct) {
  PointCloudMap map(1.0);
  map.Insert({{0.1, 0.1, 0.1}, {1.5, 1.5, 1.5}, {-1.5, -1.5, -1.5}});
  EXPECT_EQ(map.NumPoints(), 3u);
}

}  // namespace
}  // namespace slam::mapping
