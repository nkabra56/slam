#include "slam/frontend_lidar/kdtree.hpp"

#include <algorithm>
#include <random>

#include <gtest/gtest.h>

namespace slam::frontend_lidar {
namespace {

TEST(KdTree3d, FindsNearestNeighborExactMatch) {
  const std::vector<Eigen::Vector3d> points = {{0, 0, 0}, {1, 0, 0}, {5, 5, 5}, {2, 2, 2}};
  const KdTree3d tree(points);
  const auto result = tree.KNearest(Eigen::Vector3d(1.0, 0.0, 0.0), 1);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0], 1u);
}

TEST(KdTree3d, ReturnsKNearestInSortedOrder) {
  const std::vector<Eigen::Vector3d> points = {{10, 0, 0}, {0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  const KdTree3d tree(points);
  const auto result = tree.KNearest(Eigen::Vector3d(0.0, 0.0, 0.0), 3);
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0], 1u);
  EXPECT_EQ(result[1], 2u);
  EXPECT_EQ(result[2], 3u);
}

TEST(KdTree3d, MatchesBruteForceOnRandomPoints) {
  std::vector<Eigen::Vector3d> points;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> dist(-50.0, 50.0);
  for (int i = 0; i < 200; ++i) {
    points.emplace_back(dist(rng), dist(rng), dist(rng));
  }
  const KdTree3d tree(points);

  const Eigen::Vector3d query(3.0, -4.0, 10.0);
  const auto tree_result = tree.KNearest(query, 5);

  std::vector<std::pair<double, std::size_t>> brute;
  for (std::size_t i = 0; i < points.size(); ++i) {
    brute.emplace_back((points[i] - query).squaredNorm(), i);
  }
  std::sort(brute.begin(), brute.end());

  ASSERT_EQ(tree_result.size(), 5u);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(tree_result[static_cast<std::size_t>(i)], brute[static_cast<std::size_t>(i)].second);
  }
}

TEST(KdTree3d, HandlesEmptyTree) {
  const KdTree3d tree(std::vector<Eigen::Vector3d>{});
  EXPECT_TRUE(tree.KNearest(Eigen::Vector3d::Zero(), 3).empty());
}

}  // namespace
}  // namespace slam::frontend_lidar
