#include "slam/backend/loop_closure.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace slam::backend {
namespace {

frontend_lidar::ScanFeatures MakeCommonPlanarFeatures(double x_offset) {
  frontend_lidar::ScanFeatures features;
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      features.planar_points.emplace_back(
          x_offset + i * 0.5 + 0.05 * std::sin(2.1 * i + j),
          j * 0.5 + 0.04 * std::cos(1.7 * i + 3.2 * j), 0.0);
    }
  }
  return features;
}

TEST(DetectLoopClosure, FindsAndVerifiesNearbyCandidate) {
  const frontend_lidar::ScanFeatures candidate_features = MakeCommonPlanarFeatures(0.0);
  const frontend_lidar::ScanFeatures query_features = MakeCommonPlanarFeatures(0.2);

  std::vector<LoopClosureCandidateSource> candidates;
  candidates.push_back(
      LoopClosureCandidateSource{0, Eigen::Vector3d(0.0, 0.0, 0.0), &candidate_features});

  LoopClosureParams params;
  params.min_node_gap = 5;
  params.min_planar_correspondences = 20;
  params.min_edge_correspondences = 0;

  const auto result = DetectLoopClosure(candidates, query_features, Eigen::Vector3d(0.2, 0.0, 0.0),
                                         /*query_node_id=*/50, params);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->matched_node_id, 0u);
  EXPECT_GT(result->num_planar_correspondences, 20);
}

TEST(DetectLoopClosure, RejectsCandidateOutsideProximityRadius) {
  const frontend_lidar::ScanFeatures candidate_features = MakeCommonPlanarFeatures(0.0);
  const frontend_lidar::ScanFeatures query_features = MakeCommonPlanarFeatures(0.0);

  std::vector<LoopClosureCandidateSource> candidates;
  candidates.push_back(
      LoopClosureCandidateSource{0, Eigen::Vector3d(100.0, 0.0, 0.0), &candidate_features});

  LoopClosureParams params;
  params.min_node_gap = 5;

  const auto result =
      DetectLoopClosure(candidates, query_features, Eigen::Vector3d(0.0, 0.0, 0.0), 50, params);
  EXPECT_FALSE(result.has_value());
}

TEST(DetectLoopClosure, RejectsCandidateTooRecent) {
  const frontend_lidar::ScanFeatures candidate_features = MakeCommonPlanarFeatures(0.0);
  const frontend_lidar::ScanFeatures query_features = MakeCommonPlanarFeatures(0.0);

  std::vector<LoopClosureCandidateSource> candidates;
  candidates.push_back(
      LoopClosureCandidateSource{48, Eigen::Vector3d(0.0, 0.0, 0.0), &candidate_features});

  LoopClosureParams params;
  params.min_node_gap = 5;  // gap here is only 50-48=2

  const auto result =
      DetectLoopClosure(candidates, query_features, Eigen::Vector3d(0.0, 0.0, 0.0), 50, params);
  EXPECT_FALSE(result.has_value());
}

TEST(DetectLoopClosure, ReturnsNulloptWithNoCandidates) {
  const frontend_lidar::ScanFeatures query_features = MakeCommonPlanarFeatures(0.0);
  const auto result = DetectLoopClosure({}, query_features, Eigen::Vector3d::Zero(), 50);
  EXPECT_FALSE(result.has_value());
}

}  // namespace
}  // namespace slam::backend
