#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/frontend_lidar/scan_features.hpp"

namespace slam::backend {

using NodeId = std::size_t;

struct LoopClosureCandidateSource {
  NodeId node_id{};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  const frontend_lidar::ScanFeatures* features{nullptr};
};

struct LoopClosureResult {
  NodeId matched_node_id{};
  // Maps the matched (historical) keyframe's frame into the query
  // keyframe's frame: p_query = relative_pose * p_matched.
  Sophus::SE3d relative_pose;
  int num_edge_correspondences{0};
  int num_planar_correspondences{0};
};

struct LoopClosureParams {
  double proximity_radius_m = 5.0;
  NodeId min_node_gap = 20;  // ignore recent keyframes -- not a real loop
  int min_planar_correspondences = 30;
  int min_edge_correspondences = 5;
};

// Geometric-proximity loop closure via MatchScans against nearby,
// sufficiently-old candidates.
std::optional<LoopClosureResult> DetectLoopClosure(
    const std::vector<LoopClosureCandidateSource>& candidates,
    const frontend_lidar::ScanFeatures& query_features, const Eigen::Vector3d& query_position,
    NodeId query_node_id, const LoopClosureParams& params = {});

}  // namespace slam::backend
