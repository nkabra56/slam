#include "slam/backend/loop_closure.hpp"

#include "slam/frontend_lidar/scan_matcher.hpp"

namespace slam::backend {

std::optional<LoopClosureResult> DetectLoopClosure(
    const std::vector<LoopClosureCandidateSource>& candidates,
    const frontend_lidar::ScanFeatures& query_features, const Eigen::Vector3d& query_position,
    NodeId query_node_id, const LoopClosureParams& params) {
  std::optional<LoopClosureResult> best;
  int best_score = 0;

  for (const auto& candidate : candidates) {
    if (candidate.features == nullptr) continue;
    if (query_node_id < candidate.node_id ||
        query_node_id - candidate.node_id < params.min_node_gap) {
      continue;
    }
    if ((candidate.position - query_position).norm() > params.proximity_radius_m) continue;

    const auto match = frontend_lidar::MatchScans(*candidate.features, query_features);
    if (!match) continue;
    if (match->num_planar_correspondences < params.min_planar_correspondences ||
        match->num_edge_correspondences < params.min_edge_correspondences) {
      continue;
    }

    const int score = match->num_planar_correspondences + match->num_edge_correspondences;
    if (score > best_score) {
      best_score = score;
      LoopClosureResult result;
      result.matched_node_id = candidate.node_id;
      result.relative_pose = match->pose;
      result.num_edge_correspondences = match->num_edge_correspondences;
      result.num_planar_correspondences = match->num_planar_correspondences;
      best = result;
    }
  }

  return best;
}

}  // namespace slam::backend
