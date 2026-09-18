#include "slam/backend/optimizer.hpp"

#include <algorithm>
#include <utility>

namespace slam::backend {

SlidingWindowOptimizer::SlidingWindowOptimizer(Params params) : params_(std::move(params)) {}

Eigen::Matrix<double, 6, 6> SlidingWindowOptimizer::WeightedInformation(double weight,
                                                                         int num_matches) const {
  const double scale = std::min(static_cast<double>(num_matches), params_.max_edge_weight);
  return (weight * scale) * Eigen::Matrix<double, 6, 6>::Identity();
}

Eigen::Matrix<double, 6, 6> SlidingWindowOptimizer::RotationOnlyInformation(double weight) const {
  // Tangent order is (translation, rotation); zeroing rows/cols 0-2 makes
  // this edge ignore measurement's translation slot entirely.
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information.block<3, 3>(3, 3) = weight * Eigen::Matrix3d::Identity();
  return information;
}

NodeId SlidingWindowOptimizer::AddKeyframe(
    const EdgeMeasurement& vio_edge, const EdgeMeasurement& lidar_edge,
    const std::optional<frontend_vio::ImuPreintegrationResult>& imu_edge,
    std::optional<frontend_lidar::ScanFeatures> lidar_features) {
  const NodeId new_id =
      graph_.AddNode(keyframes_.empty() ? Sophus::SE3d() : graph_.Pose(keyframes_.size() - 1));

  if (new_id > 0) {
    const NodeId prev_id = new_id - 1;
    if (vio_edge.valid) {
      // relative_pose is T_new = T_prev * relative_pose, not PoseGraphEdge's
      // own T_to^-1 * T_from -- needs inverting, same as the IMU edge below.
      PoseGraphEdge edge;
      edge.from = prev_id;
      edge.to = new_id;
      edge.measurement = vio_edge.relative_pose.inverse();
      edge.information = WeightedInformation(params_.vio_weight, vio_edge.num_matches);
      graph_.AddEdge(edge);
    }
    if (lidar_edge.valid) {
      PoseGraphEdge edge;
      edge.from = prev_id;
      edge.to = new_id;
      edge.measurement = lidar_edge.relative_pose.inverse();
      edge.information = WeightedInformation(params_.lidar_weight, lidar_edge.num_matches);
      graph_.AddEdge(edge);
    }
    if (imu_edge.has_value() && imu_edge->delta_time > 0.0) {
      // delta_rotation = R_prev^-1*R_curr; PoseGraphEdge wants
      // T_to^-1*T_from, so this needs inverting.
      PoseGraphEdge edge;
      edge.from = prev_id;
      edge.to = new_id;
      edge.measurement = Sophus::SE3d(imu_edge->delta_rotation.inverse(), Eigen::Vector3d::Zero());
      edge.information = RotationOnlyInformation(params_.imu_rotation_weight);
      graph_.AddEdge(edge);
    }

    // Prefer LiDAR (denser constraint) over VIO for the seed pose.
    const Sophus::SE3d seed_relative =
        lidar_edge.valid ? lidar_edge.relative_pose
        : vio_edge.valid ? vio_edge.relative_pose
                         : Sophus::SE3d();
    graph_.SetPose(new_id, graph_.Pose(prev_id) * seed_relative.inverse());
  }

  keyframes_.push_back(KeyframeRecord{graph_.Pose(new_id).translation(), lidar_features});

  if (lidar_features.has_value()) {
    std::vector<LoopClosureCandidateSource> candidates;
    candidates.reserve(keyframes_.size());
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      if (!keyframes_[i].lidar_features.has_value()) continue;
      candidates.push_back(LoopClosureCandidateSource{
          i, keyframes_[i].position, &keyframes_[i].lidar_features.value()});
    }

    if (const auto loop = DetectLoopClosure(candidates, *lidar_features, keyframes_.back().position,
                                             new_id, params_.loop_closure)) {
      PoseGraphEdge edge;
      edge.from = loop->matched_node_id;
      edge.to = new_id;
      edge.measurement = loop->relative_pose;
      edge.information = WeightedInformation(
          params_.lidar_weight, loop->num_edge_correspondences + loop->num_planar_correspondences);
      graph_.AddEdge(edge);
      ++num_loop_closures_;
    }
  }

  // Freeze whatever has just fallen outside the active window.
  if (graph_.NumNodes() > params_.window_size) {
    const NodeId boundary = graph_.NumNodes() - params_.window_size;
    graph_.FixNode(boundary - 1);
  }

  graph_.Solve();
  return new_id;
}

const Sophus::SE3d& SlidingWindowOptimizer::PoseOf(NodeId id) const { return graph_.Pose(id); }

std::size_t SlidingWindowOptimizer::NumKeyframes() const { return keyframes_.size(); }

int SlidingWindowOptimizer::OptimizeGlobally() {
  graph_.UnfixAll();
  if (graph_.NumNodes() > 0) {
    graph_.FixNode(0);
  }
  PoseGraph::SolveParams params;
  params.max_iterations = 50;
  return graph_.Solve(params);
}

}  // namespace slam::backend
