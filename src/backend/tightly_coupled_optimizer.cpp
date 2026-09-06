#include "slam/backend/tightly_coupled_optimizer.hpp"

#include <algorithm>
#include <utility>

namespace slam::backend {

TightlyCoupledOptimizer::TightlyCoupledOptimizer(Params params) : params_(std::move(params)) {}

Eigen::Matrix<double, 6, 6> TightlyCoupledOptimizer::WeightedPoseInformation(
    double weight, int num_matches) const {
  const double scale = std::min(static_cast<double>(num_matches), params_.max_edge_weight);
  return (weight * scale) * Eigen::Matrix<double, 6, 6>::Identity();
}

void TightlyCoupledOptimizer::AddImuMeasurement(const ImuMeasurement& measurement) {
  if (!imu_since_last_keyframe_.has_value()) {
    imu_since_last_keyframe_.emplace();
    if (last_imu_sample_.has_value()) {
      // Seeds continuity across the keyframe boundary: this call only sets
      // ImuPreintegration's internal "previous sample," it contributes no
      // delta by itself (see ImuPreintegration::Integrate). The next real
      // call below then integrates from this boundary sample to the new
      // one, exactly the "single preintegration step per keyframe
      // interval" ROADMAP.md describes for KITTI's one-sample-per-frame
      // raw oxts data.
      imu_since_last_keyframe_->Integrate(*last_imu_sample_);
    }
  }
  imu_since_last_keyframe_->Integrate(measurement);
  last_imu_sample_ = measurement;
}

void TightlyCoupledOptimizer::TryDetectLoopClosure(
    NavNodeId new_id, const std::optional<frontend_lidar::ScanFeatures>& lidar_features) {
  if (!lidar_features.has_value()) return;

  std::vector<LoopClosureCandidateSource> candidates;
  candidates.reserve(keyframes_.size());
  for (std::size_t i = 0; i < keyframes_.size(); ++i) {
    if (!keyframes_[i].lidar_features.has_value()) continue;
    candidates.push_back(
        LoopClosureCandidateSource{i, keyframes_[i].position, &keyframes_[i].lidar_features.value()});
  }

  if (const auto loop = DetectLoopClosure(candidates, *lidar_features, keyframes_.back().position,
                                           new_id, params_.loop_closure)) {
    NavPoseEdge edge;
    edge.from = loop->matched_node_id;
    edge.to = new_id;
    edge.measurement = loop->relative_pose;
    edge.information = WeightedPoseInformation(
        params_.lidar_weight, loop->num_edge_correspondences + loop->num_planar_correspondences);
    graph_.AddPoseEdge(edge);
    ++num_loop_closures_;
  }
}

void TightlyCoupledOptimizer::TryInitialize() {
  const std::size_t n = params_.init_window_keyframes;
  const std::size_t start = keyframes_.size() - n;

  std::vector<Sophus::SE3d> window_poses;
  window_poses.reserve(n);
  for (std::size_t i = start; i < start + n; ++i) {
    window_poses.push_back(graph_.State(i).pose);
  }

  std::vector<ImuPreintegration> window_preintegrations;
  window_preintegrations.reserve(n - 1);
  for (std::size_t i = start; i + 1 < start + n; ++i) {
    const auto& interval = interval_preintegrations_[i];
    if (!interval.has_value() || interval->DeltaTime() <= 0.0) {
      return;  // gap in this window -- retry with the next keyframe's window
    }
    window_preintegrations.push_back(*interval);
  }

  const auto result = InitializeVio(window_poses, window_preintegrations);
  if (!result.has_value()) return;  // degenerate window -- retry next keyframe

  graph_.SetGravity(result->gravity);
  for (std::size_t i = start; i < start + n; ++i) {
    NavState state = graph_.State(i);
    state.velocity = result->velocities[i - start];
    state.bias_gyro = result->bias_gyro;
    state.bias_accel = Eigen::Vector3d::Zero();
    graph_.SetState(i, state);
  }

  for (std::size_t i = start; i + 1 < start + n; ++i) {
    NavImuEdge edge;
    edge.from = i;
    edge.to = i + 1;
    edge.preintegration = *interval_preintegrations_[i];
    edge.motion_information = params_.imu_motion_weight * Eigen::Matrix<double, 9, 9>::Identity();
    edge.bias_information =
        params_.imu_bias_random_walk_weight * Eigen::Matrix<double, 6, 6>::Identity();
    graph_.AddImuEdge(edge);
  }

  initialized_ = true;
}

void TightlyCoupledOptimizer::FreezeOutsideWindow() {
  if (graph_.NumNodes() > params_.window_size) {
    const NavNodeId boundary = graph_.NumNodes() - params_.window_size;
    graph_.FixNode(boundary - 1);
  }
}

NavNodeId TightlyCoupledOptimizer::AddKeyframe(const EdgeMeasurement& vio_edge,
                                                const EdgeMeasurement& lidar_edge,
                                                std::optional<frontend_lidar::ScanFeatures> lidar_features) {
  NavNodeId new_id;

  if (keyframes_.empty()) {
    new_id = graph_.AddNode(NavState{});
  } else {
    const NavNodeId prev_id = keyframes_.size() - 1;

    // Seed the new node's pose the same way SlidingWindowOptimizer does
    // (prefer LiDAR, denser geometric constraint, over VIO); carry over
    // the previous node's velocity/bias as a warm-start guess.
    const Sophus::SE3d seed_relative =
        lidar_edge.valid ? lidar_edge.relative_pose
        : vio_edge.valid ? vio_edge.relative_pose
                          : Sophus::SE3d();
    NavState seed = graph_.State(prev_id);
    seed.pose = graph_.State(prev_id).pose * seed_relative.inverse();
    new_id = graph_.AddNode(seed);

    if (vio_edge.valid) {
      NavPoseEdge edge;
      edge.from = prev_id;
      edge.to = new_id;
      edge.measurement = vio_edge.relative_pose.inverse();
      edge.information = WeightedPoseInformation(params_.vio_weight, vio_edge.num_matches);
      graph_.AddPoseEdge(edge);
    }
    if (lidar_edge.valid) {
      NavPoseEdge edge;
      edge.from = prev_id;
      edge.to = new_id;
      edge.measurement = lidar_edge.relative_pose.inverse();
      edge.information = WeightedPoseInformation(params_.lidar_weight, lidar_edge.num_matches);
      graph_.AddPoseEdge(edge);
    }

    std::optional<ImuPreintegration> interval;
    if (imu_since_last_keyframe_.has_value() && imu_since_last_keyframe_->DeltaTime() > 0.0) {
      interval = imu_since_last_keyframe_;
    }
    imu_since_last_keyframe_.reset();
    interval_preintegrations_.push_back(interval);

    if (initialized_ && interval.has_value()) {
      NavImuEdge edge;
      edge.from = prev_id;
      edge.to = new_id;
      edge.preintegration = *interval;
      edge.motion_information = params_.imu_motion_weight * Eigen::Matrix<double, 9, 9>::Identity();
      edge.bias_information =
          params_.imu_bias_random_walk_weight * Eigen::Matrix<double, 6, 6>::Identity();
      graph_.AddImuEdge(edge);
    }
  }

  keyframes_.push_back(KeyframeRecord{graph_.State(new_id).pose.translation(), lidar_features});
  TryDetectLoopClosure(new_id, lidar_features);

  // Solve with the newly added node/edges before TryInitialize() reads
  // window poses -- otherwise the just-added node still holds its raw seed
  // guess (correct only up to translation for a pure-translation seed
  // formula; wrong once the chain has real rotation), which poisons
  // InitializeVio's linear system.
  FreezeOutsideWindow();
  graph_.Solve();

  if (!initialized_ && keyframes_.size() >= params_.init_window_keyframes) {
    TryInitialize();
    if (initialized_) {
      // Blend the retroactively added NavImuEdges into the solved state
      // immediately rather than waiting for the next AddKeyframe call.
      graph_.Solve();
    }
  }

  return new_id;
}

const NavState& TightlyCoupledOptimizer::StateOf(NavNodeId id) const { return graph_.State(id); }

std::size_t TightlyCoupledOptimizer::NumKeyframes() const { return keyframes_.size(); }

int TightlyCoupledOptimizer::OptimizeGlobally() {
  graph_.UnfixAll();
  if (graph_.NumNodes() > 0) {
    graph_.FixNode(0);
  }
  NavStateGraph::SolveParams params;
  params.max_iterations = 50;
  return graph_.Solve(params);
}

}  // namespace slam::backend
