#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/backend/imu_factor.hpp"
#include "slam/backend/loop_closure.hpp"
#include "slam/backend/nav_state.hpp"
#include "slam/backend/nav_state_graph.hpp"
#include "slam/backend/optimizer.hpp"
#include "slam/backend/vio_initializer.hpp"
#include "slam/common/types.hpp"
#include "slam/frontend_lidar/scan_features.hpp"

namespace slam::backend {

// 15-DOF sibling to SlidingWindowOptimizer: fuses pose edges with a real IMU
// factor once VioInitializer succeeds; pose-only until then.
class TightlyCoupledOptimizer {
 public:
  using EdgeMeasurement = SlidingWindowOptimizer::EdgeMeasurement;

  struct Params {
    std::size_t window_size = 20;
    // VioInitializer window size; must be >= 3.
    std::size_t init_window_keyframes = 10;
    double vio_weight = 1.0;
    double lidar_weight = 1.0;
    double max_edge_weight = 200.0;
    // Scaled-identity weights -- no real per-sensor noise covariance.
    double imu_motion_weight = 1.0;
    double imu_bias_random_walk_weight = 1.0;
    LoopClosureParams loop_closure;
  };

  TightlyCoupledOptimizer() : TightlyCoupledOptimizer(Params{}) {}
  explicit TightlyCoupledOptimizer(Params params);

  // Accumulates since the last AddKeyframe call; seeds continuity across
  // keyframe boundaries.
  void AddImuMeasurement(const ImuMeasurement& measurement);

  NavNodeId AddKeyframe(const EdgeMeasurement& vio_edge, const EdgeMeasurement& lidar_edge,
                         std::optional<frontend_lidar::ScanFeatures> lidar_features = std::nullopt);

  bool IsInitialized() const { return initialized_; }
  const NavState& StateOf(NavNodeId id) const;
  const Eigen::Vector3d& Gravity() const { return graph_.Gravity(); }
  std::size_t NumKeyframes() const;
  int NumLoopClosures() const { return num_loop_closures_; }

  // Unfixes every node except the first and re-solves the full graph.
  int OptimizeGlobally();

 private:
  struct KeyframeRecord {
    Eigen::Vector3d position;
    std::optional<frontend_lidar::ScanFeatures> lidar_features;
  };

  Eigen::Matrix<double, 6, 6> WeightedPoseInformation(double weight, int num_matches) const;
  void TryDetectLoopClosure(NavNodeId new_id,
                             const std::optional<frontend_lidar::ScanFeatures>& lidar_features);
  void TryInitialize();
  void FreezeOutsideWindow();

  Params params_;
  NavStateGraph graph_;
  std::vector<KeyframeRecord> keyframes_;
  // interval_preintegrations_[i]: transition i -> i+1, or nullopt if no IMU
  // data was seen. Sized keyframes_.size() - 1.
  std::vector<std::optional<ImuPreintegration>> interval_preintegrations_;
  int num_loop_closures_{0};
  bool initialized_{false};

  std::optional<ImuPreintegration> imu_since_last_keyframe_;
  // Retained across keyframe boundaries to seed the next interval.
  std::optional<ImuMeasurement> last_imu_sample_;
};

}  // namespace slam::backend
