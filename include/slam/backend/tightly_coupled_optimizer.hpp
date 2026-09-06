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

// Phase 6A tightly-coupled fusion (PHASE6_PLAN.md section 2): unlike
// SlidingWindowOptimizer, which treats IMU as a weak rotation-only
// regularizer bolted onto a 6-DOF pose graph, this class carries a genuine
// 15-DOF NavState (pose + velocity + gyro/accel bias) per keyframe and
// fuses VIO/LiDAR relative-pose edges with a real bias-and-gravity-aware
// IMU factor (imu_factor.hpp) inside one NavStateGraph
// (nav_state_graph.hpp) -- a deliberately separate class from
// SlidingWindowOptimizer/PoseGraph rather than a generalization of either,
// per PHASE6_PLAN.md section 2.7.
//
// A NavState-based IMU factor is meaningless without a decent initial
// velocity/gravity/bias estimate (VioInitializer, vio_initializer.hpp), so
// this class runs in two stages, transparently to the caller:
//   1. Bootstrap: every keyframe gets a graph node and pose edges (VIO/
//      LiDAR/loop-closure) immediately, exactly like SlidingWindowOptimizer
//      -- so PoseOf-equivalent results are available from keyframe 0. Raw
//      IMU is buffered per inter-keyframe interval but not yet turned into
//      NavImuEdges. Once `params.init_window_keyframes` keyframes exist,
//      every AddKeyframe call retries VioInitializer over the most recent
//      such window until it succeeds (skips silently on a gap in IMU data
//      or a degenerate solve -- see VioInitializer's own doc comment --
//      and tries again with the next keyframe's window).
//   2. Tightly coupled: once initialization succeeds, gravity and that
//      window's per-node velocity/bias are seeded from the result, a
//      NavImuEdge is added retroactively for each of that window's
//      transitions, and every subsequent AddKeyframe call adds a live
//      NavImuEdge alongside the pose edges. IsInitialized() reports which
//      stage is active. If IMU data never arrives at all (or every window
//      is degenerate), this class simply runs as a pose-only NavStateGraph
//      forever -- a safe, honest fallback, not an error.
class TightlyCoupledOptimizer {
 public:
  using EdgeMeasurement = SlidingWindowOptimizer::EdgeMeasurement;

  struct Params {
    std::size_t window_size = 20;
    // How many keyframes VioInitializer runs over. Must be >= 3 (its own
    // requirement); each attempted window needs every one of its
    // transitions to have a valid (delta_time > 0) IMU preintegration.
    std::size_t init_window_keyframes = 10;
    double vio_weight = 1.0;
    double lidar_weight = 1.0;
    double max_edge_weight = 200.0;
    // IMU factor weights: a single scaled-identity per residual block,
    // the same "scaled identity is enough for our sources" simplification
    // PoseGraph/SlidingWindowOptimizer already make (see optimizer.hpp) --
    // this project does not propagate a real sensor-noise covariance
    // through preintegration (see imu_factor.hpp's class doc comment).
    double imu_motion_weight = 1.0;
    double imu_bias_random_walk_weight = 1.0;
    LoopClosureParams loop_closure;
  };

  TightlyCoupledOptimizer() : TightlyCoupledOptimizer(Params{}) {}
  explicit TightlyCoupledOptimizer(Params params);

  // Adds one raw IMU sample, accumulated into the interval since the last
  // AddKeyframe call. Call zero or more times between consecutive
  // AddKeyframe calls, in increasing timestamp order -- same convention as
  // VioFrontend::ProcessImu (see the KITTI demo apps). Correctly preserves
  // integration continuity across keyframe boundaries: the sample that
  // closed the previous interval also seeds the next one, so a single
  // sample per keyframe (KITTI raw oxts's rate) still produces a
  // nonzero-duration preintegration per interval.
  void AddImuMeasurement(const ImuMeasurement& measurement);

  NavNodeId AddKeyframe(const EdgeMeasurement& vio_edge, const EdgeMeasurement& lidar_edge,
                         std::optional<frontend_lidar::ScanFeatures> lidar_features = std::nullopt);

  bool IsInitialized() const { return initialized_; }
  const NavState& StateOf(NavNodeId id) const;
  const Eigen::Vector3d& Gravity() const { return graph_.Gravity(); }
  std::size_t NumKeyframes() const;
  int NumLoopClosures() const { return num_loop_closures_; }

  // Unfixes every node except the first and re-solves the full graph,
  // including all loop closures -- see SlidingWindowOptimizer::OptimizeGlobally.
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
  // interval_preintegrations_[i]: raw (zero-bias) preintegration for the
  // transition node i -> node i+1, or nullopt if no usable IMU data was
  // seen for that interval. Always sized keyframes_.size() - 1.
  std::vector<std::optional<ImuPreintegration>> interval_preintegrations_;
  int num_loop_closures_{0};
  bool initialized_{false};

  // Accumulates raw IMU since the last AddKeyframe call.
  std::optional<ImuPreintegration> imu_since_last_keyframe_;
  // Last sample seen by AddImuMeasurement, retained across keyframe
  // boundaries purely to seed the next interval's continuity (see
  // AddImuMeasurement's doc comment).
  std::optional<ImuMeasurement> last_imu_sample_;
};

}  // namespace slam::backend
