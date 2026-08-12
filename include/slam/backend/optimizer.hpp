#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/backend/loop_closure.hpp"
#include "slam/backend/pose_graph.hpp"
#include "slam/frontend_lidar/scan_features.hpp"
#include "slam/frontend_vio/imu_preintegrator.hpp"

namespace slam::backend {

// Fuses VIO and LiDAR frame-to-frame relative poses into one optimized
// trajectory: each new keyframe adds a VIO edge and/or a LiDAR edge to a
// shared PoseGraph (pose_graph.hpp), solved with hand-written Gauss-Newton
// -- no GTSAM/g2o/Ceres, per ROADMAP.md's optimizer policy.
//
// Keyframes older than the sliding window are frozen (PoseGraph::FixNode)
// rather than truly marginalized via a Schur complement -- a deliberate
// Phase 3 scope decision (see ROADMAP.md), not an oversight: true
// marginalization would fold a dropped node's constraints into a prior on
// its neighbor, which is real added complexity beyond this phase's budget.
// Loop closures (loop_closure.hpp) still add edges from the active window
// back to frozen history, correcting the *current* window relative to it;
// OptimizeGlobally() additionally unfreezes the whole trajectory for a
// one-shot final correction, e.g. at the end of a full sequence run.
class SlidingWindowOptimizer {
 public:
  struct EdgeMeasurement {
    bool valid{false};
    Sophus::SE3d relative_pose;  // previous keyframe frame -> this keyframe frame
    int num_matches{0};          // inliers (VIO) or correspondences (LiDAR)
  };

  struct Params {
    std::size_t window_size = 20;
    double vio_weight = 1.0;
    double lidar_weight = 1.0;
    double max_edge_weight = 200.0;  // caps a single edge's influence
    // Weight on the IMU rotation-only edge (see AddKeyframe). Gyro
    // integration over one KITTI frame interval (~0.1s) is normally quite
    // trustworthy -- no gravity/bias-drift concerns at that timescale --
    // so this defaults higher than the visual/LiDAR edge weights.
    double imu_rotation_weight = 10.0;
    LoopClosureParams loop_closure;
  };

  explicit SlidingWindowOptimizer(Params params = {});

  // Adds a new keyframe. `vio_edge`/`lidar_edge` are relative-pose
  // estimates from the previous keyframe (either may have `valid == false`
  // if that frontend failed to track this frame). `imu_edge`, if given, is
  // the preintegrated IMU delta between the previous keyframe and this one
  // (VioFrontend::FrameResult::imu_delta): only its *rotation* is used, via
  // an edge whose information matrix has zero weight on the translation
  // rows -- gyro integration is reliable at this timescale, but the
  // preintegrated position isn't (it has no velocity-state or gravity
  // compensation here; that's Phase 6 tightly-coupled fusion). See
  // ROADMAP.md. `lidar_features`, if given, is used both to attempt
  // closing a loop against earlier keyframes now, and for future
  // loop-closure queries against this one. Re-solves the active window
  // before returning.
  NodeId AddKeyframe(const EdgeMeasurement& vio_edge, const EdgeMeasurement& lidar_edge,
                      const std::optional<frontend_vio::ImuPreintegrationResult>& imu_edge = std::nullopt,
                      std::optional<frontend_lidar::ScanFeatures> lidar_features = std::nullopt);

  const Sophus::SE3d& PoseOf(NodeId id) const;
  std::size_t NumKeyframes() const;
  int NumLoopClosures() const { return num_loop_closures_; }

  // Unfixes every node except the first (kept as the gauge anchor) and
  // re-solves the full accumulated graph, including all loop closures.
  int OptimizeGlobally();

 private:
  struct KeyframeRecord {
    Eigen::Vector3d position;
    std::optional<frontend_lidar::ScanFeatures> lidar_features;
  };

  Eigen::Matrix<double, 6, 6> WeightedInformation(double weight, int num_matches) const;
  Eigen::Matrix<double, 6, 6> RotationOnlyInformation(double weight) const;

  Params params_;
  PoseGraph graph_;
  std::vector<KeyframeRecord> keyframes_;
  int num_loop_closures_{0};
};

}  // namespace slam::backend
