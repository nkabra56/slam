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

// Fuses VIO and LiDAR relative poses into one PoseGraph. Keyframes outside
// the window are frozen, not marginalized.
class SlidingWindowOptimizer {
 public:
  struct EdgeMeasurement {
    bool valid{false};
    Sophus::SE3d relative_pose;  // p_this = relative_pose * p_previous (frontends' convention)
    int num_matches{0};          // inliers (VIO) or correspondences (LiDAR)
  };

  struct Params {
    std::size_t window_size = 20;
    double vio_weight = 1.0;
    double lidar_weight = 1.0;
    double max_edge_weight = 200.0;  // caps a single edge's influence
    // Higher than vio/lidar_weight: gyro is trustworthy over one frame interval.
    double imu_rotation_weight = 10.0;
    LoopClosureParams loop_closure;
  };

  SlidingWindowOptimizer() : SlidingWindowOptimizer(Params{}) {}
  explicit SlidingWindowOptimizer(Params params);

  // `imu_edge`, if given, contributes rotation only. Re-solves the active window.
  NodeId AddKeyframe(const EdgeMeasurement& vio_edge, const EdgeMeasurement& lidar_edge,
                      const std::optional<frontend_vio::ImuPreintegrationResult>& imu_edge = std::nullopt,
                      std::optional<frontend_lidar::ScanFeatures> lidar_features = std::nullopt);

  const Sophus::SE3d& PoseOf(NodeId id) const;
  std::size_t NumKeyframes() const;
  int NumLoopClosures() const { return num_loop_closures_; }

  // Unfixes every node except the first and re-solves the full graph.
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
