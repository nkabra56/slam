#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace slam::backend {

using NodeId = std::size_t;

struct PoseGraphEdge {
  NodeId from{};
  NodeId to{};
  // p_to = measurement * p_from.
  Sophus::SE3d measurement;
  // 6x6 information matrix, SE3 tangent order (translation, then rotation).
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Identity()};
};

// Keyframe pose graph solved via Gauss-Newton over SE3 with numerical
// Jacobians. Dense normal equations -- fine at sliding-window scale only.
class PoseGraph {
 public:
  struct SolveParams {
    int max_iterations = 20;
    double convergence_threshold = 1e-10;  // squared-norm of the update step
    double damping = 1e-6;
    double numerical_jacobian_step = 1e-6;
  };

  NodeId AddNode(const Sophus::SE3d& initial_pose);
  void AddEdge(PoseGraphEdge edge);

  // Anchors a node's pose during Solve(); at least one node must be fixed.
  void FixNode(NodeId id);
  void UnfixAll();

  const Sophus::SE3d& Pose(NodeId id) const;
  void SetPose(NodeId id, const Sophus::SE3d& pose);
  std::size_t NumNodes() const;

  // Returns the number of iterations run.
  int Solve() { return Solve(SolveParams{}); }
  int Solve(const SolveParams& params);

 private:
  std::vector<Sophus::SE3d> poses_;
  std::vector<bool> fixed_;
  std::vector<PoseGraphEdge> edges_;
};

}  // namespace slam::backend
