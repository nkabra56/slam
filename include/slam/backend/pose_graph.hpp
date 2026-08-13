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
  // Measured relative transform: maps a point in `from`'s frame into `to`'s
  // frame (p_to = measurement * p_from) -- the same convention
  // VioFrontend/LidarFrontend use for their per-frame relative pose.
  Sophus::SE3d measurement;
  // 6x6 information (inverse-covariance) matrix, Sophus SE3 tangent order
  // (translation-like part first, then rotation part). A scaled identity is
  // enough for our sources -- see SlidingWindowOptimizer.
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Identity()};
};

// A graph of keyframe poses (nodes) connected by relative-pose constraints
// (edges) from odometry and loop closure, solved with hand-written
// Gauss-Newton over the SE3 manifold -- no g2o/GTSAM/Ceres.
//
// Edge Jacobians are computed by central finite differences rather than an
// analytic SE3 adjoint derivation. Both are legitimate from-scratch,
// no-external-optimizer techniques, but the analytic route (differentiating
// Log(z^-1 * T_to^-1 * T_from) through the SE3 adjoint) is easy to get
// subtly sign-wrong, and this project has no compiler in the loop to catch
// that empirically. Numerical differentiation is correct by construction
// (up to O(h^2) truncation) and costs nothing that matters at pose-graph
// scale (tens of nodes, not thousands). See ROADMAP.md.
//
// The normal equations are dense each iteration -- correct and simple at
// sliding-window scale, not intended for a full thousands-of-keyframes
// graph (which would want a sparse solver).
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

  // Anchors a node's pose during Solve() -- required for at least one node
  // (a graph of only relative constraints has a free-floating global gauge
  // otherwise); also how the sliding window freezes old keyframes.
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
