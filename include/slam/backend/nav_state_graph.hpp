#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/backend/imu_factor.hpp"
#include "slam/backend/nav_state.hpp"

namespace slam::backend {

using NavNodeId = std::size_t;

// A VIO/LiDAR-style relative-pose constraint applied to the pose
// sub-block (tangent dims 0-5) of two NavState nodes. Same measurement
// convention and residual formula as PoseGraphEdge (pose_graph.hpp) --
// p_to = measurement * p_from, residual = Log(measurement^-1 * T_to^-1 *
// T_from) -- reused rather than re-derived: this is a "where does this
// residual go in a bigger state" placement problem, not new math. See
// PHASE6_PLAN.md section 2.7.
struct NavPoseEdge {
  NavNodeId from{};
  NavNodeId to{};
  Sophus::SE3d measurement;
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Identity()};
};

// A full IMU factor (imu_factor.hpp) between two NavState nodes. Two
// separate information matrices because the motion residual (rotation/
// velocity/position, from sensor noise) and the bias-random-walk residual
// (from bias drift over time) have genuinely different noise sources.
struct NavImuEdge {
  NavNodeId from{};
  NavNodeId to{};
  ImuPreintegration preintegration;
  Eigen::Matrix<double, 9, 9> motion_information{Eigen::Matrix<double, 9, 9>::Identity()};
  Eigen::Matrix<double, 6, 6> bias_information{Eigen::Matrix<double, 6, 6>::Identity()};
};

// 15-DOF analogue of PoseGraph (pose_graph.hpp) for the tightly-coupled
// case where keyframes carry velocity and IMU bias alongside pose. A
// deliberately separate class rather than a generalized variable-dimension
// graph -- see PHASE6_PLAN.md section 2.7 for why. Same solving strategy
// as PoseGraph: dense Gauss-Newton, numerical (central-difference)
// Jacobians, for the same reason PoseGraph uses them (no compiler in this
// project's environment to empirically verify an analytic derivation).
class NavStateGraph {
 public:
  struct SolveParams {
    int max_iterations = 20;
    double convergence_threshold = 1e-10;
    double damping = 1e-6;
    double numerical_jacobian_step = 1e-6;
  };

  NavNodeId AddNode(const NavState& initial_state);
  void AddPoseEdge(NavPoseEdge edge);
  void AddImuEdge(NavImuEdge edge);

  // See PoseGraph::FixNode/UnfixAll -- same gauge-fixing role.
  void FixNode(NavNodeId id);
  void UnfixAll();

  const NavState& State(NavNodeId id) const;
  void SetState(NavNodeId id, const NavState& state);
  std::size_t NumNodes() const;

  void SetGravity(const Eigen::Vector3d& gravity) { gravity_ = gravity; }
  const Eigen::Vector3d& Gravity() const { return gravity_; }

  int Solve() { return Solve(SolveParams{}); }
  int Solve(const SolveParams& params);

 private:
  using ResidualFn = std::function<Eigen::VectorXd(const NavState&, const NavState&)>;

  void AccumulateFactor(const std::vector<int>& var_index, NavNodeId from_id, NavNodeId to_id,
                         int residual_dim, const ResidualFn& residual_fn,
                         const Eigen::MatrixXd& information, double numerical_step,
                         Eigen::MatrixXd& H, Eigen::VectorXd& b) const;

  std::vector<NavState> states_;
  std::vector<bool> fixed_;
  std::vector<NavPoseEdge> pose_edges_;
  std::vector<NavImuEdge> imu_edges_;
  Eigen::Vector3d gravity_{0.0, 0.0, -9.81};
};

}  // namespace slam::backend
