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

// Relative-pose constraint on the pose sub-block (tangent dims 0-5) of two
// NavState nodes; same convention as PoseGraphEdge.
struct NavPoseEdge {
  NavNodeId from{};
  NavNodeId to{};
  Sophus::SE3d measurement;
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Identity()};
};

// IMU factor between two NavState nodes. Separate information matrices for
// the motion residual and the bias-random-walk residual (different noise sources).
struct NavImuEdge {
  NavNodeId from{};
  NavNodeId to{};
  ImuPreintegration preintegration;
  Eigen::Matrix<double, 9, 9> motion_information{Eigen::Matrix<double, 9, 9>::Identity()};
  Eigen::Matrix<double, 6, 6> bias_information{Eigen::Matrix<double, 6, 6>::Identity()};
};

// 15-DOF analogue of PoseGraph for keyframes carrying velocity and IMU bias.
// Dense Gauss-Newton with numerical Jacobians, same as PoseGraph.
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
