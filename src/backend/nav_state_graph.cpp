#include "slam/backend/nav_state_graph.hpp"

#include <algorithm>
#include <utility>

namespace slam::backend {

namespace {
constexpr int kNavStateDim = 15;
}  // namespace

NavNodeId NavStateGraph::AddNode(const NavState& initial_state) {
  states_.push_back(initial_state);
  fixed_.push_back(false);
  return states_.size() - 1;
}

void NavStateGraph::AddPoseEdge(NavPoseEdge edge) { pose_edges_.push_back(std::move(edge)); }

void NavStateGraph::AddImuEdge(NavImuEdge edge) { imu_edges_.push_back(std::move(edge)); }

void NavStateGraph::FixNode(NavNodeId id) { fixed_.at(id) = true; }

void NavStateGraph::UnfixAll() { std::fill(fixed_.begin(), fixed_.end(), false); }

const NavState& NavStateGraph::State(NavNodeId id) const { return states_.at(id); }

void NavStateGraph::SetState(NavNodeId id, const NavState& state) { states_.at(id) = state; }

std::size_t NavStateGraph::NumNodes() const { return states_.size(); }

void NavStateGraph::AccumulateFactor(const std::vector<int>& var_index, NavNodeId from_id,
                                      NavNodeId to_id, int residual_dim,
                                      const ResidualFn& residual_fn,
                                      const Eigen::MatrixXd& information, double h,
                                      Eigen::MatrixXd& H, Eigen::VectorXd& b) const {
  const NavState& from_state = states_[from_id];
  const NavState& to_state = states_[to_id];
  const Eigen::VectorXd r = residual_fn(from_state, to_state);

  Eigen::MatrixXd J_from(residual_dim, kNavStateDim);
  Eigen::MatrixXd J_to(residual_dim, kNavStateDim);
  for (int k = 0; k < kNavStateDim; ++k) {
    Eigen::Matrix<double, kNavStateDim, 1> dp = Eigen::Matrix<double, kNavStateDim, 1>::Zero();

    dp(k) = h;
    const Eigen::VectorXd r_plus_from = residual_fn(Retract(from_state, dp), to_state);
    dp(k) = -h;
    const Eigen::VectorXd r_minus_from = residual_fn(Retract(from_state, dp), to_state);
    J_from.col(k) = (r_plus_from - r_minus_from) / (2 * h);

    dp(k) = h;
    const Eigen::VectorXd r_plus_to = residual_fn(from_state, Retract(to_state, dp));
    dp(k) = -h;
    const Eigen::VectorXd r_minus_to = residual_fn(from_state, Retract(to_state, dp));
    J_to.col(k) = (r_plus_to - r_minus_to) / (2 * h);
  }

  const int idx_from = var_index[from_id];
  const int idx_to = var_index[to_id];

  if (idx_from >= 0) {
    H.block(kNavStateDim * idx_from, kNavStateDim * idx_from, kNavStateDim, kNavStateDim) +=
        J_from.transpose() * information * J_from;
    b.segment(kNavStateDim * idx_from, kNavStateDim) += -J_from.transpose() * information * r;
  }
  if (idx_to >= 0) {
    H.block(kNavStateDim * idx_to, kNavStateDim * idx_to, kNavStateDim, kNavStateDim) +=
        J_to.transpose() * information * J_to;
    b.segment(kNavStateDim * idx_to, kNavStateDim) += -J_to.transpose() * information * r;
  }
  if (idx_from >= 0 && idx_to >= 0) {
    H.block(kNavStateDim * idx_from, kNavStateDim * idx_to, kNavStateDim, kNavStateDim) +=
        J_from.transpose() * information * J_to;
    H.block(kNavStateDim * idx_to, kNavStateDim * idx_from, kNavStateDim, kNavStateDim) +=
        J_to.transpose() * information * J_from;
  }
}

int NavStateGraph::Solve(const SolveParams& params) {
  if (states_.empty() || (pose_edges_.empty() && imu_edges_.empty())) return 0;

  std::vector<bool> fixed = fixed_;
  if (std::none_of(fixed.begin(), fixed.end(), [](bool f) { return f; })) {
    fixed[0] = true;
  }

  std::vector<int> var_index(states_.size(), -1);
  int num_free = 0;
  for (std::size_t i = 0; i < states_.size(); ++i) {
    if (!fixed[i]) var_index[i] = num_free++;
  }
  if (num_free == 0) return 0;

  const int dim = kNavStateDim * num_free;
  int iterations_run = 0;
  const double h = params.numerical_jacobian_step;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

    for (const auto& edge : pose_edges_) {
      const ResidualFn residual_fn = [&edge](const NavState& from, const NavState& to) {
        const Sophus::SE3d error = edge.measurement.inverse() * to.pose.inverse() * from.pose;
        return Eigen::VectorXd(error.log());
      };
      AccumulateFactor(var_index, edge.from, edge.to, 6, residual_fn, edge.information, h, H, b);
    }

    for (const auto& edge : imu_edges_) {
      const Eigen::Vector3d gravity = gravity_;
      const ResidualFn motion_fn = [&edge, gravity](const NavState& from, const NavState& to) {
        return Eigen::VectorXd(
            ComputeImuFactorResidual(from, to, edge.preintegration, gravity).motion);
      };
      AccumulateFactor(var_index, edge.from, edge.to, 9, motion_fn, edge.motion_information, h, H,
                        b);

      const ResidualFn bias_fn = [](const NavState& from, const NavState& to) {
        Eigen::Matrix<double, 6, 1> r;
        r.segment<3>(0) = to.bias_gyro - from.bias_gyro;
        r.segment<3>(3) = to.bias_accel - from.bias_accel;
        return Eigen::VectorXd(r);
      };
      AccumulateFactor(var_index, edge.from, edge.to, 6, bias_fn, edge.bias_information, h, H, b);
    }

    H.diagonal().array() += params.damping;
    const Eigen::VectorXd dx = H.ldlt().solve(b);

    for (std::size_t i = 0; i < states_.size(); ++i) {
      if (var_index[i] < 0) continue;
      const Eigen::Matrix<double, kNavStateDim, 1> delta =
          dx.segment(kNavStateDim * var_index[i], kNavStateDim);
      states_[i] = Retract(states_[i], delta);
    }

    ++iterations_run;
    if (dx.squaredNorm() < params.convergence_threshold) break;
  }

  return iterations_run;
}

}  // namespace slam::backend
