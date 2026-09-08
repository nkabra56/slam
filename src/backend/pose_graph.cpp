#include "slam/backend/pose_graph.hpp"

#include <algorithm>
#include <utility>

namespace slam::backend {

namespace {

Eigen::Matrix<double, 6, 1> ComputeResidual(const PoseGraphEdge& edge, const Sophus::SE3d& T_from,
                                             const Sophus::SE3d& T_to) {
  const Sophus::SE3d error = edge.measurement.inverse() * T_to.inverse() * T_from;
  return error.log();
}

}  // namespace

NodeId PoseGraph::AddNode(const Sophus::SE3d& initial_pose) {
  poses_.push_back(initial_pose);
  fixed_.push_back(false);
  return poses_.size() - 1;
}

void PoseGraph::AddEdge(PoseGraphEdge edge) { edges_.push_back(std::move(edge)); }

void PoseGraph::FixNode(NodeId id) { fixed_.at(id) = true; }

void PoseGraph::UnfixAll() { std::fill(fixed_.begin(), fixed_.end(), false); }

const Sophus::SE3d& PoseGraph::Pose(NodeId id) const { return poses_.at(id); }

void PoseGraph::SetPose(NodeId id, const Sophus::SE3d& pose) { poses_.at(id) = pose; }

std::size_t PoseGraph::NumNodes() const { return poses_.size(); }

int PoseGraph::Solve(const SolveParams& params) {
  if (poses_.empty() || edges_.empty()) return 0;

  // Without a fixed node the whole graph can float/rotate arbitrarily.
  std::vector<bool> fixed = fixed_;
  if (std::none_of(fixed.begin(), fixed.end(), [](bool f) { return f; })) {
    fixed[0] = true;
  }

  std::vector<int> var_index(poses_.size(), -1);
  int num_free = 0;
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    if (!fixed[i]) var_index[i] = num_free++;
  }
  if (num_free == 0) return 0;

  const int dim = 6 * num_free;
  int iterations_run = 0;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

    for (const auto& edge : edges_) {
      const Sophus::SE3d& T_from = poses_.at(edge.from);
      const Sophus::SE3d& T_to = poses_.at(edge.to);
      const Eigen::Matrix<double, 6, 1> r = ComputeResidual(edge, T_from, T_to);

      const double h = params.numerical_jacobian_step;
      Eigen::Matrix<double, 6, 6> J_from;
      Eigen::Matrix<double, 6, 6> J_to;
      for (int k = 0; k < 6; ++k) {
        Eigen::Matrix<double, 6, 1> dp = Eigen::Matrix<double, 6, 1>::Zero();
        dp(k) = h;
        const Eigen::Matrix<double, 6, 1> r_plus_from =
            ComputeResidual(edge, T_from * Sophus::SE3d::exp(dp), T_to);
        const Eigen::Matrix<double, 6, 1> r_minus_from =
            ComputeResidual(edge, T_from * Sophus::SE3d::exp(-dp), T_to);
        J_from.col(k) = (r_plus_from - r_minus_from) / (2 * h);

        const Eigen::Matrix<double, 6, 1> r_plus_to =
            ComputeResidual(edge, T_from, T_to * Sophus::SE3d::exp(dp));
        const Eigen::Matrix<double, 6, 1> r_minus_to =
            ComputeResidual(edge, T_from, T_to * Sophus::SE3d::exp(-dp));
        J_to.col(k) = (r_plus_to - r_minus_to) / (2 * h);
      }

      const int idx_from = var_index[edge.from];
      const int idx_to = var_index[edge.to];

      if (idx_from >= 0) {
        H.block<6, 6>(6 * idx_from, 6 * idx_from) +=
            J_from.transpose() * edge.information * J_from;
        b.segment<6>(6 * idx_from) += -J_from.transpose() * edge.information * r;
      }
      if (idx_to >= 0) {
        H.block<6, 6>(6 * idx_to, 6 * idx_to) += J_to.transpose() * edge.information * J_to;
        b.segment<6>(6 * idx_to) += -J_to.transpose() * edge.information * r;
      }
      if (idx_from >= 0 && idx_to >= 0) {
        H.block<6, 6>(6 * idx_from, 6 * idx_to) += J_from.transpose() * edge.information * J_to;
        H.block<6, 6>(6 * idx_to, 6 * idx_from) += J_to.transpose() * edge.information * J_from;
      }
    }

    H.diagonal().array() += params.damping;
    const Eigen::VectorXd dx = H.ldlt().solve(b);

    for (std::size_t i = 0; i < poses_.size(); ++i) {
      if (var_index[i] < 0) continue;
      const Eigen::Matrix<double, 6, 1> delta = dx.segment<6>(6 * var_index[i]);
      poses_[i] = poses_[i] * Sophus::SE3d::exp(delta);
    }

    ++iterations_run;
    if (dx.squaredNorm() < params.convergence_threshold) break;
  }

  return iterations_run;
}

}  // namespace slam::backend
