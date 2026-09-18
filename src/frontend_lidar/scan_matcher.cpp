#include "slam/frontend_lidar/scan_matcher.hpp"

#include <Eigen/Dense>

#include "slam/frontend_lidar/kdtree.hpp"

namespace slam::frontend_lidar {

namespace {

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  // clang-format off
  m <<     0, -v.z(),  v.y(),
        v.z(),      0, -v.x(),
       -v.y(),  v.x(),      0;
  // clang-format on
  return m;
}

// d(pose * Exp(dxi) * p_source)/dxi at dxi=0, for Sophus's SE3 tangent
// ordering (translation-like part first, then rotation part).
Eigen::Matrix<double, 3, 6> PointJacobian(const Eigen::Matrix3d& R, const Eigen::Vector3d& p_source) {
  Eigen::Matrix<double, 3, 6> j;
  j.block<3, 3>(0, 0) = R;
  j.block<3, 3>(0, 3) = -R * Skew(p_source);
  return j;
}

}  // namespace

std::optional<ScanMatchResult> MatchScans(const ScanFeatures& source, const ScanFeatures& target,
                                           const Sophus::SE3d& initial_guess,
                                           const ScanMatcherParams& params) {
  if (target.edge_points.size() < 2 && target.planar_points.size() < 3) {
    return std::nullopt;
  }

  const KdTree3d edge_tree(target.edge_points);
  const KdTree3d planar_tree(target.planar_points);

  Sophus::SE3d pose = initial_guess;
  int last_num_edge = 0;
  int last_num_planar = 0;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    std::vector<Eigen::Matrix<double, 1, 6>> jacobian_rows;
    std::vector<double> residual_rows;

    const Eigen::Matrix3d R = pose.rotationMatrix();

    int num_edge = 0;
    for (const auto& p_source : source.edge_points) {
      const Eigen::Vector3d p_target = pose * p_source;
      const auto neighbors = edge_tree.KNearest(p_target, 2);
      if (neighbors.size() < 2) continue;
      const Eigen::Vector3d& a = target.edge_points[neighbors[0]];
      const Eigen::Vector3d& b = target.edge_points[neighbors[1]];
      if ((p_target - a).norm() > params.max_edge_correspondence_dist) continue;

      const Eigen::Vector3d d = b - a;
      const double d_norm = d.norm();
      if (d_norm < 1e-6) continue;
      const Eigen::Vector3d u = d / d_norm;

      const Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - u * u.transpose();
      const Eigen::Vector3d e = projector * (p_target - a);
      const Eigen::Matrix<double, 3, 6> de_dxi = projector * PointJacobian(R, p_source);

      for (int row = 0; row < 3; ++row) {
        jacobian_rows.push_back(de_dxi.row(row));
        residual_rows.push_back(e(row));
      }
      ++num_edge;
    }

    int num_planar = 0;
    for (const auto& p_source : source.planar_points) {
      const Eigen::Vector3d p_target = pose * p_source;
      const auto neighbors = planar_tree.KNearest(p_target, 3);
      if (neighbors.size() < 3) continue;
      const Eigen::Vector3d& a = target.planar_points[neighbors[0]];
      const Eigen::Vector3d& b = target.planar_points[neighbors[1]];
      const Eigen::Vector3d& c = target.planar_points[neighbors[2]];
      if ((p_target - a).norm() > params.max_planar_correspondence_dist) continue;

      Eigen::Vector3d n = (b - a).cross(c - a);
      const double n_norm = n.norm();
      if (n_norm < 1e-6) continue;
      n /= n_norm;

      const double e = n.dot(p_target - a);
      const Eigen::Matrix<double, 1, 6> de_dxi = n.transpose() * PointJacobian(R, p_source);

      jacobian_rows.push_back(de_dxi);
      residual_rows.push_back(e);
      ++num_planar;
    }

    last_num_edge = num_edge;
    last_num_planar = num_planar;

    const int num_residuals = static_cast<int>(residual_rows.size());
    if (num_residuals < 6) {
      return std::nullopt;
    }

    Eigen::MatrixXd J(num_residuals, 6);
    Eigen::VectorXd r(num_residuals);
    for (int i = 0; i < num_residuals; ++i) {
      J.row(i) = jacobian_rows[static_cast<std::size_t>(i)];
      r(i) = residual_rows[static_cast<std::size_t>(i)];
    }

    Eigen::Matrix<double, 6, 6> H = J.transpose() * J;
    H.diagonal().array() += params.damping_ratio * num_residuals;
    const Eigen::Matrix<double, 6, 1> b = -J.transpose() * r;
    const Eigen::Matrix<double, 6, 1> delta = H.ldlt().solve(b);

    pose = pose * Sophus::SE3d::exp(delta);

    if (delta.squaredNorm() < params.convergence_threshold) {
      break;
    }
  }

  ScanMatchResult result;
  result.pose = pose;
  result.num_edge_correspondences = last_num_edge;
  result.num_planar_correspondences = last_num_planar;
  return result;
}

}  // namespace slam::frontend_lidar
