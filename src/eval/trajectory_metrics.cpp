#include "slam/eval/trajectory_metrics.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace slam::eval {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, 8> kSegmentLengthsM = {100, 200, 300, 400, 500, 600, 700, 800};

std::vector<double> CumulativePathLength(const std::vector<Sophus::SE3d>& poses) {
  std::vector<double> lengths(poses.size(), 0.0);
  for (std::size_t i = 1; i < poses.size(); ++i) {
    lengths[i] = lengths[i - 1] + (poses[i].translation() - poses[i - 1].translation()).norm();
  }
  return lengths;
}

// First frame at or beyond `target_length` of ground-truth path past
// `start`, or cumulative_length.size() if the trajectory doesn't reach it.
std::size_t FindEndFrame(const std::vector<double>& cumulative_length, std::size_t start,
                          double target_length) {
  const double target = cumulative_length[start] + target_length;
  for (std::size_t j = start + 1; j < cumulative_length.size(); ++j) {
    if (cumulative_length[j] >= target) return j;
  }
  return cumulative_length.size();
}

}  // namespace

AbsoluteTrajectoryError ComputeAte(const std::vector<Sophus::SE3d>& estimated,
                                    const std::vector<Sophus::SE3d>& ground_truth) {
  if (estimated.size() != ground_truth.size()) {
    throw std::invalid_argument("ComputeAte: estimated and ground_truth must be the same length");
  }
  if (estimated.size() < 3) {
    throw std::invalid_argument("ComputeAte: need at least 3 poses to align a trajectory");
  }

  const std::size_t n = estimated.size();

  Eigen::Vector3d centroid_est = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroid_gt = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    centroid_est += estimated[i].translation();
    centroid_gt += ground_truth[i].translation();
  }
  centroid_est /= static_cast<double>(n);
  centroid_gt /= static_cast<double>(n);

  // Kabsch/Horn: SVD(H)=U*S*V^T, R=V*diag(1,1,det(V*U^T))*U^T (rules out reflection).
  Eigen::Matrix3d cross_covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d p = estimated[i].translation() - centroid_est;
    const Eigen::Vector3d q = ground_truth[i].translation() - centroid_gt;
    cross_covariance += p * q.transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_covariance,
                                               Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
    d(2, 2) = -1.0;
  }
  const Eigen::Matrix3d rotation = svd.matrixV() * d * svd.matrixU().transpose();
  const Eigen::Vector3d translation = centroid_gt - rotation * centroid_est;

  const Sophus::SE3d alignment(Eigen::Quaterniond(rotation).normalized(), translation);

  double sum_squared_error = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d aligned = alignment * estimated[i].translation();
    sum_squared_error += (ground_truth[i].translation() - aligned).squaredNorm();
  }

  AbsoluteTrajectoryError result;
  result.rmse_m = std::sqrt(sum_squared_error / static_cast<double>(n));
  result.alignment = alignment;
  return result;
}

KittiOdometryError ComputeKittiOdometryError(const std::vector<Sophus::SE3d>& estimated,
                                              const std::vector<Sophus::SE3d>& ground_truth) {
  if (estimated.size() != ground_truth.size()) {
    throw std::invalid_argument(
        "ComputeKittiOdometryError: estimated and ground_truth must be the same length");
  }

  const std::vector<double> cumulative_length = CumulativePathLength(ground_truth);
  const std::size_t n = ground_truth.size();

  double sum_translation_error_ratio = 0.0;
  double sum_rotation_error_rad_per_m = 0.0;
  std::size_t num_segments = 0;

  for (std::size_t start = 0; start < n; ++start) {
    for (const double segment_length : kSegmentLengthsM) {
      const std::size_t end = FindEndFrame(cumulative_length, start, segment_length);
      if (end >= n) continue;  // not enough trajectory left for this segment length

      const Sophus::SE3d gt_relative = ground_truth[start].inverse() * ground_truth[end];
      const Sophus::SE3d est_relative = estimated[start].inverse() * estimated[end];
      const Sophus::SE3d error = gt_relative.inverse() * est_relative;

      sum_translation_error_ratio += error.translation().norm() / segment_length;
      sum_rotation_error_rad_per_m += error.so3().log().norm() / segment_length;
      ++num_segments;
    }
  }

  KittiOdometryError result;
  result.num_segments_evaluated = num_segments;
  if (num_segments > 0) {
    result.avg_translation_error_percent =
        (sum_translation_error_ratio / static_cast<double>(num_segments)) * 100.0;
    result.avg_rotation_error_deg_per_100m =
        (sum_rotation_error_rad_per_m / static_cast<double>(num_segments)) * (180.0 / kPi) * 100.0;
  }
  return result;
}

}  // namespace slam::eval
