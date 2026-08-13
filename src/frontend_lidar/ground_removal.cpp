#include "slam/frontend_lidar/ground_removal.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/Geometry>

namespace slam::frontend_lidar {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

std::optional<GroundRemovalResult> RemoveGround(const std::vector<Eigen::Vector3d>& points,
                                                 const GroundRemovalParams& params) {
  if (points.size() < 3) {
    return std::nullopt;
  }

  std::mt19937 rng(42);
  std::uniform_int_distribution<std::size_t> dist(0, points.size() - 1);

  Eigen::Vector3d best_normal = Eigen::Vector3d::Zero();
  double best_d = 0.0;
  std::size_t best_inliers = 0;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    const std::size_t i0 = dist(rng);
    const std::size_t i1 = dist(rng);
    const std::size_t i2 = dist(rng);
    if (i0 == i1 || i1 == i2 || i0 == i2) continue;

    const Eigen::Vector3d& p0 = points[i0];
    const Eigen::Vector3d& p1 = points[i1];
    const Eigen::Vector3d& p2 = points[i2];

    Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
    const double norm = normal.norm();
    if (norm < 1e-9) continue;
    normal /= norm;
    const double d = -normal.dot(p0);

    std::size_t inliers = 0;
    for (const auto& p : points) {
      if (std::abs(normal.dot(p) + d) < params.distance_threshold_m) ++inliers;
    }

    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_normal = normal;
      best_d = d;
    }
  }

  if (best_inliers == 0) {
    return std::nullopt;
  }

  const double inlier_ratio = static_cast<double>(best_inliers) / points.size();
  if (inlier_ratio < params.min_inlier_ratio) {
    return std::nullopt;
  }

  const double tilt_deg = std::acos(std::clamp(std::abs(best_normal.z()), 0.0, 1.0)) * 180.0 / kPi;
  if (tilt_deg > params.max_normal_tilt_deg) {
    return std::nullopt;
  }

  GroundRemovalResult result;
  result.plane = Eigen::Vector4d(best_normal.x(), best_normal.y(), best_normal.z(), best_d);
  for (const auto& p : points) {
    if (std::abs(best_normal.dot(p) + best_d) < params.distance_threshold_m) {
      result.ground_points.push_back(p);
    } else {
      result.non_ground_points.push_back(p);
    }
  }
  return result;
}

}  // namespace slam::frontend_lidar
