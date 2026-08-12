#pragma once

#include <optional>

#include <sophus/se3.hpp>

#include "slam/frontend_lidar/scan_features.hpp"

namespace slam::frontend_lidar {

struct ScanMatchResult {
  // Maps a point expressed in `source`'s frame into `target`'s frame:
  // p_target = pose * p_source.
  Sophus::SE3d pose;
  int num_edge_correspondences{0};
  int num_planar_correspondences{0};
};

struct ScanMatcherParams {
  int max_iterations = 15;
  double convergence_threshold = 1e-8;  // squared-norm of the update step
  double max_edge_correspondence_dist = 1.0;    // meters
  double max_planar_correspondence_dist = 1.0;  // meters
};

// Point-to-line (edge features) / point-to-plane (planar features)
// Gauss-Newton scan matching, LOAM-style: for each source feature point,
// find its nearest neighbor(s) in the target's k-d tree (KdTree3d), form a
// line/plane residual, and solve the 6-DoF normal equations on the SE3 Lie
// algebra (Sophus) each iteration. No PCL/g2o/Ceres -- correspondence
// search and the least-squares solve are both hand-written over Eigen.
//
// `initial_guess` seeds the iteration; identity is fine for small
// inter-frame motion. Returns std::nullopt if there aren't enough total
// correspondences (< 6) to solve the 6-DoF system in some iteration.
std::optional<ScanMatchResult> MatchScans(const ScanFeatures& source, const ScanFeatures& target,
                                           const Sophus::SE3d& initial_guess = Sophus::SE3d(),
                                           const ScanMatcherParams& params = {});

}  // namespace slam::frontend_lidar
