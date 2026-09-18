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
  // Edge matches get a Tukey biweight: those at least this far (meters) from
  // their line are ignored. Edge picks are sparse, so many have no true match.
  double edge_robust_cutoff = 0.4;
  // damping_ratio * num_residuals is added to H's diagonal each iteration --
  // scales with the problem so it suppresses degenerate directions consistently.
  double damping_ratio = 0.05;
};

// LOAM-style point-to-line/point-to-plane Gauss-Newton scan matching over
// SE3. Returns nullopt if any iteration has fewer than 6 correspondences.
std::optional<ScanMatchResult> MatchScans(const ScanFeatures& source, const ScanFeatures& target,
                                           const Sophus::SE3d& initial_guess = Sophus::SE3d(),
                                           const ScanMatcherParams& params = {});

}  // namespace slam::frontend_lidar
