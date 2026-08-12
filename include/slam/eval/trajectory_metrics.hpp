#pragma once

#include <cstddef>
#include <vector>

#include <sophus/se3.hpp>

namespace slam::eval {

// Root-mean-square translation error of the estimated trajectory against
// ground truth, after best-fit rigid (SE3, no scale) alignment via the
// closed-form Kabsch/Horn method. No scale factor is estimated -- our VIO
// (stereo) and LiDAR frontends already produce metric-scale poses, so a
// Sim3 alignment would hide a real scale bug rather than factor out a
// benign one. This is the "ATE RMSE" reported across the SLAM literature
// (Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems").
//
// Requires estimated.size() == ground_truth.size() and at least 3 poses
// (the alignment is underdetermined otherwise); throws std::invalid_argument
// if not.
struct AbsoluteTrajectoryError {
  double rmse_m{};
  Sophus::SE3d alignment;  // the SE3 that best maps estimated -> ground_truth
};

AbsoluteTrajectoryError ComputeAte(const std::vector<Sophus::SE3d>& estimated,
                                    const std::vector<Sophus::SE3d>& ground_truth);

// KITTI's official odometry evaluation protocol (see the devkit's
// evaluate_odometry.cpp, widely reproduced across the literature): for
// each starting frame and each segment length in {100,200,...,800} meters
// measured along the ground-truth path, finds the ending frame closest to
// that path length, compares the estimated vs. true relative pose over
// that sub-segment, and averages the translation error (as a % of segment
// length) and rotation error (as degrees per 100m) over every segment
// evaluated. No alignment step -- KITTI assumes (and this project's poses
// satisfy) pose[0] == identity for both estimated and ground truth, so
// each segment's relative-pose comparison is self-contained.
//
// Requires estimated.size() == ground_truth.size(); throws
// std::invalid_argument if not. num_segments_evaluated is 0 (not an error)
// if the trajectory is too short for any of the segment lengths to fit.
struct KittiOdometryError {
  double avg_translation_error_percent{};
  double avg_rotation_error_deg_per_100m{};
  std::size_t num_segments_evaluated{};
};

KittiOdometryError ComputeKittiOdometryError(const std::vector<Sophus::SE3d>& estimated,
                                              const std::vector<Sophus::SE3d>& ground_truth);

}  // namespace slam::eval
