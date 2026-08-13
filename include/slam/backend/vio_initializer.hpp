#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/backend/imu_factor.hpp"

namespace slam::backend {

struct VioInitializationResult {
  Eigen::Vector3d gravity{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> velocities;  // one per input pose, same order/count
  Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
  // Accel bias is intentionally left at zero -- poorly observable without
  // deliberate acceleration excitation; the online optimizer (NavStateGraph)
  // refines it. See PHASE6_PLAN.md section 2.4.
};

// Recovers initial per-keyframe velocity, gravity, and gyroscope bias from
// a short window of already-metric-scale poses (from VioFrontend or
// LidarFrontend -- this project's odometry is never scale-ambiguous, so
// unlike monocular VIO initialization (e.g. VINS-Mono), no scale unknown
// needs solving for here) and the raw (zero-bias) IMU preintegration
// between each consecutive pair. See PHASE6_PLAN.md section 2.4.
//
// Two-step procedure:
//   1. Gyro bias: a small (3-parameter) numeric Gauss-Newton minimizing
//      the rotation-only mismatch between each pair's preintegrated
//      rotation and the poses' actual relative rotation (re-integrating
//      via ImuPreintegration::BiasCorrected at each iteration, per the
//      same numeric-over-analytic-Jacobian choice imu_factor.hpp makes).
//   2. Gravity + per-keyframe velocity: with gyro bias applied (accel
//      bias assumed zero), the velocity/position preintegration relations
//      are LINEAR in the unknowns (v_0, ..., v_N, gravity), so this is
//      one least-squares solve, no iteration needed.
//
// `poses.size()` must equal `preintegrations.size() + 1`. Returns
// std::nullopt if there are fewer than 3 poses (not enough segments to
// solve the gravity/velocity system), the sizes don't match, or the
// solved gravity vector's magnitude isn't physically plausible (outside
// roughly 5-15 m/s^2) -- the latter is a real, if crude, guard against
// trusting a degenerate/under-constrained solve.
std::optional<VioInitializationResult> InitializeVio(
    const std::vector<Sophus::SE3d>& poses, const std::vector<ImuPreintegration>& preintegrations);

}  // namespace slam::backend
