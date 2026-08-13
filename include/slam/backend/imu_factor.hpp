#pragma once

#include <vector>

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "slam/backend/nav_state.hpp"
#include "slam/common/types.hpp"

namespace slam::backend {

struct ImuBias {
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

// Bias-aware IMU preintegration between two keyframes -- integrates
// (gyro - bias.gyro) and (accel - bias.accel) with the same
// forward-Euler recursion frontend_vio::ImuPreintegrator uses for the
// zero-bias case, but against a supplied linearization bias.
//
// Deliberately does NOT propagate an analytic bias-Jacobian the way the
// SLAM literature (Forster et al., "On-Manifold Preintegration for
// Real-Time Visual-Inertial Odometry", T-RO 2017) does. Correcting for a
// changed bias estimate is done by re-integrating the retained raw
// measurement buffer (BiasCorrected()) instead -- real runtime cost, in
// exchange for math that doesn't have to be trusted without a compiler to
// check it against. See PHASE6_PLAN.md section 2.3 for the reasoning and
// the (documented, not implemented) analytic alternative.
class ImuPreintegration {
 public:
  explicit ImuPreintegration(ImuBias linearization_bias = {});

  void Integrate(const ImuMeasurement& measurement);

  const Sophus::SO3d& DeltaRotation() const { return delta_rotation_; }
  const Eigen::Vector3d& DeltaVelocity() const { return delta_velocity_; }
  const Eigen::Vector3d& DeltaPosition() const { return delta_position_; }
  double DeltaTime() const { return delta_time_; }
  const ImuBias& LinearizationBias() const { return linearization_bias_; }

  // Re-integrates the same raw measurements against a different bias.
  // O(number of measurements integrated so far).
  ImuPreintegration BiasCorrected(const ImuBias& new_bias) const;

 private:
  ImuBias linearization_bias_;
  std::vector<ImuMeasurement> raw_measurements_;

  Sophus::SO3d delta_rotation_;
  Eigen::Vector3d delta_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_position_{Eigen::Vector3d::Zero()};
  double delta_time_{0.0};

  bool has_previous_{false};
  ImuMeasurement previous_{};
};

// Residual for the factor connecting NavState nodes i ("from") and j
// ("to") via `preintegration`. `gravity` is the fixed world-frame gravity
// vector (see VioInitializer for how it's estimated). Residual layout:
// motion = (rotation error (3), velocity error (3), position error (3));
// bias_random_walk = (gyro bias drift (3), accel bias drift (3)). Both
// are zero when state_i/state_j exactly satisfy the IMU kinematics
// implied by `preintegration` and `gravity`. See PHASE6_PLAN.md section
// 2.3 for the derivation.
struct ImuFactorResidual {
  Eigen::Matrix<double, 9, 1> motion{Eigen::Matrix<double, 9, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> bias_random_walk{Eigen::Matrix<double, 6, 1>::Zero()};
};

ImuFactorResidual ComputeImuFactorResidual(const NavState& state_i, const NavState& state_j,
                                            const ImuPreintegration& preintegration,
                                            const Eigen::Vector3d& gravity);

}  // namespace slam::backend
