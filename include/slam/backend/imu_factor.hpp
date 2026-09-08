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

// Bias-aware IMU preintegration; corrects for a changed bias by
// re-integrating raw measurements (BiasCorrected()), not an analytic Jacobian.
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

// Residual connecting NavState i->j via `preintegration` and world `gravity`.
// motion = (rotation, velocity, position error); zero when consistent.
struct ImuFactorResidual {
  Eigen::Matrix<double, 9, 1> motion{Eigen::Matrix<double, 9, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> bias_random_walk{Eigen::Matrix<double, 6, 1>::Zero()};
};

ImuFactorResidual ComputeImuFactorResidual(const NavState& state_i, const NavState& state_j,
                                            const ImuPreintegration& preintegration,
                                            const Eigen::Vector3d& gravity);

}  // namespace slam::backend
