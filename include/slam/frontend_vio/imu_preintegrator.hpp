#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "slam/common/types.hpp"

namespace slam::frontend_vio {

// Result of integrating a run of IMU measurements between two keyframes.
// Gravity-free and bias-free by construction (the standard preintegration
// formulation, Forster et al.): gravity compensation and bias correction
// happen when this delta is consumed downstream, not here.
struct ImuPreintegrationResult {
  Sophus::SO3d delta_rotation;
  Eigen::Vector3d delta_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_position{Eigen::Vector3d::Zero()};
  double delta_time{0.0};
};

// On-manifold IMU preintegration between two keyframes, via the discrete
// forward-Euler recursion (each sample's angular velocity/acceleration held
// constant over the interval to the next sample):
//
//   delta_R  <- delta_R * Exp(gyro * dt)
//   delta_v  <- delta_v + delta_R * accel * dt
//   delta_p  <- delta_p + delta_v * dt + 0.5 * delta_R * accel * dt^2
//
// Biases are assumed zero -- once the backend (Phase 3) makes them
// optimizable states, this class grows bias-Jacobian propagation and
// re-integration on bias updates.
class ImuPreintegrator {
 public:
  ImuPreintegrator() = default;

  void Reset();

  // Measurements must be added in increasing timestamp order. The first
  // call after construction or Reset() only seeds the starting sample and
  // contributes no delta.
  void Integrate(const ImuMeasurement& measurement);

  const ImuPreintegrationResult& result() const { return result_; }

 private:
  ImuPreintegrationResult result_;
  bool has_previous_{false};
  ImuMeasurement previous_{};
};

}  // namespace slam::frontend_vio
