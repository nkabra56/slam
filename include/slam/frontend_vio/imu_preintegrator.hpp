#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "slam/common/types.hpp"

namespace slam::frontend_vio {

// Result of integrating IMU between two keyframes. Gravity/bias-free by
// construction -- corrected when consumed downstream.
struct ImuPreintegrationResult {
  Sophus::SO3d delta_rotation;
  Eigen::Vector3d delta_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_position{Eigen::Vector3d::Zero()};
  double delta_time{0.0};
};

// On-manifold IMU preintegration via forward-Euler. Zero-bias; see
// backend::ImuPreintegration for the bias-aware version.
class ImuPreintegrator {
 public:
  ImuPreintegrator() = default;

  void Reset();

  // Like Reset(), but keeps the last sample as the next window's seed --
  // needed when feeding one IMU sample per frame.
  void ResetKeepingSeed();

  // Must be added in increasing timestamp order; the first call after
  // construction/Reset() only seeds, no delta.
  void Integrate(const ImuMeasurement& measurement);

  const ImuPreintegrationResult& result() const { return result_; }

 private:
  ImuPreintegrationResult result_;
  bool has_previous_{false};
  ImuMeasurement previous_{};
};

}  // namespace slam::frontend_vio
