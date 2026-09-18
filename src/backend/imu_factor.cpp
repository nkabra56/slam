#include "slam/backend/imu_factor.hpp"

#include <utility>

namespace slam::backend {

ImuPreintegration::ImuPreintegration(ImuBias linearization_bias)
    : linearization_bias_(std::move(linearization_bias)) {}

void ImuPreintegration::Integrate(const ImuMeasurement& measurement) {
  raw_measurements_.push_back(measurement);

  if (!has_previous_) {
    previous_ = measurement;
    has_previous_ = true;
    return;
  }

  const double dt = measurement.timestamp - previous_.timestamp;
  if (dt <= 0.0) {
    previous_ = measurement;
    return;
  }

  const Eigen::Vector3d gyro = previous_.angular_velocity - linearization_bias_.gyro;
  const Eigen::Vector3d accel = previous_.linear_acceleration - linearization_bias_.accel;

  delta_position_ += delta_velocity_ * dt + 0.5 * (delta_rotation_ * accel) * dt * dt;
  delta_velocity_ += (delta_rotation_ * accel) * dt;
  delta_rotation_ = delta_rotation_ * Sophus::SO3d::exp(gyro * dt);
  delta_time_ += dt;

  previous_ = measurement;
}

ImuPreintegration ImuPreintegration::BiasCorrected(const ImuBias& new_bias) const {
  ImuPreintegration result(new_bias);
  for (const auto& m : raw_measurements_) {
    result.Integrate(m);
  }
  return result;
}

ImuFactorResidual ComputeImuFactorResidual(const NavState& state_i, const NavState& state_j,
                                            const ImuPreintegration& preintegration,
                                            const Eigen::Vector3d& gravity) {
  const double dt = preintegration.DeltaTime();
  const Eigen::Matrix3d R_i = state_i.pose.rotationMatrix();

  const ImuBias current_bias{state_i.bias_gyro, state_i.bias_accel};
  const ImuPreintegration corrected = preintegration.BiasCorrected(current_bias);

  const Sophus::SO3d delta_rotation_actual = state_i.pose.so3().inverse() * state_j.pose.so3();
  const Sophus::SO3d rotation_error = corrected.DeltaRotation().inverse() * delta_rotation_actual;

  const Eigen::Vector3d velocity_predicted =
      R_i.transpose() * (state_j.velocity - state_i.velocity - gravity * dt);
  const Eigen::Vector3d position_predicted =
      R_i.transpose() * (state_j.pose.translation() - state_i.pose.translation() -
                          state_i.velocity * dt - 0.5 * gravity * dt * dt);

  ImuFactorResidual residual;
  residual.motion.segment<3>(0) = rotation_error.log();
  residual.motion.segment<3>(3) = velocity_predicted - corrected.DeltaVelocity();
  residual.motion.segment<3>(6) = position_predicted - corrected.DeltaPosition();

  residual.bias_random_walk.segment<3>(0) = state_j.bias_gyro - state_i.bias_gyro;
  residual.bias_random_walk.segment<3>(3) = state_j.bias_accel - state_i.bias_accel;

  return residual;
}

}  // namespace slam::backend
