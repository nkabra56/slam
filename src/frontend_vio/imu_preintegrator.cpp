#include "slam/frontend_vio/imu_preintegrator.hpp"

namespace slam::frontend_vio {

void ImuPreintegrator::Reset() {
  result_ = ImuPreintegrationResult{};
  has_previous_ = false;
  previous_ = ImuMeasurement{};
}

void ImuPreintegrator::Integrate(const ImuMeasurement& measurement) {
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

  const Eigen::Vector3d& gyro = previous_.angular_velocity;
  const Eigen::Vector3d& accel = previous_.linear_acceleration;

  result_.delta_position +=
      result_.delta_velocity * dt + 0.5 * (result_.delta_rotation * accel) * dt * dt;
  result_.delta_velocity += (result_.delta_rotation * accel) * dt;
  result_.delta_rotation = result_.delta_rotation * Sophus::SO3d::exp(gyro * dt);
  result_.delta_time += dt;

  previous_ = measurement;
}

}  // namespace slam::frontend_vio
