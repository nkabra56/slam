#include "slam/backend/vio_initializer.hpp"

#include <Eigen/Dense>

namespace slam::backend {

namespace {

constexpr int kGyroBiasIterations = 10;
constexpr double kGyroBiasNumericalStep = 1e-6;

// Small (3-unknown) Gauss-Newton solve for the constant gyro bias that
// best explains the mismatch between each pair's preintegrated rotation
// and the poses' actual relative rotation. Numeric Jacobian, same
// reasoning as everywhere else in this project that touches an
// SE3/SO3-adjacent derivative -- see imu_factor.hpp's class doc comment.
Eigen::Vector3d EstimateGyroBias(const std::vector<Sophus::SE3d>& poses,
                                  const std::vector<ImuPreintegration>& preintegrations) {
  const std::size_t n = preintegrations.size();

  const auto residual = [&](const Eigen::Vector3d& bias) {
    Eigen::VectorXd r(static_cast<int>(3 * n));
    for (std::size_t i = 0; i < n; ++i) {
      const ImuPreintegration corrected =
          preintegrations[i].BiasCorrected(ImuBias{bias, Eigen::Vector3d::Zero()});
      const Sophus::SO3d delta_actual = poses[i].so3().inverse() * poses[i + 1].so3();
      const Sophus::SO3d error = corrected.DeltaRotation().inverse() * delta_actual;
      r.segment<3>(static_cast<int>(3 * i)) = error.log();
    }
    return r;
  };

  Eigen::Vector3d bias = Eigen::Vector3d::Zero();
  for (int iter = 0; iter < kGyroBiasIterations; ++iter) {
    const Eigen::VectorXd r = residual(bias);

    Eigen::MatrixXd J(static_cast<int>(3 * n), 3);
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d db = Eigen::Vector3d::Zero();
      db(k) = kGyroBiasNumericalStep;
      const Eigen::VectorXd r_plus = residual(bias + db);
      const Eigen::VectorXd r_minus = residual(bias - db);
      J.col(k) = (r_plus - r_minus) / (2 * kGyroBiasNumericalStep);
    }

    const Eigen::Matrix3d H = J.transpose() * J + 1e-9 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d delta = H.ldlt().solve(-J.transpose() * r);
    bias += delta;

    if (delta.norm() < 1e-12) break;
  }

  return bias;
}

}  // namespace

std::optional<VioInitializationResult> InitializeVio(
    const std::vector<Sophus::SE3d>& poses, const std::vector<ImuPreintegration>& preintegrations) {
  if (poses.size() < 3 || preintegrations.size() != poses.size() - 1) {
    return std::nullopt;
  }

  const std::size_t n = preintegrations.size();  // number of consecutive pairs
  const std::size_t num_poses = poses.size();

  const Eigen::Vector3d gyro_bias = EstimateGyroBias(poses, preintegrations);

  std::vector<ImuPreintegration> corrected;
  corrected.reserve(n);
  for (const auto& p : preintegrations) {
    corrected.push_back(p.BiasCorrected(ImuBias{gyro_bias, Eigen::Vector3d::Zero()}));
  }

  // Linear system in unknowns x = [v_0, v_1, ..., v_{N-1}, gravity], from
  // the (noise-free) IMU kinematics relations -- see ComputeImuFactorResidual
  // in imu_factor.cpp for the same relations this is the linear inverse of:
  //   v_{i+1} - v_i - g*dt        = R_i * Delta_v_i               (Eq A)
  //   v_i*dt  + 0.5*g*dt^2        = (p_{i+1}-p_i) - R_i*Delta_p_i (Eq B)
  const int num_unknowns = static_cast<int>(3 * num_poses + 3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<int>(6 * n), num_unknowns);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(static_cast<int>(6 * n));

  for (std::size_t i = 0; i < n; ++i) {
    const double dt = corrected[i].DeltaTime();
    if (dt <= 0.0) return std::nullopt;

    const Eigen::Matrix3d R_i = poses[i].rotationMatrix();
    const int row_a = static_cast<int>(6 * i);
    const int row_b = row_a + 3;
    const int col_vi = static_cast<int>(3 * i);
    const int col_vi1 = static_cast<int>(3 * (i + 1));
    const int col_g = static_cast<int>(3 * num_poses);

    A.block<3, 3>(row_a, col_vi) = -Eigen::Matrix3d::Identity();
    A.block<3, 3>(row_a, col_vi1) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(row_a, col_g) = -dt * Eigen::Matrix3d::Identity();
    rhs.segment<3>(row_a) = R_i * corrected[i].DeltaVelocity();

    A.block<3, 3>(row_b, col_vi) = dt * Eigen::Matrix3d::Identity();
    A.block<3, 3>(row_b, col_g) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
    rhs.segment<3>(row_b) =
        (poses[i + 1].translation() - poses[i].translation()) - R_i * corrected[i].DeltaPosition();
  }

  const Eigen::VectorXd solution = A.colPivHouseholderQr().solve(rhs);

  VioInitializationResult result;
  result.bias_gyro = gyro_bias;
  result.velocities.reserve(num_poses);
  for (std::size_t i = 0; i < num_poses; ++i) {
    result.velocities.push_back(solution.segment<3>(static_cast<int>(3 * i)));
  }
  result.gravity = solution.segment<3>(static_cast<int>(3 * num_poses));

  const double gravity_norm = result.gravity.norm();
  if (gravity_norm < 5.0 || gravity_norm > 15.0) {
    return std::nullopt;  // degenerate/unreliable solve -- see header doc comment
  }

  return result;
}

}  // namespace slam::backend
