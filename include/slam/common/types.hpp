#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace slam {

using TimestampSec = double;

struct ImuMeasurement {
  TimestampSec timestamp{};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
};

struct ImageFrame {
  TimestampSec timestamp{};
  cv::Mat image;
};

struct LidarPoint {
  float x{};
  float y{};
  float z{};
  float intensity{};
};

struct LidarScan {
  TimestampSec timestamp{};
  std::vector<LidarPoint> points;
};

}  // namespace slam
