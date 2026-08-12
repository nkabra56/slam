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

struct StereoFrame {
  TimestampSec timestamp{};
  cv::Mat left;
  cv::Mat right;
};

struct CameraIntrinsics {
  double fx{1.0};
  double fy{1.0};
  double cx{0.0};
  double cy{0.0};
};

struct StereoCalibration {
  CameraIntrinsics left;
  double baseline_m{};  // left-to-right camera baseline, meters
};

}  // namespace slam
