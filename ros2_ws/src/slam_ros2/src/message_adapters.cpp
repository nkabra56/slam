#include "slam_ros2/message_adapters.hpp"

#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace slam_ros2 {

slam::ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception& e) {
    throw std::runtime_error(std::string("ToImageFrame: cv_bridge conversion failed: ") + e.what());
  }

  slam::ImageFrame frame;
  frame.timestamp = rclcpp::Time(msg.header.stamp).seconds();
  frame.image = cv_ptr->image;
  return frame;
}

slam::ImuMeasurement ToImuMeasurement(const sensor_msgs::msg::Imu& msg) {
  slam::ImuMeasurement measurement;
  measurement.timestamp = rclcpp::Time(msg.header.stamp).seconds();
  measurement.angular_velocity =
      Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  measurement.linear_acceleration = Eigen::Vector3d(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  return measurement;
}

slam::LidarScan ToLidarScan(const sensor_msgs::msg::PointCloud2& msg) {
  slam::LidarScan scan;
  scan.timestamp = rclcpp::Time(msg.header.stamp).seconds();
  scan.points.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));

  bool has_intensity = false;
  for (const auto& field : msg.fields) {
    if (field.name == "intensity") {
      has_intensity = true;
      break;
    }
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  if (has_intensity) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_i(msg, "intensity");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
      slam::LidarPoint p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      p.intensity = *iter_i;
      scan.points.push_back(p);
    }
  } else {
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      slam::LidarPoint p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      p.intensity = 0.0f;
      scan.points.push_back(p);
    }
  }

  return scan;
}

slam::CameraIntrinsics ToCameraIntrinsics(const sensor_msgs::msg::CameraInfo& msg) {
  slam::CameraIntrinsics intrinsics;
  intrinsics.fx = msg.k[0];
  intrinsics.fy = msg.k[4];
  intrinsics.cx = msg.k[2];
  intrinsics.cy = msg.k[5];
  return intrinsics;
}

slam::StereoCalibration ToStereoCalibration(const sensor_msgs::msg::CameraInfo& left,
                                             const sensor_msgs::msg::CameraInfo& right) {
  slam::StereoCalibration calibration;
  calibration.left = ToCameraIntrinsics(left);
  calibration.baseline_m = -right.p[3] / right.p[0];
  return calibration;
}

Eigen::Matrix3d OpticalToBaseRotation() {
  Eigen::Matrix3d R;
  // base_x (forward) = optical_z (forward)
  // base_y (left)    = -optical_x (right)
  // base_z (up)      = -optical_y (down)
  // clang-format off
  R <<  0,  0,  1,
       -1,  0,  0,
        0, -1,  0;
  // clang-format on
  return R;
}

Sophus::SE3d ToRosConvention(const Sophus::SE3d& pose_optical) {
  static const Sophus::SE3d kConvert(Eigen::Quaterniond(OpticalToBaseRotation()).normalized(),
                                      Eigen::Vector3d::Zero());
  return kConvert * pose_optical * kConvert.inverse();
}

nav_msgs::msg::Odometry ToOdometryMsg(const Sophus::SE3d& pose, const Eigen::Vector3d& velocity,
                                       const std::string& frame_id,
                                       const std::string& child_frame_id, rclcpp::Time stamp) {
  const Sophus::SE3d pose_ros = ToRosConvention(pose);
  const Eigen::Vector3d velocity_ros = OpticalToBaseRotation() * velocity;

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;

  const Eigen::Vector3d t = pose_ros.translation();
  msg.pose.pose.position.x = t.x();
  msg.pose.pose.position.y = t.y();
  msg.pose.pose.position.z = t.z();

  const Eigen::Quaterniond q = pose_ros.unit_quaternion();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.twist.twist.linear.x = velocity_ros.x();
  msg.twist.twist.linear.y = velocity_ros.y();
  msg.twist.twist.linear.z = velocity_ros.z();

  return msg;
}

sensor_msgs::msg::PointCloud2 ToPointCloud2(const std::vector<Eigen::Vector3d>& points,
                                             const std::string& frame_id, rclcpp::Time stamp) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  const Eigen::Matrix3d R = OpticalToBaseRotation();
  for (const auto& p : points) {
    const Eigen::Vector3d p_ros = R * p;
    *iter_x = static_cast<float>(p_ros.x());
    *iter_y = static_cast<float>(p_ros.y());
    *iter_z = static_cast<float>(p_ros.z());
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return msg;
}

}  // namespace slam_ros2
