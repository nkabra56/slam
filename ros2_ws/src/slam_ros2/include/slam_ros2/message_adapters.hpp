#pragma once

#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>

#include "slam/common/types.hpp"

namespace slam_ros2 {

// Pure conversions between ROS2 messages and slam_core's plain structs, no
// ROS2 runtime state -- unit-testable by hand-building messages.

slam::ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg);
slam::ImuMeasurement ToImuMeasurement(const sensor_msgs::msg::Imu& msg);
slam::LidarScan ToLidarScan(const sensor_msgs::msg::PointCloud2& msg);

slam::CameraIntrinsics ToCameraIntrinsics(const sensor_msgs::msg::CameraInfo& msg);
// Baseline from P(0,3) = -fx*baseline (standard ROS CameraInfo convention).
slam::StereoCalibration ToStereoCalibration(const sensor_msgs::msg::CameraInfo& left,
                                             const sensor_msgs::msg::CameraInfo& right);

// Converts optical frame (X-right,Y-down,Z-forward) to ROS REP-103 body
// frame (X-forward,Y-left,Z-up). Verify visually in rviz2.
Eigen::Matrix3d OpticalToBaseRotation();
Sophus::SE3d ToRosConvention(const Sophus::SE3d& pose_optical);

nav_msgs::msg::Odometry ToOdometryMsg(const Sophus::SE3d& pose, const Eigen::Vector3d& velocity,
                                       const std::string& frame_id,
                                       const std::string& child_frame_id, rclcpp::Time stamp);

// Points must be in optical-frame world coordinates; converted to ROS
// convention internally.
sensor_msgs::msg::PointCloud2 ToPointCloud2(const std::vector<Eigen::Vector3d>& points,
                                             const std::string& frame_id, rclcpp::Time stamp);

}  // namespace slam_ros2
