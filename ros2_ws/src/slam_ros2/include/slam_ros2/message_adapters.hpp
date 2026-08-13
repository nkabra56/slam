#pragma once

#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "slam/common/types.hpp"

namespace slam_ros2 {

// Pure conversion functions between ROS2 message types and slam_core's
// plain structs -- deliberately free of any ROS2 node/runtime state, so
// they're unit-testable by hand-building messages (see
// test/test_message_adapters.cpp), the same way the rest of this project
// tests things with synthetic fixtures rather than a live system.
//
// This file is the most confidently-written part of PHASE6_PLAN.md's Part
// B: it's pure functions over well-documented, long-stable message
// definitions. slam_node.hpp/.cpp, which combines several ROS2 library
// APIs (rclcpp, message_filters, tf2_ros, cv_bridge) into a running node,
// carries meaningfully more uncertainty -- see its own doc comment.

slam::ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg);
slam::ImuMeasurement ToImuMeasurement(const sensor_msgs::msg::Imu& msg);
slam::LidarScan ToLidarScan(const sensor_msgs::msg::PointCloud2& msg);

slam::CameraIntrinsics ToCameraIntrinsics(const sensor_msgs::msg::CameraInfo& msg);
// Baseline from the right camera's projection matrix: P(0,3) = -fx*baseline,
// the same relation kitti_dataset.cpp's LoadCalibration uses for KITTI's
// calib.txt (standard ROS CameraInfo / REP-104 convention).
slam::StereoCalibration ToStereoCalibration(const sensor_msgs::msg::CameraInfo& left,
                                             const sensor_msgs::msg::CameraInfo& right);

// Fixed rotation converting this project's internal camera-optical-frame
// convention (X-right, Y-down, Z-forward, inherited from KITTI/OpenCV) to
// ROS REP-103's body-frame convention (X-forward, Y-left, Z-up). See
// PHASE6_PLAN.md section 3.5 -- this is the single highest-stakes formula
// in Part B (wrong, and the trajectory silently renders sideways or
// upside-down in rviz2); OpticalToBaseRotationTest in
// test_message_adapters.cpp checks it against its own defining property,
// but that's still not the same as a real visual check. Verify one before
// trusting this in practice.
Eigen::Matrix3d OpticalToBaseRotation();
Sophus::SE3d ToRosConvention(const Sophus::SE3d& pose_optical);

nav_msgs::msg::Odometry ToOdometryMsg(const Sophus::SE3d& pose, const Eigen::Vector3d& velocity,
                                       const std::string& frame_id,
                                       const std::string& child_frame_id, rclcpp::Time stamp);

// Points must be in this project's internal (optical-frame) world
// coordinates; converted to ROS convention via OpticalToBaseRotation
// before being packed into the message.
sensor_msgs::msg::PointCloud2 ToPointCloud2(const std::vector<Eigen::Vector3d>& points,
                                             const std::string& frame_id, rclcpp::Time stamp);

}  // namespace slam_ros2
