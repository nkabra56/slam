#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "slam/backend/tightly_coupled_optimizer.hpp"
#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/mapping/map.hpp"

namespace slam_ros2 {

// Wraps the VIO/LiDAR/backend/mapping pipeline (slam_core) as a ROS2 node
// -- the live/bag-playback counterpart to the KITTI demo apps. Backed by
// TightlyCoupledOptimizer (Phase 6A, tightly_coupled_optimizer.hpp) rather
// than the loosely-coupled SlidingWindowOptimizer, so /odometry carries a
// real fused velocity once IMU-based initialization succeeds (see
// TightlyCoupledOptimizer::IsInitialized) instead of always publishing
// zero. Subscription callbacks only buffer messages; a dedicated
// processing thread does the actual SLAM work, so a slow frame never
// silently starves the executor. See PHASE6_PLAN.md section 3.4 for the
// reasoning behind that split, and section 3.5 for the coordinate-frame
// conversion this node publishes through (message_adapters.hpp's
// ToOdometryMsg/ToPointCloud2).
//
// Read this before trusting it: this file has never been built against a
// real ROS2 install, and unlike slam_core's math (which could be
// hand-derived and cross-checked against its own residual formulas), it
// combines several ROS2 library surfaces (rclcpp, message_filters,
// tf2_ros, cv_bridge) that can only be written from documented API
// knowledge, not verified the same way. Treat it as a real, complete
// attempt at the design in PHASE6_PLAN.md section 3.4 -- not as
// build-verified code the rest of this project's math generally is.
class SlamNode : public rclcpp::Node {
 public:
  SlamNode();
  ~SlamNode() override;

  SlamNode(const SlamNode&) = delete;
  SlamNode& operator=(const SlamNode&) = delete;

 private:
  struct PendingLidarScan {
    rclcpp::Time stamp;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
  };
  struct StereoJob {
    sensor_msgs::msg::Image::ConstSharedPtr left;
    sensor_msgs::msg::Image::ConstSharedPtr right;
  };

  void OnStereo(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                const sensor_msgs::msg::Image::ConstSharedPtr& right);
  void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void OnLeftCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void OnRightCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void UpdateCalibrationLocked();  // caller must hold calibration_mutex_

  void ProcessingLoop();
  sensor_msgs::msg::PointCloud2::ConstSharedPtr TakeNearestLidarScan(const rclcpp::Time& stereo_stamp);

  // --- Subscriptions ---
  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
  using StereoSyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::unique_ptr<message_filters::Synchronizer<StereoSyncPolicy>> stereo_sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;

  // --- Publishers ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // --- SLAM pipeline state -- only ever touched from processing_thread_ ---
  std::optional<slam::frontend_vio::VioFrontend> vio_;
  slam::frontend_lidar::LidarFrontend lidar_;
  slam::backend::TightlyCoupledOptimizer optimizer_;
  slam::mapping::Map map_;
  nav_msgs::msg::Path path_;

  // --- Cross-thread state ---
  std::mutex calibration_mutex_;
  std::optional<sensor_msgs::msg::CameraInfo> left_info_;
  std::optional<sensor_msgs::msg::CameraInfo> right_info_;
  std::optional<slam::StereoCalibration> calibration_;

  std::mutex lidar_mutex_;
  std::deque<PendingLidarScan> pending_lidar_scans_;

  std::mutex imu_mutex_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> pending_imu_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<StereoJob> stereo_queue_;
  static constexpr std::size_t kMaxQueueDepth = 4;
  static constexpr std::size_t kMaxLidarBuffer = 4;

  std::atomic<bool> shutdown_{false};
  std::thread processing_thread_;

  std::string map_frame_id_;
  std::string base_frame_id_;
  double lidar_time_tolerance_s_{0.05};
};

}  // namespace slam_ros2
