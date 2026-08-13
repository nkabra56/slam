#include "slam_ros2/slam_node.hpp"

#include <cmath>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "slam_ros2/message_adapters.hpp"

namespace slam_ros2 {

SlamNode::SlamNode() : rclcpp::Node("slam_node") {
  map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  lidar_time_tolerance_s_ = declare_parameter<double>("lidar_time_tolerance_s", 0.05);

  const auto sensor_qos = rclcpp::SensorDataQoS();

  left_image_sub_.subscribe(this, "left/image_raw", sensor_qos.get_rmw_qos_profile());
  right_image_sub_.subscribe(this, "right/image_raw", sensor_qos.get_rmw_qos_profile());
  stereo_sync_ = std::make_unique<message_filters::Synchronizer<StereoSyncPolicy>>(
      StereoSyncPolicy(10), left_image_sub_, right_image_sub_);
  stereo_sync_->registerCallback(
      std::bind(&SlamNode::OnStereo, this, std::placeholders::_1, std::placeholders::_2));

  left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "left/camera_info", 1,
      std::bind(&SlamNode::OnLeftCameraInfo, this, std::placeholders::_1));
  right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "right/camera_info", 1,
      std::bind(&SlamNode::OnRightCameraInfo, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", sensor_qos, std::bind(&SlamNode::OnImu, this, std::placeholders::_1));
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", sensor_qos, std::bind(&SlamNode::OnPointCloud, this, std::placeholders::_1));

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_.header.frame_id = map_frame_id_;

  processing_thread_ = std::thread(&SlamNode::ProcessingLoop, this);
}

SlamNode::~SlamNode() {
  shutdown_ = true;
  queue_cv_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

void SlamNode::OnStereo(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                         const sensor_msgs::msg::Image::ConstSharedPtr& right) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (stereo_queue_.size() >= kMaxQueueDepth) {
    RCLCPP_WARN(get_logger(), "SLAM processing is falling behind -- dropping a stereo frame");
    stereo_queue_.pop_front();
  }
  stereo_queue_.push_back(StereoJob{left, right});
  queue_cv_.notify_one();
}

void SlamNode::OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  pending_imu_.push_back(msg);
}

void SlamNode::OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  pending_lidar_scans_.push_back(PendingLidarScan{rclcpp::Time(msg->header.stamp), msg});
  while (pending_lidar_scans_.size() > kMaxLidarBuffer) {
    pending_lidar_scans_.pop_front();
  }
}

void SlamNode::OnLeftCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  left_info_ = *msg;
  UpdateCalibrationLocked();
}

void SlamNode::OnRightCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  right_info_ = *msg;
  UpdateCalibrationLocked();
}

void SlamNode::UpdateCalibrationLocked() {
  if (left_info_.has_value() && right_info_.has_value() && !calibration_.has_value()) {
    calibration_ = ToStereoCalibration(*left_info_, *right_info_);
    RCLCPP_INFO(get_logger(), "Stereo calibration received (fx=%.2f, baseline=%.4fm)",
                calibration_->left.fx, calibration_->baseline_m);
  }
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr SlamNode::TakeNearestLidarScan(
    const rclcpp::Time& stereo_stamp) {
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  if (pending_lidar_scans_.empty()) return nullptr;

  auto best = pending_lidar_scans_.begin();
  double best_diff = std::abs((best->stamp - stereo_stamp).seconds());
  for (auto it = pending_lidar_scans_.begin(); it != pending_lidar_scans_.end(); ++it) {
    const double diff = std::abs((it->stamp - stereo_stamp).seconds());
    if (diff < best_diff) {
      best = it;
      best_diff = diff;
    }
  }
  if (best_diff > lidar_time_tolerance_s_) return nullptr;

  const auto msg = best->msg;
  pending_lidar_scans_.erase(pending_lidar_scans_.begin(), std::next(best));
  return msg;
}

void SlamNode::ProcessingLoop() {
  while (!shutdown_) {
    StereoJob job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return shutdown_.load() || !stereo_queue_.empty(); });
      if (shutdown_) return;
      job = stereo_queue_.front();
      stereo_queue_.pop_front();
    }

    std::optional<slam::StereoCalibration> calibration;
    {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      calibration = calibration_;
    }
    if (!calibration.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "No camera_info received yet -- dropping stereo frame");
      continue;
    }
    if (!vio_.has_value()) {
      vio_.emplace(*calibration);
    }

    const rclcpp::Time stamp(job.left->header.stamp);

    // Feed every buffered IMU sample up to this frame's timestamp before
    // processing the frame -- matches VioFrontend::ProcessStereoFrame's
    // calling convention (see the KITTI demo apps: ProcessImu, then
    // ProcessStereoFrame, so the returned imu_delta covers exactly the
    // interval since the previous frame).
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      while (!pending_imu_.empty() && rclcpp::Time(pending_imu_.front()->header.stamp) <= stamp) {
        vio_->ProcessImu(ToImuMeasurement(*pending_imu_.front()));
        pending_imu_.pop_front();
      }
    }

    const slam::ImageFrame left_frame = ToImageFrame(*job.left);
    const slam::ImageFrame right_frame = ToImageFrame(*job.right);
    slam::StereoFrame stereo_frame;
    stereo_frame.timestamp = left_frame.timestamp;
    stereo_frame.left = left_frame.image;
    stereo_frame.right = right_frame.image;
    const auto vio_result = vio_->ProcessStereoFrame(stereo_frame);

    slam::backend::SlidingWindowOptimizer::EdgeMeasurement vio_edge;
    vio_edge.valid = vio_result.has_pose;
    vio_edge.relative_pose = vio_result.relative_pose;
    vio_edge.num_matches = vio_result.num_inliers;

    slam::backend::SlidingWindowOptimizer::EdgeMeasurement lidar_edge;
    std::optional<slam::frontend_lidar::ScanFeatures> lidar_features_for_map;

    if (const auto scan_msg = TakeNearestLidarScan(stamp)) {
      const slam::LidarScan scan = ToLidarScan(*scan_msg);
      const auto lidar_result = lidar_.ProcessScan(scan);
      lidar_edge.valid = lidar_result.has_pose;
      lidar_edge.relative_pose = lidar_result.relative_pose;
      lidar_edge.num_matches =
          lidar_result.num_edge_correspondences + lidar_result.num_planar_correspondences;
      lidar_features_for_map = lidar_.LastFeatures();
    }

    const auto node_id =
        optimizer_.AddKeyframe(vio_edge, lidar_edge, std::nullopt, lidar_features_for_map);
    const Sophus::SE3d pose = optimizer_.PoseOf(node_id);

    if (lidar_features_for_map.has_value()) {
      std::vector<Eigen::Vector3d> world_points;
      world_points.reserve(lidar_features_for_map->edge_points.size() +
                            lidar_features_for_map->planar_points.size());
      for (const auto& p : lidar_features_for_map->edge_points) world_points.push_back(pose * p);
      for (const auto& p : lidar_features_for_map->planar_points) world_points.push_back(pose * p);
      map_.InsertLidarPoints(world_points);
    }
    std::vector<Eigen::Vector3d> world_landmarks;
    world_landmarks.reserve(vio_->LastLandmarks().size());
    for (const auto& [track_id, p] : vio_->LastLandmarks()) world_landmarks.push_back(pose * p);
    map_.InsertLandmarks(world_landmarks);

    const auto odometry_msg =
        ToOdometryMsg(pose, Eigen::Vector3d::Zero(), map_frame_id_, base_frame_id_, stamp);
    odometry_pub_->publish(odometry_msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = odometry_msg.pose.pose.position.x;
    transform.transform.translation.y = odometry_msg.pose.pose.position.y;
    transform.transform.translation.z = odometry_msg.pose.pose.position.z;
    transform.transform.rotation = odometry_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = odometry_msg.header;
    path_pose.pose = odometry_msg.pose.pose;
    path_.header.stamp = stamp;
    path_.poses.push_back(path_pose);
    path_pub_->publish(path_);

    const auto landmark_points = map_.LandmarkMap().Points();
    const auto lidar_points = map_.LidarMap().Points();
    std::vector<Eigen::Vector3d> all_points;
    all_points.reserve(landmark_points.size() + lidar_points.size());
    all_points.insert(all_points.end(), landmark_points.begin(), landmark_points.end());
    all_points.insert(all_points.end(), lidar_points.begin(), lidar_points.end());
    map_pub_->publish(ToPointCloud2(all_points, map_frame_id_, stamp));
  }
}

}  // namespace slam_ros2
