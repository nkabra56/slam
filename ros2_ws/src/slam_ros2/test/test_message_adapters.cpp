#include "slam_ros2/message_adapters.hpp"

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace slam_ros2 {
namespace {

TEST(ToImuMeasurement, CopiesFieldsCorrectly) {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = 10;
  msg.header.stamp.nanosec = 500000000;
  msg.angular_velocity.x = 0.1;
  msg.angular_velocity.y = 0.2;
  msg.angular_velocity.z = 0.3;
  msg.linear_acceleration.x = 1.0;
  msg.linear_acceleration.y = 2.0;
  msg.linear_acceleration.z = 3.0;

  const slam::ImuMeasurement measurement = ToImuMeasurement(msg);

  EXPECT_NEAR(measurement.timestamp, 10.5, 1e-9);
  EXPECT_NEAR(measurement.angular_velocity.x(), 0.1, 1e-9);
  EXPECT_NEAR(measurement.angular_velocity.z(), 0.3, 1e-9);
  EXPECT_NEAR(measurement.linear_acceleration.z(), 3.0, 1e-9);
}

TEST(ToLidarScan, ParsesXyzIntensityFields) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = 1;
  msg.height = 1;
  msg.width = 2;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                 sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                 sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                 sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");

  *iter_x = 1.0f;
  *iter_y = 2.0f;
  *iter_z = 3.0f;
  *iter_i = 0.5f;
  ++iter_x;
  ++iter_y;
  ++iter_z;
  ++iter_i;
  *iter_x = 4.0f;
  *iter_y = 5.0f;
  *iter_z = 6.0f;
  *iter_i = 0.9f;

  const slam::LidarScan scan = ToLidarScan(msg);

  ASSERT_EQ(scan.points.size(), 2u);
  EXPECT_FLOAT_EQ(scan.points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(scan.points[0].intensity, 0.5f);
  EXPECT_FLOAT_EQ(scan.points[1].z, 6.0f);
  EXPECT_FLOAT_EQ(scan.points[1].intensity, 0.9f);
}

TEST(ToLidarScan, HandlesMissingIntensityField) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = 1;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  *iter_x = 7.0f;
  *iter_y = 8.0f;
  *iter_z = 9.0f;

  const slam::LidarScan scan = ToLidarScan(msg);
  ASSERT_EQ(scan.points.size(), 1u);
  EXPECT_FLOAT_EQ(scan.points[0].x, 7.0f);
  EXPECT_FLOAT_EQ(scan.points[0].intensity, 0.0f);
}

TEST(ToCameraIntrinsics, ExtractsFocalLengthAndPrincipalPoint) {
  sensor_msgs::msg::CameraInfo msg;
  msg.k = {700.0, 0.0, 300.0, 0.0, 700.0, 200.0, 0.0, 0.0, 1.0};

  const slam::CameraIntrinsics intrinsics = ToCameraIntrinsics(msg);
  EXPECT_DOUBLE_EQ(intrinsics.fx, 700.0);
  EXPECT_DOUBLE_EQ(intrinsics.fy, 700.0);
  EXPECT_DOUBLE_EQ(intrinsics.cx, 300.0);
  EXPECT_DOUBLE_EQ(intrinsics.cy, 200.0);
}

TEST(ToStereoCalibration, ComputesBaselineFromRightProjectionMatrix) {
  sensor_msgs::msg::CameraInfo left;
  left.k = {700.0, 0.0, 300.0, 0.0, 700.0, 200.0, 0.0, 0.0, 1.0};

  sensor_msgs::msg::CameraInfo right;
  right.p = {700.0, 0.0, 300.0, -350.0, 0.0, 700.0, 200.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  const slam::StereoCalibration calibration = ToStereoCalibration(left, right);
  EXPECT_DOUBLE_EQ(calibration.left.fx, 700.0);
  EXPECT_DOUBLE_EQ(calibration.baseline_m, 0.5);
}

// Wrong here means trajectories/maps render sideways or upside-down in rviz2.
TEST(OpticalToBaseRotation, MapsForwardAndUpAxesCorrectly) {
  const Eigen::Matrix3d R = OpticalToBaseRotation();

  const Eigen::Vector3d forward_optical(0.0, 0.0, 1.0);  // optical +Z = forward
  EXPECT_TRUE((R * forward_optical).isApprox(Eigen::Vector3d(1.0, 0.0, 0.0), 1e-9));

  const Eigen::Vector3d up_optical(0.0, -1.0, 0.0);  // optical -Y = up
  EXPECT_TRUE((R * up_optical).isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-9));

  const Eigen::Vector3d left_optical(-1.0, 0.0, 0.0);  // optical -X = left
  EXPECT_TRUE((R * left_optical).isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-9));

  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);  // must be a proper rotation
}

TEST(ToRosConvention, PreservesIdentityAtOrigin) {
  const Sophus::SE3d identity;
  const Sophus::SE3d converted = ToRosConvention(identity);
  EXPECT_TRUE(converted.translation().isApprox(Eigen::Vector3d::Zero(), 1e-9));
  EXPECT_TRUE(converted.rotationMatrix().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(ToRosConvention, ForwardOpticalTranslationBecomesForwardBaseTranslation) {
  const Sophus::SE3d forward_in_optical(Eigen::Quaterniond::Identity(),
                                         Eigen::Vector3d(0.0, 0.0, 5.0));  // 5m along optical +Z
  const Sophus::SE3d converted = ToRosConvention(forward_in_optical);
  EXPECT_TRUE(converted.translation().isApprox(Eigen::Vector3d(5.0, 0.0, 0.0), 1e-9));
}

}  // namespace
}  // namespace slam_ros2
