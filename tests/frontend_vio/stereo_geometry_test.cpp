#include "slam/frontend_vio/stereo_geometry.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::frontend_vio {
namespace {

TEST(TriangulateStereoPoint, RecoversKnownDepth) {
  StereoCalibration calib;
  calib.left = CameraIntrinsics{500.0, 500.0, 320.0, 240.0};
  calib.baseline_m = 0.5;

  const double depth = 10.0;
  const double disparity = calib.left.fx * calib.baseline_m / depth;
  const cv::Point2f left_point(320.0f, 240.0f);
  const cv::Point2f right_point(320.0f - static_cast<float>(disparity), 240.0f);

  const auto point = TriangulateStereoPoint(left_point, right_point, calib);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->z(), depth, 1e-6);
  EXPECT_NEAR(point->x(), 0.0, 1e-6);
  EXPECT_NEAR(point->y(), 0.0, 1e-6);
}

TEST(TriangulateStereoPoint, RejectsNonPositiveDisparity) {
  StereoCalibration calib;
  calib.left = CameraIntrinsics{500.0, 500.0, 320.0, 240.0};
  calib.baseline_m = 0.5;

  EXPECT_FALSE(TriangulateStereoPoint({100, 100}, {105, 100}, calib).has_value());
}

TEST(EstimateRelativePose, RecoversKnownRigidTransform) {
  const CameraIntrinsics intrinsics{500.0, 500.0, 320.0, 240.0};

  const Sophus::SE3d ground_truth(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY())),
      Eigen::Vector3d(0.3, -0.05, 0.1));

  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2f> image_points;
  for (int i = 0; i < 20; ++i) {
    const Eigen::Vector3d p_prev(-2.0 + 0.2 * i, 0.5 * std::sin(static_cast<double>(i)), 8.0 + 0.1 * i);
    const Eigen::Vector3d p_curr = ground_truth * p_prev;
    const double u = intrinsics.fx * p_curr.x() / p_curr.z() + intrinsics.cx;
    const double v = intrinsics.fy * p_curr.y() / p_curr.z() + intrinsics.cy;
    object_points.emplace_back(p_prev.x(), p_prev.y(), p_prev.z());
    image_points.emplace_back(static_cast<float>(u), static_cast<float>(v));
  }

  const auto estimate = EstimateRelativePose(object_points, image_points, intrinsics);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_GE(estimate->num_inliers, 15);

  const Sophus::SE3d error = ground_truth.inverse() * estimate->pose;
  EXPECT_LT(error.log().norm(), 1e-3);
}

TEST(EstimateRelativePose, ReturnsNulloptWithTooFewPoints) {
  const CameraIntrinsics intrinsics{500.0, 500.0, 320.0, 240.0};
  const std::vector<cv::Point3d> object_points = {{0, 0, 5}, {1, 0, 5}};
  const std::vector<cv::Point2f> image_points = {{320, 240}, {420, 240}};

  EXPECT_FALSE(EstimateRelativePose(object_points, image_points, intrinsics).has_value());
}

}  // namespace
}  // namespace slam::frontend_vio
