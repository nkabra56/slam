#include "slam/frontend_vio/stereo_geometry.hpp"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

namespace slam::frontend_vio {

std::optional<Eigen::Vector3d> TriangulateStereoPoint(const cv::Point2f& left_point,
                                                        const cv::Point2f& right_point,
                                                        const StereoCalibration& calibration) {
  const double disparity = left_point.x - right_point.x;
  if (disparity <= 1.0) {
    return std::nullopt;
  }

  const CameraIntrinsics& k = calibration.left;
  const double depth = k.fx * calibration.baseline_m / disparity;
  if (!(depth > 0.0) || depth > 200.0) {
    return std::nullopt;
  }

  const double x = (left_point.x - k.cx) * depth / k.fx;
  const double y = (left_point.y - k.cy) * depth / k.fy;
  return Eigen::Vector3d(x, y, depth);
}

std::optional<RelativePoseEstimate> EstimateRelativePose(
    const std::vector<cv::Point3d>& object_points, const std::vector<cv::Point2f>& image_points,
    const CameraIntrinsics& intrinsics, int min_inliers) {
  if (object_points.size() != image_points.size() ||
      object_points.size() < static_cast<std::size_t>(min_inliers)) {
    return std::nullopt;
  }

  const cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.cx, 0, intrinsics.fy,
                      intrinsics.cy, 0, 0, 1);

  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<int> inliers;
  const bool ok = cv::solvePnPRansac(object_points, image_points, K, cv::noArray(), rvec, tvec,
                                      false, 200, 3.0, 0.99, inliers);
  if (!ok || static_cast<int>(inliers.size()) < min_inliers) {
    return std::nullopt;
  }

  cv::Mat r_cv;
  cv::Rodrigues(rvec, r_cv);
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  for (int row = 0; row < 3; ++row) {
    t(row) = tvec.at<double>(row);
    for (int col = 0; col < 3; ++col) {
      r(row, col) = r_cv.at<double>(row, col);
    }
  }

  RelativePoseEstimate estimate;
  estimate.pose = Sophus::SE3d(Eigen::Quaterniond(r).normalized(), t);
  estimate.num_inliers = static_cast<int>(inliers.size());
  return estimate;
}

}  // namespace slam::frontend_vio
