#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "slam/common/types.hpp"

namespace slam::frontend_vio {

// Triangulates a rectified stereo correspondence into the left camera frame.
// Returns nullopt for degenerate/negative/too-far disparity.
std::optional<Eigen::Vector3d> TriangulateStereoPoint(const cv::Point2f& left_point,
                                                        const cv::Point2f& right_point,
                                                        const StereoCalibration& calibration);

struct RelativePoseEstimate {
  // Maps a point expressed in the `object_points` frame into the
  // `image_points` camera frame: p_image_cam = pose * p_object_frame.
  Sophus::SE3d pose;
  int num_inliers{0};
};

// PnP+RANSAC for the observing camera's pose given 3D object_points in a
// reference frame. Returns nullopt if PnP fails or inliers are too few.
std::optional<RelativePoseEstimate> EstimateRelativePose(
    const std::vector<cv::Point3d>& object_points, const std::vector<cv::Point2f>& image_points,
    const CameraIntrinsics& intrinsics, int min_inliers = 6);

}  // namespace slam::frontend_vio
