#pragma once

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "slam/common/types.hpp"

namespace slam::frontend_vio {

// Triangulates a single stereo correspondence (already rectified, so the
// epipolar lines are horizontal) into a 3D point in the left camera frame.
// Returns std::nullopt for degenerate/negative/too-far disparity.
std::optional<Eigen::Vector3d> TriangulateStereoPoint(const cv::Point2f& left_point,
                                                        const cv::Point2f& right_point,
                                                        const StereoCalibration& calibration);

struct RelativePoseEstimate {
  // Maps a point expressed in the `object_points` frame into the
  // `image_points` camera frame: p_image_cam = pose * p_object_frame.
  Sophus::SE3d pose;
  int num_inliers{0};
};

// Solves PnP+RANSAC for the pose of the camera that observed `image_points`,
// given the same landmarks' 3D coordinates (`object_points`) in some
// reference camera frame. Returns std::nullopt if PnP fails or there aren't
// enough inliers to trust the estimate.
std::optional<RelativePoseEstimate> EstimateRelativePose(
    const std::vector<cv::Point3d>& object_points, const std::vector<cv::Point2f>& image_points,
    const CameraIntrinsics& intrinsics, int min_inliers = 6);

}  // namespace slam::frontend_vio
