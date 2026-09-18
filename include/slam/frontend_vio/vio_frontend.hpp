#pragma once

#include <unordered_map>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/common/types.hpp"
#include "slam/frontend_vio/feature_tracker.hpp"
#include "slam/frontend_vio/imu_preintegrator.hpp"

namespace slam::frontend_vio {

// Frame-to-frame stereo VIO via triangulation + PnP/RANSAC. IMU is
// preintegrated in parallel but not fused into the pose here.
class VioFrontend {
 public:
  struct FrameResult {
    bool has_pose{false};
    Sophus::SE3d relative_pose;  // previous camera frame -> current camera frame
    int num_inliers{0};
    ImuPreintegrationResult imu_delta;
  };

  explicit VioFrontend(StereoCalibration calibration);

  FrameResult ProcessStereoFrame(const StereoFrame& frame);
  void ProcessImu(const ImuMeasurement& measurement);

  // Landmarks from the most recent stereo pair, in local camera coordinates.
  // Track IDs are only stable across short local tracks, not a long-lived identity.
  const std::unordered_map<TrackId, Eigen::Vector3d>& LastLandmarks() const {
    return previous_landmarks_;
  }

 private:
  std::unordered_map<TrackId, Eigen::Vector3d> TriangulateTracks(
      const cv::Mat& left, const cv::Mat& right,
      const std::vector<TrackedFeature>& left_tracks) const;

  StereoCalibration calibration_;
  FeatureTracker tracker_;
  ImuPreintegrator imu_preintegrator_;
  std::unordered_map<TrackId, Eigen::Vector3d> previous_landmarks_;
  bool has_previous_frame_{false};
};

}  // namespace slam::frontend_vio
