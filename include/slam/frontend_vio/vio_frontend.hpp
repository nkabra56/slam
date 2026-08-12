#pragma once

#include <unordered_map>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "slam/common/types.hpp"
#include "slam/frontend_vio/feature_tracker.hpp"
#include "slam/frontend_vio/imu_preintegrator.hpp"

namespace slam::frontend_vio {

// Frame-to-frame stereo visual-inertial odometry.
//
// Each stereo pair is triangulated into 3D landmarks
// (stereo_geometry::TriangulateStereoPoint); 2D features are tracked across
// time with FeatureTracker; and the previous frame's landmarks are matched
// against the current frame's 2D observations via PnP+RANSAC
// (stereo_geometry::EstimateRelativePose) to recover a metric-scale
// relative pose -- unlike monocular VO, there is no scale ambiguity to
// resolve later.
//
// IMU measurements between frames are preintegrated in parallel
// (ImuPreintegrator) and reported alongside the visual pose, ready for the
// tightly-coupled fusion added in the Phase 3 backend; they are not fused
// into the pose estimate here.
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

  // Landmarks triangulated from the most recently processed stereo pair,
  // in that frame's local camera coordinates -- exposed so callers (e.g.
  // the Phase 4 mapping module) can accumulate a world-frame map without
  // re-triangulating. Track IDs are only stable across short local tracks
  // (FeatureTracker reassigns them as features are lost/replenished), so
  // this isn't a long-lived landmark identity, just this frame's points.
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
