#include "slam/frontend_vio/vio_frontend.hpp"

#include <opencv2/video/tracking.hpp>

#include "slam/frontend_vio/stereo_geometry.hpp"

namespace slam::frontend_vio {

VioFrontend::VioFrontend(StereoCalibration calibration) : calibration_(std::move(calibration)) {}

std::unordered_map<TrackId, Eigen::Vector3d> VioFrontend::TriangulateTracks(
    const cv::Mat& left, const cv::Mat& right,
    const std::vector<TrackedFeature>& left_tracks) const {
  std::unordered_map<TrackId, Eigen::Vector3d> landmarks;
  if (left_tracks.empty()) {
    return landmarks;
  }

  std::vector<cv::Point2f> left_points;
  left_points.reserve(left_tracks.size());
  for (const auto& track : left_tracks) {
    left_points.push_back(track.point);
  }

  // Rectified stereo pair: match left features into the right image with
  // the same pyramidal KLT used for temporal tracking. This assumes
  // moderate disparities (fine for KITTI's driving scenes); a stricter
  // epipolar-constrained search is a reasonable follow-up, not required for
  // this phase's raw trajectory.
  std::vector<cv::Point2f> right_points;
  std::vector<uchar> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(left, right, left_points, right_points, status, error);

  for (std::size_t i = 0; i < left_tracks.size(); ++i) {
    if (!status[i]) continue;
    if (const auto point = TriangulateStereoPoint(left_points[i], right_points[i], calibration_)) {
      landmarks.emplace(left_tracks[i].id, *point);
    }
  }
  return landmarks;
}

VioFrontend::FrameResult VioFrontend::ProcessStereoFrame(const StereoFrame& frame) {
  FrameResult result;
  result.imu_delta = imu_preintegrator_.result();
  imu_preintegrator_.Reset();

  const auto& tracks = tracker_.Track(frame.left);

  if (has_previous_frame_) {
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2f> image_points;
    for (const auto& track : tracks) {
      const auto it = previous_landmarks_.find(track.id);
      if (it == previous_landmarks_.end()) continue;
      object_points.emplace_back(it->second.x(), it->second.y(), it->second.z());
      image_points.push_back(track.point);
    }

    if (const auto estimate = EstimateRelativePose(object_points, image_points, calibration_.left)) {
      result.relative_pose = estimate->pose;
      result.num_inliers = estimate->num_inliers;
      result.has_pose = true;
    }
  }

  previous_landmarks_ = TriangulateTracks(frame.left, frame.right, tracks);
  has_previous_frame_ = true;
  return result;
}

void VioFrontend::ProcessImu(const ImuMeasurement& measurement) {
  imu_preintegrator_.Integrate(measurement);
}

}  // namespace slam::frontend_vio
