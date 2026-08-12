#include "slam/frontend_vio/feature_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace slam::frontend_vio {

FeatureTracker::FeatureTracker(Params params) : params_(params) {}

void FeatureTracker::DetectNewCorners(const cv::Mat& image) {
  const int num_needed = params_.max_corners - static_cast<int>(tracks_.size());
  if (num_needed <= 0) {
    return;
  }

  cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(255));
  for (const auto& track : tracks_) {
    cv::circle(mask, track.point, static_cast<int>(params_.min_distance), cv::Scalar(0), -1);
  }

  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(image, corners, num_needed, params_.quality_level, params_.min_distance,
                           mask);

  for (const auto& corner : corners) {
    tracks_.push_back(TrackedFeature{next_id_++, corner});
  }
}

const std::vector<TrackedFeature>& FeatureTracker::Track(const cv::Mat& image) {
  if (prev_image_.empty() || tracks_.empty()) {
    tracks_.clear();
    DetectNewCorners(image);
    prev_image_ = image.clone();
    return tracks_;
  }

  std::vector<cv::Point2f> prev_points;
  prev_points.reserve(tracks_.size());
  for (const auto& track : tracks_) {
    prev_points.push_back(track.point);
  }

  std::vector<cv::Point2f> next_points;
  std::vector<uchar> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(prev_image_, image, prev_points, next_points, status, error);

  std::vector<TrackedFeature> survivors;
  survivors.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (!status[i]) continue;
    const cv::Point2f& p = next_points[i];
    if (p.x < 0 || p.y < 0 || p.x >= image.cols || p.y >= image.rows) continue;
    survivors.push_back(TrackedFeature{tracks_[i].id, p});
  }
  tracks_ = std::move(survivors);

  if (static_cast<int>(tracks_.size()) < params_.min_tracks) {
    DetectNewCorners(image);
  }

  prev_image_ = image.clone();
  return tracks_;
}

}  // namespace slam::frontend_vio
