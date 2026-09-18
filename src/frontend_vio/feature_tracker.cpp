#include "slam/frontend_vio/feature_tracker.hpp"

#include <opencv2/imgproc.hpp>

#ifdef SLAM_USE_CUDA_OPTFLOW
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>
#else
#include <opencv2/video/tracking.hpp>
#endif

namespace slam::frontend_vio {

namespace {

// Pyramidal KLT: GPU (OpenCV cudaoptflow) when built with SLAM_USE_CUDA_OPTFLOW,
// CPU otherwise. Same next_points/status contract as cv::calcOpticalFlowPyrLK.
void TrackPoints(const cv::Mat& prev_image, const cv::Mat& image,
                  const std::vector<cv::Point2f>& prev_points,
                  std::vector<cv::Point2f>& next_points, std::vector<uchar>& status) {
#ifdef SLAM_USE_CUDA_OPTFLOW
  cv::cuda::GpuMat d_prev, d_curr;
  d_prev.upload(prev_image);
  d_curr.upload(image);

  cv::Mat prev_pts_mat(1, static_cast<int>(prev_points.size()), CV_32FC2,
                        const_cast<cv::Point2f*>(prev_points.data()));
  cv::cuda::GpuMat d_prev_pts;
  d_prev_pts.upload(prev_pts_mat);

  cv::cuda::GpuMat d_next_pts, d_status;
  static const auto klt = cv::cuda::SparsePyrLKOpticalFlow::create();
  klt->calc(d_prev, d_curr, d_prev_pts, d_next_pts, d_status);

  cv::Mat next_pts_mat, status_mat;
  d_next_pts.download(next_pts_mat);
  d_status.download(status_mat);

  next_points.assign(next_pts_mat.ptr<cv::Point2f>(),
                      next_pts_mat.ptr<cv::Point2f>() + prev_points.size());
  status.assign(status_mat.ptr<uchar>(), status_mat.ptr<uchar>() + prev_points.size());
#else
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(prev_image, image, prev_points, next_points, status, error);
#endif
}

}  // namespace

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
  TrackPoints(prev_image_, image, prev_points, next_points, status);

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
