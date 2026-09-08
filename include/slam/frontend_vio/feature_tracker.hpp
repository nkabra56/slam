#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace slam::frontend_vio {

using TrackId = std::uint64_t;

struct TrackedFeature {
  TrackId id{};
  cv::Point2f point;
};

// Shi-Tomasi corners tracked via pyramidal KLT, replenished below min_tracks.
// Track IDs are stable across Track() calls.
class FeatureTracker {
 public:
  struct Params {
    int max_corners = 400;
    double quality_level = 0.01;
    double min_distance = 8.0;
    int min_tracks = 150;
  };

  FeatureTracker() : FeatureTracker(Params{}) {}
  explicit FeatureTracker(Params params);

  // Tracks into `image`, tops up below min_tracks. Returns the live tracks.
  const std::vector<TrackedFeature>& Track(const cv::Mat& image);

  const std::vector<TrackedFeature>& tracks() const { return tracks_; }

 private:
  void DetectNewCorners(const cv::Mat& image);

  Params params_;
  cv::Mat prev_image_;
  std::vector<TrackedFeature> tracks_;
  TrackId next_id_{0};
};

}  // namespace slam::frontend_vio
