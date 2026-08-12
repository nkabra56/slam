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

// Detects Shi-Tomasi corners and tracks them frame-to-frame with pyramidal
// KLT optical flow, replenishing new corners whenever the live track count
// drops below a minimum. Track IDs are stable across calls to Track(), so
// callers can match a feature's 3D landmark (from a previous frame) to its
// current 2D observation.
class FeatureTracker {
 public:
  struct Params {
    int max_corners = 400;
    double quality_level = 0.01;
    double min_distance = 8.0;
    int min_tracks = 150;
  };

  explicit FeatureTracker(Params params = {});

  // Tracks existing features into `image` and tops up with fresh corners
  // when the live count falls below Params::min_tracks. Returns the live
  // tracks after this update.
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
