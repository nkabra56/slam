#include "slam/frontend_vio/feature_tracker.hpp"

#include <cmath>
#include <unordered_map>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

namespace slam::frontend_vio {
namespace {

cv::Mat MakeCheckerboard(int size, int square) {
  cv::Mat image(size, size, CV_8UC1, cv::Scalar(255));
  for (int y = 0; y < size; y += square) {
    for (int x = 0; x < size; x += square) {
      if (((x / square) + (y / square)) % 2 == 0) {
        cv::rectangle(image, cv::Rect(x, y, square, square), cv::Scalar(0), -1);
      }
    }
  }
  return image;
}

TEST(FeatureTracker, DetectsCornersOnFirstFrame) {
  const cv::Mat image = MakeCheckerboard(200, 20);
  FeatureTracker tracker;
  const auto& tracks = tracker.Track(image);
  EXPECT_GT(tracks.size(), 20u);
}

TEST(FeatureTracker, TracksFollowKnownTranslation) {
  const cv::Mat image = MakeCheckerboard(200, 20);
  cv::Mat shifted;
  const cv::Mat translation_matrix = (cv::Mat_<double>(2, 3) << 1, 0, 5, 0, 1, 0);
  cv::warpAffine(image, shifted, translation_matrix, image.size());

  FeatureTracker tracker;
  const auto initial_tracks = tracker.Track(image);
  ASSERT_GT(initial_tracks.size(), 10u);

  std::unordered_map<TrackId, cv::Point2f> initial_positions;
  for (const auto& t : initial_tracks) {
    initial_positions[t.id] = t.point;
  }

  const auto& tracked = tracker.Track(shifted);

  int matched_expected_shift = 0;
  for (const auto& t : tracked) {
    const auto it = initial_positions.find(t.id);
    if (it == initial_positions.end()) continue;  // newly detected, not from frame 1
    const double dx = t.point.x - it->second.x;
    const double dy = t.point.y - it->second.y;
    if (std::abs(dx - 5.0) < 1.0 && std::abs(dy) < 1.0) {
      ++matched_expected_shift;
    }
  }
  EXPECT_GT(matched_expected_shift, 5);
}

TEST(FeatureTracker, TrackCountStaysWithinMaxCorners) {
  // A fine checkerboard has far more candidate corners (~361) than
  // max_corners, so the first call should fill up close to the cap.
  const cv::Mat image = MakeCheckerboard(200, 10);
  FeatureTracker::Params params;
  params.max_corners = 120;
  params.min_tracks = 100;
  FeatureTracker tracker(params);

  const auto& first = tracker.Track(image);
  EXPECT_LE(first.size(), static_cast<std::size_t>(params.max_corners));
  EXPECT_GE(first.size(), 100u);

  // Re-tracking the same, unmoved image should keep every track alive
  // (zero optical flow) without exceeding the cap.
  const auto& second = tracker.Track(image);
  EXPECT_LE(second.size(), static_cast<std::size_t>(params.max_corners));
  EXPECT_GE(second.size(), 100u);
}

}  // namespace
}  // namespace slam::frontend_vio
