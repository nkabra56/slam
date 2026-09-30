#include "slam/frontend_lidar/lidar_frontend.hpp"

#include <utility>

#include "slam/frontend_lidar/voxel_grid.hpp"

namespace slam::frontend_lidar {

namespace {

constexpr double kFeatureVoxelSizeM = 0.4;
constexpr double kNoPriorRadiusScale = 2.0;

ScanFeatures Downsample(ScanFeatures features) {
  features.edge_points = VoxelDownsample(features.edge_points, kFeatureVoxelSizeM);
  features.planar_points = VoxelDownsample(features.planar_points, kFeatureVoxelSizeM);
  return features;
}

void Transform(ScanFeatures& features, const Eigen::Isometry3d& transform) {
  for (auto& p : features.edge_points) p = transform * p;
  for (auto& p : features.planar_points) p = transform * p;
}

}  // namespace

LidarFrontend::LidarFrontend(FeatureExtractionParams feature_params, ScanMatcherParams matcher_params,
                              Eigen::Isometry3d lidar_to_camera)
    : feature_params_(feature_params),
      matcher_params_(matcher_params),
      lidar_to_camera_(std::move(lidar_to_camera)) {}

LidarFrontend::FrameResult LidarFrontend::ProcessScan(const LidarScan& scan) {
  FrameResult result;

  // Extraction relies on the LiDAR's own z-up ring geometry, so the extrinsic
  // is applied afterwards.
  ScanFeatures features = Downsample(ExtractFeatures(scan, feature_params_));
  Transform(features, lidar_to_camera_);

  if (has_previous_scan_) {
    // source=previous, target=current. Seed with the last frame's motion --
    // identity fails once per-frame motion exceeds MatchScans's correspondence
    // search radius, which highway-speed sequences do on every frame.
    ScanMatcherParams params = matcher_params_;
    if (!has_motion_prior_) {
      // No prior means an identity seed, so search wider (as at the first frame).
      params.max_edge_correspondence_dist *= kNoPriorRadiusScale;
      params.max_planar_correspondence_dist *= kNoPriorRadiusScale;
    }
    const auto match = MatchScans(previous_features_, features, last_relative_pose_, params);
    has_motion_prior_ = match.has_value();
    if (match) {
      result.relative_pose = match->pose;
      result.num_edge_correspondences = match->num_edge_correspondences;
      result.num_planar_correspondences = match->num_planar_correspondences;
      result.has_pose = true;
      last_relative_pose_ = match->pose;
    } else {
      // Drop the stale seed so the next no-prior call actually searches around
      // identity, matching this function's widened-radius assumption.
      last_relative_pose_ = Sophus::SE3d();
    }
  }

  previous_features_ = std::move(features);
  has_previous_scan_ = true;
  return result;
}

}  // namespace slam::frontend_lidar
