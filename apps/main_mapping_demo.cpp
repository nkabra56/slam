#include <filesystem>
#include <iostream>
#include <optional>
#include <vector>

#include "slam/backend/optimizer.hpp"
#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/mapping/map.hpp"
#include "slam/sensors/kitti_dataset.hpp"

// Runs VIO + LiDAR + backend over a KITTI sequence (as slam_backend_demo
// does), then builds a world-frame point-cloud map from each keyframe's
// triangulated VIO landmarks and LiDAR edge/planar features using the
// FINAL globally-optimized poses (a two-pass approach: per-frame local
// points are collected during the main loop, then transformed and merged
// into the map only after OptimizeGlobally() -- so the map reflects
// loop-closure corrections instead of the raw incremental poses), and
// writes the result to a PLY file.
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: slam_mapping_demo <sequence_dir> <output.ply> [poses_file]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  const std::filesystem::path output_ply{argv[2]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 4) {
    poses_file = std::filesystem::path{argv[3]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    const slam::StereoCalibration calibration = reader.LoadCalibration();

    slam::frontend_vio::VioFrontend vio(calibration);
    slam::frontend_lidar::LidarFrontend lidar;
    slam::backend::SlidingWindowOptimizer optimizer;

    std::vector<std::vector<Eigen::Vector3d>> landmarks_per_frame;
    std::vector<std::vector<Eigen::Vector3d>> lidar_points_per_frame;
    landmarks_per_frame.reserve(reader.NumFrames());
    lidar_points_per_frame.reserve(reader.NumFrames());

    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      slam::StereoFrame stereo_frame;
      stereo_frame.timestamp = reader.TimestampAt(i);
      stereo_frame.left = reader.LoadImage(i, 0).image;
      stereo_frame.right = reader.LoadImage(i, 1).image;
      const auto vio_result = vio.ProcessStereoFrame(stereo_frame);

      const slam::LidarScan scan = reader.LoadLidarScan(i);
      const auto lidar_result = lidar.ProcessScan(scan);

      slam::backend::SlidingWindowOptimizer::EdgeMeasurement vio_edge;
      vio_edge.valid = vio_result.has_pose;
      vio_edge.relative_pose = vio_result.relative_pose;
      vio_edge.num_matches = vio_result.num_inliers;

      slam::backend::SlidingWindowOptimizer::EdgeMeasurement lidar_edge;
      lidar_edge.valid = lidar_result.has_pose;
      lidar_edge.relative_pose = lidar_result.relative_pose;
      lidar_edge.num_matches =
          lidar_result.num_edge_correspondences + lidar_result.num_planar_correspondences;

      optimizer.AddKeyframe(vio_edge, lidar_edge, std::nullopt, lidar.LastFeatures());

      std::vector<Eigen::Vector3d> landmarks;
      landmarks.reserve(vio.LastLandmarks().size());
      for (const auto& [track_id, point] : vio.LastLandmarks()) {
        landmarks.push_back(point);
      }
      landmarks_per_frame.push_back(std::move(landmarks));

      const auto& features = lidar.LastFeatures();
      std::vector<Eigen::Vector3d> lidar_points;
      lidar_points.reserve(features.edge_points.size() + features.planar_points.size());
      lidar_points.insert(lidar_points.end(), features.edge_points.begin(),
                           features.edge_points.end());
      lidar_points.insert(lidar_points.end(), features.planar_points.begin(),
                           features.planar_points.end());
      lidar_points_per_frame.push_back(std::move(lidar_points));

      if (i % 100 == 0) {
        std::cout << "frame " << i << "/" << reader.NumFrames() << "\n";
      }
    }

    std::cout << "Running final global optimization...\n";
    optimizer.OptimizeGlobally();

    slam::mapping::Map map;
    for (std::size_t i = 0; i < optimizer.NumKeyframes(); ++i) {
      const Sophus::SE3d& pose = optimizer.PoseOf(i);

      std::vector<Eigen::Vector3d> world_landmarks;
      world_landmarks.reserve(landmarks_per_frame[i].size());
      for (const auto& p : landmarks_per_frame[i]) world_landmarks.push_back(pose * p);
      map.InsertLandmarks(world_landmarks);

      std::vector<Eigen::Vector3d> world_lidar;
      world_lidar.reserve(lidar_points_per_frame[i].size());
      for (const auto& p : lidar_points_per_frame[i]) world_lidar.push_back(pose * p);
      map.InsertLidarPoints(world_lidar);
    }

    std::cout << "Landmark map points: " << map.LandmarkMap().NumPoints() << "\n";
    std::cout << "LiDAR map points: " << map.LidarMap().NumPoints() << "\n";

    if (!map.ExportPly(output_ply)) {
      std::cerr << "Failed to write " << output_ply << "\n";
      return 1;
    }
    std::cout << "Wrote map to " << output_ply << "\n";
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
