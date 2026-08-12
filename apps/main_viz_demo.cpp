#include <filesystem>
#include <iostream>
#include <optional>
#include <vector>

#include "slam/backend/optimizer.hpp"
#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/mapping/map.hpp"
#include "slam/sensors/kitti_dataset.hpp"
#include "slam/viz/pangolin_viewer.hpp"

// Live Pangolin viewer over VIO + LiDAR + backend fusion. Unlike
// slam_mapping_demo (which waits for OptimizeGlobally() so its exported
// map reflects loop-closure corrections), this shows the trajectory and
// map growing incrementally, frame by frame -- what a live operator would
// actually see.
//
// This is the one demo in the project that can't be verified without a
// display; see PangolinViewer's class doc comment.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: slam_viz_demo <sequence_dir> [poses_file]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 3) {
    poses_file = std::filesystem::path{argv[2]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    const slam::StereoCalibration calibration = reader.LoadCalibration();

    slam::frontend_vio::VioFrontend vio(calibration);
    slam::frontend_lidar::LidarFrontend lidar;
    slam::backend::SlidingWindowOptimizer optimizer;
    slam::viz::PangolinViewer viewer("slam_viz_demo");

    slam::mapping::Map map;  // grows incrementally for a live view
    std::vector<Sophus::SE3d> trajectory;

    for (std::size_t i = 0; i < reader.NumFrames() && !viewer.ShouldClose(); ++i) {
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

      const auto node_id =
          optimizer.AddKeyframe(vio_edge, lidar_edge, std::nullopt, lidar.LastFeatures());
      const Sophus::SE3d& pose = optimizer.PoseOf(node_id);
      trajectory.push_back(pose);

      std::vector<Eigen::Vector3d> world_landmarks;
      world_landmarks.reserve(vio.LastLandmarks().size());
      for (const auto& [track_id, p] : vio.LastLandmarks()) world_landmarks.push_back(pose * p);
      map.InsertLandmarks(world_landmarks);

      const auto& features = lidar.LastFeatures();
      std::vector<Eigen::Vector3d> world_lidar;
      world_lidar.reserve(features.edge_points.size() + features.planar_points.size());
      for (const auto& p : features.edge_points) world_lidar.push_back(pose * p);
      for (const auto& p : features.planar_points) world_lidar.push_back(pose * p);
      map.InsertLidarPoints(world_lidar);

      viewer.Update(trajectory, map.LandmarkMap().Points(), map.LidarMap().Points());
    }

    std::cout << "Done. Close the viewer window to exit.\n";
    while (!viewer.ShouldClose()) {
      viewer.Update(trajectory, map.LandmarkMap().Points(), map.LidarMap().Points());
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
