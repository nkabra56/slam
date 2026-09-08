#include <filesystem>
#include <iostream>
#include <optional>

#include "slam/backend/tightly_coupled_optimizer.hpp"
#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/sensors/kitti_dataset.hpp"
#include "slam/sensors/kitti_raw_imu.hpp"

// Like slam_backend_demo, but backed by TightlyCoupledOptimizer. Requires a
// raw KITTI dataset root for real IMU data.
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: slam_tightly_coupled_demo <sequence_dir> <raw_kitti_root> [poses_file]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  const std::filesystem::path raw_kitti_root{argv[2]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 4) {
    poses_file = std::filesystem::path{argv[3]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    const slam::StereoCalibration calibration = reader.LoadCalibration();

    const std::string sequence_name = sequence_dir.filename().string();
    const auto mapping = slam::sensors::LookupRawDriveMapping(sequence_name);
    if (!mapping.has_value()) {
      std::cerr << "Error: no raw-data mapping known for sequence '" << sequence_name
                << "'; this demo requires real IMU data.\n";
      return 1;
    }
    const std::filesystem::path oxts_dir =
        raw_kitti_root / mapping->date / (mapping->drive + "_sync") / "oxts";
    slam::sensors::KittiOxtsReader oxts_reader(oxts_dir, mapping->start_frame);
    std::cout << "Loaded IMU from " << oxts_dir << " (start_frame=" << mapping->start_frame << ")\n";

    slam::frontend_vio::VioFrontend vio(calibration);
    slam::frontend_lidar::LidarFrontend lidar;
    slam::backend::TightlyCoupledOptimizer optimizer;

    bool announced_init = false;

    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      if (i < oxts_reader.NumMeasurements()) {
        optimizer.AddImuMeasurement(oxts_reader.MeasurementAt(i));
      }

      slam::StereoFrame stereo_frame;
      stereo_frame.timestamp = reader.TimestampAt(i);
      stereo_frame.left = reader.LoadImage(i, 0).image;
      stereo_frame.right = reader.LoadImage(i, 1).image;
      const auto vio_result = vio.ProcessStereoFrame(stereo_frame);

      const slam::LidarScan scan = reader.LoadLidarScan(i);
      const auto lidar_result = lidar.ProcessScan(scan);

      slam::backend::TightlyCoupledOptimizer::EdgeMeasurement vio_edge;
      vio_edge.valid = vio_result.has_pose;
      vio_edge.relative_pose = vio_result.relative_pose;
      vio_edge.num_matches = vio_result.num_inliers;

      slam::backend::TightlyCoupledOptimizer::EdgeMeasurement lidar_edge;
      lidar_edge.valid = lidar_result.has_pose;
      lidar_edge.relative_pose = lidar_result.relative_pose;
      lidar_edge.num_matches =
          lidar_result.num_edge_correspondences + lidar_result.num_planar_correspondences;

      const auto node_id = optimizer.AddKeyframe(vio_edge, lidar_edge, lidar.LastFeatures());

      if (optimizer.IsInitialized() && !announced_init) {
        announced_init = true;
        const Eigen::Vector3d g = optimizer.Gravity();
        std::cout << "-- tightly-coupled initialization succeeded at frame " << i
                  << " (gravity=[" << g.x() << ", " << g.y() << ", " << g.z() << "]) --\n";
      }

      const auto& state = optimizer.StateOf(node_id);
      const Eigen::Vector3d t = state.pose.translation();
      std::cout << "frame " << i << ": pos=[" << t.x() << ", " << t.y() << ", " << t.z() << "]"
                << " vel=[" << state.velocity.x() << ", " << state.velocity.y() << ", "
                << state.velocity.z() << "]";
      if (reader.HasGroundTruth()) {
        const Eigen::Vector3d gt = reader.GroundTruthPoseAt(i).translation();
        std::cout << " gt=[" << gt.x() << ", " << gt.y() << ", " << gt.z() << "]";
      }
      std::cout << "\n";
    }

    std::cout << "Loop closures detected: " << optimizer.NumLoopClosures() << "\n";
    std::cout << "Running final global optimization...\n";
    const int iterations = optimizer.OptimizeGlobally();
    std::cout << "Converged in " << iterations << " iterations.\n";

    for (std::size_t i = 0; i < optimizer.NumKeyframes(); ++i) {
      const auto& state = optimizer.StateOf(i);
      const Eigen::Vector3d t = state.pose.translation();
      std::cout << "final frame " << i << ": pos=[" << t.x() << ", " << t.y() << ", " << t.z()
                << "] vel=[" << state.velocity.x() << ", " << state.velocity.y() << ", "
                << state.velocity.z() << "] bias_gyro=[" << state.bias_gyro.x() << ", "
                << state.bias_gyro.y() << ", " << state.bias_gyro.z() << "]\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
