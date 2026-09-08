#include <filesystem>
#include <iostream>
#include <optional>

#include "slam/backend/optimizer.hpp"
#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/sensors/kitti_dataset.hpp"
#include "slam/sensors/kitti_raw_imu.hpp"

// Runs VIO + LiDAR through the sliding-window backend and prints the fused
// trajectory. With a raw KITTI root, also feeds real IMU.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: slam_backend_demo <sequence_dir> [poses_file] [raw_kitti_root]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 3) {
    poses_file = std::filesystem::path{argv[2]};
  }
  std::optional<std::filesystem::path> raw_kitti_root;
  if (argc >= 4) {
    raw_kitti_root = std::filesystem::path{argv[3]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    const slam::StereoCalibration calibration = reader.LoadCalibration();

    std::optional<slam::sensors::KittiOxtsReader> oxts_reader;
    if (raw_kitti_root.has_value()) {
      const std::string sequence_name = sequence_dir.filename().string();
      if (const auto mapping = slam::sensors::LookupRawDriveMapping(sequence_name)) {
        const std::filesystem::path oxts_dir =
            *raw_kitti_root / mapping->date / (mapping->drive + "_sync") / "oxts";
        oxts_reader.emplace(oxts_dir, mapping->start_frame);
        std::cout << "Loaded IMU from " << oxts_dir << " (start_frame=" << mapping->start_frame
                  << ")\n";
      } else {
        std::cerr << "Warning: no raw-data mapping known for sequence '" << sequence_name
                  << "'; continuing without IMU.\n";
      }
    }

    slam::frontend_vio::VioFrontend vio(calibration);
    slam::frontend_lidar::LidarFrontend lidar;
    slam::backend::SlidingWindowOptimizer optimizer;

    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      if (oxts_reader.has_value() && i < oxts_reader->NumMeasurements()) {
        vio.ProcessImu(oxts_reader->MeasurementAt(i));
      }

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

      std::optional<slam::frontend_vio::ImuPreintegrationResult> imu_edge;
      if (oxts_reader.has_value()) {
        imu_edge = vio_result.imu_delta;
      }

      const auto node_id = optimizer.AddKeyframe(vio_edge, lidar_edge, imu_edge, lidar.LastFeatures());

      const Eigen::Vector3d t = optimizer.PoseOf(node_id).translation();
      std::cout << "frame " << i << ": pos=[" << t.x() << ", " << t.y() << ", " << t.z() << "]";
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
      const Eigen::Vector3d t = optimizer.PoseOf(i).translation();
      std::cout << "final frame " << i << ": pos=[" << t.x() << ", " << t.y() << ", " << t.z()
                << "]\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
