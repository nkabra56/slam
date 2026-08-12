#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "slam/backend/optimizer.hpp"
#include "slam/eval/trajectory_metrics.hpp"
#include "slam/frontend_lidar/lidar_frontend.hpp"
#include "slam/frontend_vio/vio_frontend.hpp"
#include "slam/sensors/kitti_dataset.hpp"

namespace {

void PrintRow(const std::string& name, const slam::eval::AbsoluteTrajectoryError& ate,
              const slam::eval::KittiOdometryError& kitti) {
  std::cout << std::left << std::setw(24) << name << std::right << std::setw(12) << std::fixed
            << std::setprecision(3) << ate.rmse_m << std::setw(14) << std::setprecision(2)
            << kitti.avg_translation_error_percent << std::setw(16) << std::setprecision(4)
            << kitti.avg_rotation_error_deg_per_100m << std::setw(12)
            << kitti.num_segments_evaluated << "\n";
}

}  // namespace

// Runs the VIO-only, LiDAR-only, backend-fused (incremental), and
// backend-fused (globally re-optimized) trajectories over a KITTI sequence
// with ground truth, and prints ATE + the official KITTI odometry
// evaluation protocol (avg translation/rotation error over 100-800m
// segments) for each -- the tooling behind ROADMAP.md's Phase 5 comparison
// table. It does not fabricate numbers for that table itself: this project
// has never been build-verified in the environment it was written in, so
// run this against a real downloaded sequence to get real ones.
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: slam_eval_demo <sequence_dir> <poses_file>\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  const std::filesystem::path poses_file{argv[2]};

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    if (!reader.HasGroundTruth()) {
      std::cerr << "Error: no ground truth loaded; evaluation requires it.\n";
      return 1;
    }
    const slam::StereoCalibration calibration = reader.LoadCalibration();

    std::vector<Sophus::SE3d> ground_truth;
    ground_truth.reserve(reader.NumFrames());
    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      const Eigen::Isometry3d gt_pose = reader.GroundTruthPoseAt(i);
      ground_truth.push_back(
          Sophus::SE3d(Eigen::Quaterniond(gt_pose.rotation()).normalized(), gt_pose.translation()));
    }

    slam::frontend_vio::VioFrontend vio(calibration);
    slam::frontend_lidar::LidarFrontend lidar;
    slam::backend::SlidingWindowOptimizer optimizer;

    std::vector<Sophus::SE3d> vio_trajectory{Sophus::SE3d()};
    std::vector<Sophus::SE3d> lidar_trajectory{Sophus::SE3d()};
    std::vector<Sophus::SE3d> fused_incremental_trajectory;
    fused_incremental_trajectory.reserve(reader.NumFrames());

    for (std::size_t i = 0; i < reader.NumFrames(); ++i) {
      slam::StereoFrame stereo_frame;
      stereo_frame.timestamp = reader.TimestampAt(i);
      stereo_frame.left = reader.LoadImage(i, 0).image;
      stereo_frame.right = reader.LoadImage(i, 1).image;
      const auto vio_result = vio.ProcessStereoFrame(stereo_frame);
      if (i > 0) {
        vio_trajectory.push_back(vio_result.has_pose
                                      ? vio_trajectory.back() * vio_result.relative_pose.inverse()
                                      : vio_trajectory.back());
      }

      const slam::LidarScan scan = reader.LoadLidarScan(i);
      const auto lidar_result = lidar.ProcessScan(scan);
      if (i > 0) {
        lidar_trajectory.push_back(
            lidar_result.has_pose ? lidar_trajectory.back() * lidar_result.relative_pose.inverse()
                                   : lidar_trajectory.back());
      }

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
      fused_incremental_trajectory.push_back(optimizer.PoseOf(node_id));

      if (i % 200 == 0) {
        std::cout << "frame " << i << "/" << reader.NumFrames() << "\n";
      }
    }

    std::cout << "Running final global optimization...\n";
    optimizer.OptimizeGlobally();
    std::vector<Sophus::SE3d> fused_global_trajectory;
    fused_global_trajectory.reserve(optimizer.NumKeyframes());
    for (std::size_t i = 0; i < optimizer.NumKeyframes(); ++i) {
      fused_global_trajectory.push_back(optimizer.PoseOf(i));
    }

    std::cout << "\n"
              << std::left << std::setw(24) << "Method" << std::right << std::setw(12)
              << "ATE (m)" << std::setw(14) << "Trans (%)" << std::setw(16) << "Rot (deg/100m)"
              << std::setw(12) << "Segments"
              << "\n";
    std::cout << std::string(78, '-') << "\n";

    PrintRow("VIO-only", slam::eval::ComputeAte(vio_trajectory, ground_truth),
             slam::eval::ComputeKittiOdometryError(vio_trajectory, ground_truth));
    PrintRow("LiDAR-only", slam::eval::ComputeAte(lidar_trajectory, ground_truth),
             slam::eval::ComputeKittiOdometryError(lidar_trajectory, ground_truth));
    PrintRow("Fused (incremental)",
             slam::eval::ComputeAte(fused_incremental_trajectory, ground_truth),
             slam::eval::ComputeKittiOdometryError(fused_incremental_trajectory, ground_truth));
    PrintRow("Fused (global opt.)", slam::eval::ComputeAte(fused_global_trajectory, ground_truth),
             slam::eval::ComputeKittiOdometryError(fused_global_trajectory, ground_truth));
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
