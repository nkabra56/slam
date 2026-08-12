#include <filesystem>
#include <iostream>
#include <optional>

#include "slam/sensors/kitti_dataset.hpp"

// Smoke-test / usage example for KittiSequenceReader. Not a SLAM pipeline
// yet -- that arrives as each phase in ROADMAP.md wires into slam_core.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: slam_kitti_demo <sequence_dir> [poses_file]\n";
    return 1;
  }

  const std::filesystem::path sequence_dir{argv[1]};
  std::optional<std::filesystem::path> poses_file;
  if (argc >= 3) {
    poses_file = std::filesystem::path{argv[2]};
  }

  try {
    const slam::sensors::KittiSequenceReader reader(sequence_dir, poses_file);
    std::cout << "Loaded sequence: " << sequence_dir << "\n";
    std::cout << "Frames: " << reader.NumFrames() << "\n";
    if (reader.NumFrames() > 0) {
      std::cout << "First timestamp: " << reader.TimestampAt(0) << " s\n";
    }
    std::cout << std::boolalpha << "Ground truth available: " << reader.HasGroundTruth() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
