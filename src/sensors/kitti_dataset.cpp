#include "slam/sensors/kitti_dataset.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace slam::sensors {

namespace {

std::string ZeroPadded(std::size_t index, int width = 6) {
  std::ostringstream oss;
  oss << std::setw(width) << std::setfill('0') << index;
  return oss.str();
}

std::vector<TimestampSec> ReadTimestamps(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("KittiSequenceReader: could not open times.txt at " +
                              path.string());
  }
  std::vector<TimestampSec> timestamps;
  double t;
  while (file >> t) {
    timestamps.push_back(t);
  }
  return timestamps;
}

std::vector<Eigen::Isometry3d> ReadGroundTruthPoses(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("KittiSequenceReader: could not open poses file at " +
                              path.string());
  }
  std::vector<Eigen::Isometry3d> poses;
  std::string line;
  while (std::getline(file, line)) {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }
    std::istringstream iss(line);
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        iss >> mat(row, col);
      }
    }
    if (!iss) {
      throw std::runtime_error("KittiSequenceReader: malformed pose line in " + path.string());
    }
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.matrix() = mat;
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace

KittiSequenceReader::KittiSequenceReader(const std::filesystem::path& sequence_dir,
                                          std::optional<std::filesystem::path> poses_file)
    : sequence_dir_(sequence_dir) {
  if (!std::filesystem::exists(sequence_dir_)) {
    throw std::runtime_error("KittiSequenceReader: sequence directory does not exist: " +
                              sequence_dir_.string());
  }

  timestamps_ = ReadTimestamps(sequence_dir_ / "times.txt");

  if (poses_file.has_value()) {
    ground_truth_poses_ = ReadGroundTruthPoses(*poses_file);
    if (ground_truth_poses_.size() != timestamps_.size()) {
      throw std::runtime_error(
          "KittiSequenceReader: ground-truth pose count does not match frame count in " +
          sequence_dir_.string());
    }
  }
}

std::size_t KittiSequenceReader::NumFrames() const { return timestamps_.size(); }

TimestampSec KittiSequenceReader::TimestampAt(std::size_t index) const {
  return timestamps_.at(index);
}

ImageFrame KittiSequenceReader::LoadImage(std::size_t index, int camera) const {
  const std::filesystem::path path =
      sequence_dir_ / ("image_" + std::to_string(camera)) / (ZeroPadded(index) + ".png");
  cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    throw std::runtime_error("KittiSequenceReader: failed to load image at " + path.string());
  }
  return ImageFrame{TimestampAt(index), std::move(image)};
}

LidarScan KittiSequenceReader::LoadLidarScan(std::size_t index) const {
  const std::filesystem::path path = sequence_dir_ / "velodyne" / (ZeroPadded(index) + ".bin");
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("KittiSequenceReader: failed to open velodyne scan at " +
                              path.string());
  }

  file.seekg(0, std::ios::end);
  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  constexpr std::streamsize kPointBytes = 4 * sizeof(float);
  if (size % kPointBytes != 0) {
    throw std::runtime_error("KittiSequenceReader: velodyne scan has unexpected size: " +
                              path.string());
  }

  LidarScan scan;
  scan.timestamp = TimestampAt(index);
  scan.points.resize(static_cast<std::size_t>(size / kPointBytes));
  file.read(reinterpret_cast<char*>(scan.points.data()), size);

  return scan;
}

bool KittiSequenceReader::HasGroundTruth() const { return !ground_truth_poses_.empty(); }

Eigen::Isometry3d KittiSequenceReader::GroundTruthPoseAt(std::size_t index) const {
  if (!HasGroundTruth()) {
    throw std::runtime_error("KittiSequenceReader: sequence has no ground-truth poses loaded");
  }
  return ground_truth_poses_.at(index);
}

}  // namespace slam::sensors
