#include "slam/sensors/kitti_raw_imu.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace slam::sensors {

namespace {

struct SequenceMappingEntry {
  const char* sequence;
  const char* date;
  const char* drive;
  std::size_t start_frame;
};

// Verified against the official odometry devkit's sequence-to-raw mapping
// table (cross-checked two independent devkit mirrors); see
// KittiRawDriveMapping's doc comment.
constexpr std::array<SequenceMappingEntry, 11> kSequenceMapping{{
    {"00", "2011_10_03", "2011_10_03_drive_0027", 0},
    {"01", "2011_10_03", "2011_10_03_drive_0042", 0},
    {"02", "2011_10_03", "2011_10_03_drive_0034", 0},
    {"03", "2011_09_26", "2011_09_26_drive_0067", 0},
    {"04", "2011_09_30", "2011_09_30_drive_0016", 0},
    {"05", "2011_09_30", "2011_09_30_drive_0018", 0},
    {"06", "2011_09_30", "2011_09_30_drive_0020", 0},
    {"07", "2011_09_30", "2011_09_30_drive_0027", 0},
    {"08", "2011_09_30", "2011_09_30_drive_0028", 1100},
    {"09", "2011_09_30", "2011_09_30_drive_0033", 0},
    {"10", "2011_09_30", "2011_09_30_drive_0034", 0},
}};

std::string ZeroPadded(std::size_t index, int width = 6) {
  std::ostringstream oss;
  oss << std::setw(width) << std::setfill('0') << index;
  return oss.str();
}

// Days since 1970-01-01 for a proleptic Gregorian y-m-d. The well-known
// branchless algorithm from Howard Hinnant's "chrono-Compatible Low-Level
// Date Algorithms" -- used instead of timegm/_mkgmtime to avoid relying on
// a libc extension that isn't uniformly available across the Linux/Windows
// CI matrix without extra feature-test-macro juggling. Verified against
// its defining property: DaysFromCivil(1970, 1, 1) == 0.
long long DaysFromCivil(long long y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const long long yoe = y - era * 400;                                   // [0, 399]
  const long long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;  // [0, 365]
  const long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;           // [0, 146096]
  return era * 146097 + doe - 719468;
}

TimestampSec ParseOxtsTimestamp(const std::string& line) {
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  double seconds = 0.0;
  if (std::sscanf(line.c_str(), "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute,
                   &seconds) != 6) {
    throw std::runtime_error("KittiOxtsReader: malformed timestamp line: " + line);
  }

  const long long days = DaysFromCivil(year, month, day);
  return static_cast<double>(days) * 86400.0 + hour * 3600.0 + minute * 60.0 + seconds;
}

}  // namespace

std::optional<KittiRawDriveMapping> LookupRawDriveMapping(const std::string& sequence) {
  for (const auto& entry : kSequenceMapping) {
    if (sequence == entry.sequence) {
      return KittiRawDriveMapping{entry.date, entry.drive, entry.start_frame};
    }
  }
  return std::nullopt;
}

KittiOxtsReader::KittiOxtsReader(const std::filesystem::path& oxts_dir, std::size_t start_frame)
    : oxts_dir_(oxts_dir), start_frame_(start_frame) {
  const std::filesystem::path timestamps_path = oxts_dir_ / "timestamps.txt";
  std::ifstream file(timestamps_path);
  if (!file.is_open()) {
    throw std::runtime_error("KittiOxtsReader: could not open timestamps.txt at " +
                              timestamps_path.string());
  }

  std::vector<TimestampSec> raw_timestamps;
  std::string line;
  while (std::getline(file, line)) {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
    raw_timestamps.push_back(ParseOxtsTimestamp(line));
  }
  if (start_frame_ >= raw_timestamps.size()) {
    throw std::runtime_error("KittiOxtsReader: start_frame is beyond the end of " +
                              timestamps_path.string());
  }

  const TimestampSec t0 = raw_timestamps[start_frame_];
  timestamps_.reserve(raw_timestamps.size() - start_frame_);
  for (std::size_t i = start_frame_; i < raw_timestamps.size(); ++i) {
    timestamps_.push_back(raw_timestamps[i] - t0);
  }
}

std::size_t KittiOxtsReader::NumMeasurements() const { return timestamps_.size(); }

ImuMeasurement KittiOxtsReader::MeasurementAt(std::size_t index) const {
  const std::size_t raw_index = start_frame_ + index;
  const std::filesystem::path path = oxts_dir_ / "data" / (ZeroPadded(raw_index) + ".txt");
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("KittiOxtsReader: could not open oxts data file at " + path.string());
  }

  std::vector<double> fields;
  fields.reserve(30);
  double value;
  while (file >> value) fields.push_back(value);
  if (fields.size() < 23) {
    throw std::runtime_error("KittiOxtsReader: oxts data file has too few fields: " + path.string());
  }

  // Field order (0-indexed): lat,lon,alt,roll,pitch,yaw,vn,ve,vf,vl,vu,
  // ax,ay,az,af,al,au,wx,wy,wz,wf,wl,wu,... -- see the class doc comment
  // for why ax,ay,az/wx,wy,wz (indices 11-13, 17-19) are used and
  // af,al,au/wf,wl,wu are not.
  ImuMeasurement measurement;
  measurement.timestamp = timestamps_.at(index);
  measurement.linear_acceleration = Eigen::Vector3d(fields[11], fields[12], fields[13]);
  measurement.angular_velocity = Eigen::Vector3d(fields[17], fields[18], fields[19]);
  return measurement;
}

}  // namespace slam::sensors
