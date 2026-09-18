#include "slam/mapping/map.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace slam::mapping {
namespace {

TEST(Map, InsertsIntoSeparateLandmarkAndLidarMaps) {
  Map map;
  map.InsertLandmarks({{1.0, 0.0, 0.0}});
  map.InsertLidarPoints({{2.0, 0.0, 0.0}, {2.6, 0.0, 0.0}});

  EXPECT_EQ(map.LandmarkMap().NumPoints(), 1u);
  EXPECT_GE(map.LidarMap().NumPoints(), 1u);
}

TEST(Map, ExportPlyWritesValidHeaderAndVertexCount) {
  Map map;
  map.InsertLandmarks({{1.0, 2.0, 3.0}});
  map.InsertLidarPoints({{40.0, 50.0, 60.0}});

  const auto path = std::filesystem::temp_directory_path() / "slam_map_test.ply";
  ASSERT_TRUE(map.ExportPly(path));

  std::ifstream file(path);
  ASSERT_TRUE(file.is_open());
  std::string line;
  std::getline(file, line);
  EXPECT_EQ(line, "ply");
  std::getline(file, line);
  EXPECT_EQ(line, "format ascii 1.0");
  std::getline(file, line);
  EXPECT_EQ(line, "element vertex 2");

  file.close();
  std::filesystem::remove(path);
}

TEST(Map, ExportPlyFailsGracefullyForUnwritablePath) {
  Map map;
  EXPECT_FALSE(map.ExportPly(
      std::filesystem::temp_directory_path() / "slam_nonexistent_dir_xyz" / "out.ply"));
}

}  // namespace
}  // namespace slam::mapping
