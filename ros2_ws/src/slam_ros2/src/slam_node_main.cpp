#include <rclcpp/rclcpp.hpp>

#include "slam_ros2/slam_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_ros2::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
