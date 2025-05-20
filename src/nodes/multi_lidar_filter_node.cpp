
#include "lidar_preprocessor/multi_lidar_filter.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_preprocessor::MultiLidarFilter>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
