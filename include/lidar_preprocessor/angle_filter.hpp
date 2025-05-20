// angle_filter.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>
#include <string>
#include <utility>
#include <cmath>

namespace lidar_preprocessor {

class AngleFilter : public rclcpp::Node {
public:
  AngleFilter(const rclcpp::NodeOptions & options);

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool isInExcludedRegion(float angle_deg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::vector<std::pair<float, float>> excluded_ranges_;  // in degrees
};

} // namespace lidar_preprocessor

