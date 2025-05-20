// multi_lidar_filter.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <map>
#include <mutex>

namespace lidar_preprocessor {

class MultiLidarFilter : public rclcpp::Node {
public:
  MultiLidarFilter(const rclcpp::NodeOptions & options);

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string & topic);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  bool isInExcludedRegion(float angle_deg);
  void mergeAndPublish();

  std::vector<std::pair<float, float>> excluded_ranges_;
  std::string output_topic_;

  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
  std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;

  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_list_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::mutex buffer_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace lidar_preprocessor

