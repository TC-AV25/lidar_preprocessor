// multi_lidar_filter.cpp

#include "lidar_preprocessor/multi_lidar_filter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <mutex>

namespace lidar_preprocessor {

double center_exclusion_radius_ = 0.0;

MultiLidarFilter::MultiLidarFilter(const rclcpp::NodeOptions & options)
: Node("multi_lidar_filter_node", options) {
  this->declare_parameter("input_lidar_topics", std::vector<std::string>{});
  this->declare_parameter("output_lidar_topics", std::vector<std::string>{});
  this->declare_parameter("excluded_angles", std::vector<double>{});
  this->declare_parameter("exclude_center_radius", 0.0);
  this->declare_parameter("input_imu_topic", std::string{});
  this->declare_parameter("output_imu_topic", std::string{});

  std::vector<std::string> input_lidar_topics;
  std::vector<std::string> output_lidar_topics;
  std::string input_imu_topic;
  std::string output_imu_topic;
  std::vector<double> ranges_flat;

  this->get_parameter("input_lidar_topics", input_lidar_topics);
  this->get_parameter("output_lidar_topics", output_lidar_topics);
  this->get_parameter("excluded_angles", ranges_flat);
  this->get_parameter("exclude_center_radius", center_exclusion_radius_);
  this->get_parameter("input_imu_topic", input_imu_topic);
  this->get_parameter("output_imu_topic", output_imu_topic);

  for (size_t i = 0; i + 1 < ranges_flat.size(); i += 2) {
    excluded_ranges_.emplace_back(ranges_flat[i], ranges_flat[i + 1]);
  }

  for (const auto & topic : input_lidar_topics) {
    auto callback = [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      this->pointcloudCallback(msg, topic);
    };
    subscribers_[topic] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS(), callback);
  }

  for (const auto & out_topic : output_lidar_topics) {
    pub_list_.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10));
  }

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_imu_topic, 10);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    input_imu_topic, 10, std::bind(&MultiLidarFilter::imuCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MultiLidarFilter::mergeAndPublish, this));

  RCLCPP_INFO(this->get_logger(), "MultiLidarFilter started with %zu input topics, %zu excluded angle ranges, center exclusion radius %.2f.",
              input_lidar_topics.size(), excluded_ranges_.size(), center_exclusion_radius_);
}

bool MultiLidarFilter::isInExcludedRegion(float angle_deg) {
  for (const auto & range : excluded_ranges_) {
    if (angle_deg >= range.first && angle_deg <= range.second) {
      return true;
    }
  }
  return false;
}

void MultiLidarFilter::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string & topic) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  latest_clouds_[topic] = msg;
}

void MultiLidarFilter::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  auto imu = *msg;
  imu.header.frame_id = "tamagawa/imu_link";
  imu_pub_->publish(imu);
}

void MultiLidarFilter::mergeAndPublish() {
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  sensor_msgs::msg::PointCloud2 merged;
  merged.header.frame_id = "base_link";
  merged.height = 1;
  merged.is_bigendian = false;
  merged.is_dense = true;

  std::vector<uint8_t> merged_data;
  bool first = true;

  for (const auto & [topic, msg] : latest_clouds_) {
    if (!msg || msg->data.empty()) continue;

    if (first) {
      merged.fields = msg->fields;
      merged.point_step = msg->point_step;
      first = false;
    }

    if (msg->point_step == 0) {
      RCLCPP_WARN(this->get_logger(), "点云 %s 的 point_step 为 0，跳过", topic.c_str());
      continue;
    }

    int offset_x = -1, offset_y = -1;
    for (const auto & field : msg->fields) {
      if (field.name == "x") offset_x = field.offset;
      if (field.name == "y") offset_y = field.offset;
    }
    if (offset_x < 0 || offset_y < 0) {
      RCLCPP_WARN(this->get_logger(), "点云 %s 缺少 x/y 字段，跳过", topic.c_str());
      continue;
    }

    size_t point_count = msg->width * msg->height;
    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t* point_ptr = &msg->data[i * msg->point_step];
      float x, y;
      memcpy(&x, point_ptr + offset_x, sizeof(float));
      memcpy(&y, point_ptr + offset_y, sizeof(float));

      float angle = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
      float distance = std::sqrt(x * x + y * y);

      if (isInExcludedRegion(angle) || distance < center_exclusion_radius_) {
        continue;
      }

      merged_data.insert(merged_data.end(), point_ptr, point_ptr + msg->point_step);
    }
  }

  if (merged.point_step == 0 || merged_data.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "没有任何有效点可合并，跳过本轮发布");
    return;
  }

  merged.data = std::move(merged_data);
  merged.width = merged.data.size() / merged.point_step;
  merged.row_step = merged.point_step * merged.width;
  merged.header.stamp = this->now();

  for (auto & pub : pub_list_) {
    pub->publish(merged);
  }
}

}  // namespace lidar_preprocessor

