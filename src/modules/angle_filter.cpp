#include "lidar_preprocessor/angle_filter.hpp"

namespace lidar_preprocessor {

AngleFilter::AngleFilter(const rclcpp::NodeOptions & options)
: Node("angle_filter_node", options) {
  // Declare and get parameters
  std::vector<double> ranges_flat;
  this->declare_parameter("excluded_angles", std::vector<double>{});
  this->get_parameter("excluded_angles", ranges_flat);

  if (ranges_flat.size() % 2 != 0) {
    RCLCPP_WARN(this->get_logger(), "excluded_angles size should be even (pairs of min/max)");
  }

  for (size_t i = 0; i + 1 < ranges_flat.size(); i += 2) {
    excluded_ranges_.emplace_back(ranges_flat[i], ranges_flat[i + 1]);
  }

  std::string input_topic = this->declare_parameter("input_topic", "/input_cloud");
  std::string output_topic = this->declare_parameter("output_topic", "/filtered_cloud");

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&AngleFilter::pointcloudCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

  RCLCPP_INFO(this->get_logger(), "AngleFilter node started with %zu excluded angle ranges", excluded_ranges_.size());
}

bool AngleFilter::isInExcludedRegion(float angle_deg) {
  for (const auto & range : excluded_ranges_) {
    if (angle_deg >= range.first && angle_deg <= range.second) {
      return true;
    }
  }
  return false;
}

void AngleFilter::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  sensor_msgs::msg::PointCloud2 output;
  output.header = msg->header;
  output.fields = msg->fields;
  output.point_step = msg->point_step;
  output.is_bigendian = msg->is_bigendian;
  output.is_dense = true;
  output.height = 1;

  const size_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;

  // 获取 x 和 y 的偏移
  int offset_x = -1, offset_y = -1;
  for (const auto & field : msg->fields) {
    if (field.name == "x") offset_x = field.offset;
    if (field.name == "y") offset_y = field.offset;
  }

  if (offset_x < 0 || offset_y < 0) {
    RCLCPP_ERROR(this->get_logger(), "PointCloud2 does not have x/y fields!");
    return;
  }

  std::vector<uint8_t> filtered_bytes;

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* point_ptr = &msg->data[i * point_step];

    float x, y;
    memcpy(&x, point_ptr + offset_x, sizeof(float));
    memcpy(&y, point_ptr + offset_y, sizeof(float));

    float angle = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
    if (isInExcludedRegion(angle)) {
      continue;
    }

    // 拷贝原始整点
    filtered_bytes.insert(filtered_bytes.end(), point_ptr, point_ptr + point_step);
  }

  output.data = std::move(filtered_bytes);
  output.width = output.data.size() / output.point_step;
  output.row_step = output.point_step * output.width;

  pub_->publish(output);
}

}  // namespace lidar_preprocessor
