cmake_minimum_required(VERSION 3.5)
project(lidar_preprocessor)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 包含头文件路径
include_directories(include)

# 编译 angle_filter_node
add_executable(angle_filter_node
  src/nodes/angle_filter_node.cpp
  src/modules/angle_filter.cpp
)

ament_target_dependencies(angle_filter_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ✅ 编译 multi_lidar_filter_node
add_executable(multi_lidar_filter_node
  src/nodes/multi_lidar_filter_node.cpp
  src/modules/multi_lidar_filter.cpp
)

ament_target_dependencies(multi_lidar_filter_node
  rclcpp
  sensor_msgs
  std_msgs
)

# 安装节点可执行文件
install(TARGETS
  angle_filter_node
  multi_lidar_filter_node  # ✅ 安装新节点
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（可供其他包 include）
install(DIRECTORY include/
  DESTINATION include/
)

# 安装参数 & 启动文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

