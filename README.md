# Lidar Preprocessor (ROS 2 HUMBLE)

## Overview

`lidar_preprocessor` is a ROS 2 package designed for autonomous vehicle applications. It performs multi-LiDAR point cloud preprocessing, including angle-based filtering, center-radius filtering, point cloud fusion, and IMU data redirection with frame transformation. It provides a clean and unified output for downstream modules such as perception, localization, and planning.

---

## Features

### ✅ Multi-LiDAR Point Cloud Processing

* Subscribes to multiple LiDAR point cloud topics
* Applies consistent filtering to all sources
* Merges the filtered point clouds into a single unified output

### ✅ Multi-Topic Publishing

* The fused point cloud is simultaneously published to:

  * `/sensing/lidar/concatenated/pointcloud`
  * `/sensing/lidar/top/outlier_filtered/pointcloud`

### ✅ Filtering Mechanisms

* **Angle Filtering**: Configurable excluded angle ranges via the `excluded_angles` parameter, e.g., `[-30, 30]` filters a 60° front sector
* **Center Radius Filtering**: Removes points within a specified radius of the origin, set using `exclude_center_radius`

### ✅ IMU Data Redirection

* Subscribes to a single IMU topic (e.g. `/livox/imu_xxx`)
* Republished to another topic (e.g. `/sensing/imu/tamagawa/imu_raw`)
* IMU message frame ID is rewritten to `tamagawa/imu_link`

---

## Parameters (YAML Example)

```yaml
multi_lidar_filter_node:
  ros__parameters:
    input_lidar_topics:
      - "/livox/lidar_xxx"
      - "/livox/lidar_xxx"
    input_imu_topic: "/livox/imu_xxx"
    output_lidar_topics:
      - "/sensing/lidar/concatenated/pointcloud"
      - "/sensing/lidar/top/outlier_filtered/pointcloud"
    output_imu_topic: "/sensing/imu/tamagawa/imu_raw"
    excluded_angles: [-30.0, 30.0, 150.0, 210.0]
    exclude_center_radius: 2.0
```

---

## Launch

```bash
ros2 launch lidar_preprocessor multi_lidar_filter.launch.py
```

---

## Coordinate Frames

* All point clouds are published in the `base_link` frame
* IMU messages are published in the `tamagawa/imu_link` frame

It is recommended to use `static_transform_publisher` or TF2 broadcasters to ensure proper frame alignment.

---

## License

This package is released under the MIT License.
