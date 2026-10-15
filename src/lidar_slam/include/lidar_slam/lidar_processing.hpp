#pragma once

#include "sensor_msgs/LaserScan.h"
#incldue "pcl/point_cloud.h"

namespace lidar_slam {
class LidarProcessing {
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  convertScanToPointCloud(sensor_msgs::LaserScan::ConstPtr scan_msg,
                          double min_range, double max_range,
                          double intensity_threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
      if (scan_msg->ranges[i] < min_range || scan_msg->ranges[i] > max_range ||
          scan_msg->intensities[i] < intensity_threshold) {
        continue;
      }
      float angle = scan_msg->angl_min + i * scan_msg->angle_increment;
      pcl::PointXYZ point{scan_msg->ranges[i] * cos(angle),
                          scan_msg->ranges[i] * sin(angle), 0.0f};
      cloud->points.push_back(point);
    }
    return cloud;
  };

  
  



  
};
} // namespace lidar_slam