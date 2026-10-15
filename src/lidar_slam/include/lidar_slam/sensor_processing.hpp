#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_slam {
class IcpPointtoLine {
public:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  void setTargetCloud(const Cloud::ConstPtr &target_cloud) {
    target_cloud_ = target_cloud;
    if (target_cloud_ && !target_cloud_->empty()) {
      target_tree_.setInputCloud(target_cloud_);
    }
  }

  Cloud::Ptr findTargetNeighbours(const Eigen::Vector2f &source_point,
                                 float search_radius) const {
    Cloud::Ptr neighbours(new Cloud());
    if (!target_cloud_ || target_cloud_->empty() || !source_point.allFinite() ||
        !std::isfinite(search_radius) || search_radius <= 0.0f) {
      return neighbours;
    }

    Point query_point{};
    query_point.x = source_point.x();
    query_point.y = source_point.y();
    query_point.z = 0.0f;

    std::vector<int> indices;
    std::vector<float> squared_distances;
    const int found = target_tree_.radiusSearch(
        query_point, search_radius, indices, squared_distances);
    if (found <= 0) {
      return neighbours;
    }

    neighbours->points.reserve(static_cast<std::size_t>(found));
    for (int i = 0; i < found; ++i) {
      const int point_index = indices[static_cast<std::size_t>(i)];
      if (point_index >= 0 &&
          static_cast<std::size_t>(point_index) < target_cloud_->points.size()) {
        neighbours->points.push_back(
            target_cloud_->points[static_cast<std::size_t>(point_index)]);
      }
    }
    neighbours->width = static_cast<std::uint32_t>(neighbours->points.size());
    neighbours->height = 1;
    neighbours->is_dense = target_cloud_->is_dense;
    return neighbours;
  }

private:
  Cloud::ConstPtr target_cloud_;
  pcl::KdTreeFLANN<Point> target_tree_;
};

} // namespace lidar_slam
