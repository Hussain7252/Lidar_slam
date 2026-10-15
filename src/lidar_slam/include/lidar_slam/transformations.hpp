#pragma once

#include "iostream"
#include <Eigen/Dense>

// The class contains helper functions related to transformations for the 2D
// lidar slam
namespace lidar_slam {
class custom_transformations {

  std::pair<Eigen::Vector2f, float>
  extractTranslationAndRotation(const Eigen::Matrix3f &transformation_matrix) {
    Eigen::Vector2f translation = transformation_matrix.block<2, 1>(0, 2);
    float rotation_angle =
        atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));
    return std::make_pair(translation, rotation_angle);
  }
  Eigen::Matrix3f
  createHomogeneousTransformationMatrix(const Eigen::Vector2f &translation,
                                        float rotation_angle) {
    Eigen::Matrix2f rotation_matrix = createRotationMatrix(rotation_angle);
    Eigen::Matrix3f homogeneous_transformation_matrix;
    homogeneous_transformation_matrix << rotation_matrix(0, 0),
        rotation_matrix(0, 1), translation(0), rotation_matrix(1, 0),
        rotation_matrix(1, 1), translation(1), 0, 0, 1;
    return homogeneous_transformation_matrix;
  }

  Eigen::Matrix2f createRotationMatrix(float rotation_angle) {
    Eigen::Matrix2f rotation_matrix;
    rotation_matrix << cos(rotation_angle), -sin(rotation_angle),
        sin(rotation_angle), cos(rotation_angle);
    return rotation_matrix;
  }

  Eigen::Matrix3f
  createRelativeTranformationMatrix(const Eigen::Vector3f &target,
                                    const Eigen::Vector3f &source) {
    Eigen::Matrix3f tranform1 =
        createHomogeneousTransformationMatrix(source.head<2>(), source(2));
    Eigen::Matrix3f tranform2 =
        createHomogeneousTransformationMatrix(target.head<2>(), target(2));
    Eigen::Matrix3f relative_transform = tranform2.inverse() * tranform1;
    return relative_transform;
  }

  Eigen::Vector2f transformPoint(const Eigen::Vector2f &point,
                                 const Eigen::Matrix3f &transformation_matrix) {
    Eigen::Vector3f point_homogeneous(point(0), point(1), 1.0f);
    Eigen::Vector3f transformed_point_homogeneous =
        transformation_matrix * point_homogeneous;
    return transformed_point_homogeneous.head<2>();
  }
};
} // namespace lidar_slam