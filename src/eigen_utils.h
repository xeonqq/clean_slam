//
// Created by root on 3/14/20.
//

#ifndef CLEAN_SLAM_SRC_EIGEN_UTILS_H_
#define CLEAN_SLAM_SRC_EIGEN_UTILS_H_

#include <Eigen/Dense>

namespace clean_slam {
Eigen::Vector3d
SkewSymmetricMatToTranslation(const Eigen::Matrix3d &skew_symetric_mat);
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_EIGEN_UTILS_H_
