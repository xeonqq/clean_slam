//
// Created by root on 1/20/20.
//

#ifndef CLEAN_SLAM_SRC_HOMOGENEOUS_MATRIX_H_
#define CLEAN_SLAM_SRC_HOMOGENEOUS_MATRIX_H_

#include "third_party/g2o/g2o/types/se3quat.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {
using HomogeneousMatrix = g2o::SE3Quat;

HomogeneousMatrix CreateHomogeneousMatrix(const cv::Mat &r, const cv::Mat &t);

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_HOMOGENEOUS_MATRIX_H_
