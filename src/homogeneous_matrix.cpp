//
// Created by root on 3/10/20.
//
#include "homogeneous_matrix.h"
#include <opencv2/core/eigen.hpp>

namespace clean_slam {

HomogeneousMatrix CreateHomogeneousMatrix(const cv::Mat &r, const cv::Mat &t) {
  g2o::Matrix3d rotation;
  g2o::Vector3d translation;
  cv::cv2eigen(r, rotation);
  cv::cv2eigen(t, translation);
  return HomogeneousMatrix(rotation, translation);
}

} // namespace clean_slam
