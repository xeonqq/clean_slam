//
// Created by root on 3/10/20.
//
#include "homogeneous_matrix.h"
#include "cv_utils.h"

namespace clean_slam {

HomogeneousMatrix CreateHomogeneousMatrix(const cv::Mat &r, const cv::Mat &t) {
  g2o::Matrix3d rotation;

  rotation << r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
      r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2),
      r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2);

  return HomogeneousMatrix(rotation, ToVector3d<double>(t));
}

} // namespace clean_slam
