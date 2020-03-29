//
// Created by root on 3/10/20.
//
#include "homogeneous_matrix.h"
#include "cv_utils.h"
#include <opencv2/core/eigen.hpp>

namespace clean_slam {

HomogeneousMatrix CreateHomogeneousMatrix(const cv::Mat &r, const cv::Mat &t) {
  g2o::Vector3d translation(t.at<double>(0), t.at<double>(1), t.at<double>(2));
  g2o::Matrix3d rotation;

  rotation << r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
      r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2),
      r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2);

  //  cv::cv2eigen(r, rotation);
  //  cv::cv2eigen(t, translation);
  return HomogeneousMatrix(rotation, translation);
}

} // namespace clean_slam
