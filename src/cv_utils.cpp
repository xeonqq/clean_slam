//
// Created by root on 1/3/20.
//

#include "cv_utils.h"
#include <opencv2/core/hal/interface.h>

namespace clean_slam {

std::string MatType2Str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
  case CV_8U:
    r = "8U";
    break;
  case CV_8S:
    r = "8S";
    break;
  case CV_16U:
    r = "16U";
    break;
  case CV_16S:
    r = "16S";
    break;
  case CV_32S:
    r = "32S";
    break;
  case CV_32F:
    r = "32F";
    break;
  case CV_64F:
    r = "64F";
    break;
  default:
    r = "User";
    break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}
cv::MatExpr ZerosLike(cv::Mat m) {
  return cv::Mat::zeros(m.rows, m.cols, m.type());
}
cv::Mat ToTransformationMatrix(const cv::Mat R, const cv::Mat T) {
  cv::Mat transformation_matrix(3, 4, R.type());
  transformation_matrix(cv::Range::all(), cv::Range(0, 3)) = R;
  transformation_matrix.col(3) = T;
  return transformation_matrix;
}
} // namespace clean_slam
