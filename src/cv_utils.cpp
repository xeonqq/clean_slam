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

std::string MatType2Str(cv::Mat mat) {
  std::ostringstream os;
  os << "type: ";
  os << MatType2Str(mat.type());
  os << " rows*cols: ";
  os << mat.size;
  ;
  return os.str();
}

cv::MatExpr ZerosLike(cv::Mat m) {
  return cv::Mat::zeros(m.rows, m.cols, m.type());
}
cv::Mat ToTransformationMatrix(const cv::Mat &R, const cv::Mat &T) {
  cv::Mat extrinsics(3, 4, T.type());
  // assigning using MatExpr, if not values are not assigned properly,
  // dangerous!
  extrinsics(cv::Range::all(), cv::Range(0, 3)) = R * 1.0;
  extrinsics.col(3) = T * 1.0;
  return extrinsics;
}

cv::Mat NormPoints(const cv::Mat &m) {
  // calculate norm of points, i.e. 3 dim points means 3 channels
  cv::Mat norms = m.mul(m);
  norms = SumChannels(norms);
  cv::sqrt(norms, norms);
  return norms;
}

cv::Mat SumChannels(const cv::Mat &m) { return SumColumns(m.reshape(1)); }

cv::Mat SumColumns(const cv::Mat &m) {
  assert(m.cols > 1);
  cv::Mat result = m.col(0);
  for (int i = 1; i < m.cols; ++i) {
    result += m.col(i);
  }
  return result;
}
} // namespace clean_slam
