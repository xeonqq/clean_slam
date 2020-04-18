//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_CV_UTILS_H_
#define CLEAN_SLAM_SRC_CV_UTILS_H_

#include "bound.h"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

namespace clean_slam {

std::string MatType2Str(int type);
std::string MatType2Str(cv::Mat mat);

struct QueryIdxs {
  std::vector<int> operator()(const std::vector<cv::DMatch> &matches) {
    std::vector<int> idxs;
    idxs.reserve(matches.size());
    std::transform(matches.begin(), matches.end(), std::back_inserter(idxs),
                   [](const auto &match) { return match.queryIdx; });
    return idxs;
  }
};

struct TrainIdxs {
  std::vector<int> operator()(const std::vector<cv::DMatch> &matches) {
    std::vector<int> idxs;
    idxs.reserve(matches.size());
    std::transform(matches.begin(), matches.end(), std::back_inserter(idxs),
                   [](const auto &match) { return match.trainIdx; });
    return idxs;
  }
};

cv::MatExpr ZerosLike(cv::Mat m);

template <typename T, typename V = uint8_t>
std::vector<T> FilterByMask(const std::vector<T> &vec, const cv::Mat &mask) {
  assert(static_cast<int>(vec.size()) == mask.rows);
  assert(mask.cols == 1);
  std::vector<T> result;
  for (int i = 0; i < mask.rows; ++i) {
    if (mask.at<V>(i) == 1) {
      result.push_back(vec[i]);
    }
  }
  return result;
}
cv::Mat ToMat(const std::vector<Eigen::Vector3d> &vec);
std::vector<Eigen::Vector3d> ToStdVectorByMask(const cv::Mat &mat,
                                               const cv::Mat &mask);

cv::Mat FilterByMask(const cv::Mat &mat, const cv::Mat &mask);

template <typename T>
std::vector<T> FilterByIndex(const std::vector<T> &vec,
                             const std::vector<int> &indexes) {
  std::vector<T> result;
  for (auto index : indexes) {
    result.push_back(vec[index]);
  }
  return result;
}

cv::Mat FilterByIndex(const cv::Mat &mat, const std::vector<int> &indexes);

Eigen::Vector2d Point2fToVector2d(const cv::Point2f &point2f);

template <typename T> Eigen::Vector3d ToVector3d(const cv::Mat &point3) {
  Eigen::Vector3d vec(point3.at<T>(0), point3.at<T>(1), point3.at<T>(2));
  return vec;
}

cv::Mat ToTransformationMatrix(const cv::Mat &R, const cv::Mat &T);

cv::Mat NormPoints(const cv::Mat &m);
cv::Mat SumChannels(const cv::Mat &m);
cv::Mat SumColumns(const cv::Mat &m);



bool IsPointWithInBounds(const Eigen::Vector2d &point, const Bound &x_bounds,
                         const Bound &y_bounds);

template <template <typename, typename> class Container, typename Value,
          typename Allocator = std::allocator<Value>>
std::ostream &operator<<(std::ostream &os,
                         const Container<Value, Allocator> &values) {
  os << "[";
  for (const auto &v : values) {
    os << v << "; \n";
  }
  os << "]";
  return os;
}

template <typename T, size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &values) {
  os << "[";
  for (const auto &v : values) {
    os << v << "; \n";
  }
  os << "]";
  return os;
}
std::ostream &operator<<(std::ostream &os, const cv::KeyPoint &v);

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CV_UTILS_H_
