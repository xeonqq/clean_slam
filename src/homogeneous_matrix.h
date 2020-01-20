//
// Created by root on 1/20/20.
//

#ifndef CLEAN_SLAM_SRC_HOMOGENEOUS_MATRIX_H_
#define CLEAN_SLAM_SRC_HOMOGENEOUS_MATRIX_H_

#include <opencv2/core/core.hpp>
namespace clean_slam {
class HomogeneousMatrix {
public:
  HomogeneousMatrix(const cv::Mat &translation, const cv::Mat &rotation)
      : _translation{translation}, _rotation{rotation} {}
  const cv::Mat &GetTranslation() const { return _translation; }
  const cv::Mat &GetRotation() const { return _rotation; }

private:
  cv::Mat _translation;
  cv::Mat _rotation;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_HOMOGENEOUS_MATRIX_H_
