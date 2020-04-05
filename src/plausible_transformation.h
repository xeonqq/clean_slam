//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_PLAUSIBLE_TRANSFORMATION_H_
#define CLEAN_SLAM_SRC_PLAUSIBLE_TRANSFORMATION_H_

#include "homogeneous_matrix.h"
#include <opencv2/core/mat.hpp>

namespace clean_slam {

class PlausibleTransformation {
public:
  PlausibleTransformation(const cv::Mat &r, const cv::Mat &t,
                          int num_of_good_points,
                          const cv::Mat &good_points_mask,
                          const cv::Mat &triangulated_points,
                          bool has_similar_good);
  PlausibleTransformation() = default;
  const cv::Mat &R() const;
  const cv::Mat &T() const;
  int GetNumOfGoodPoints() const;
  const cv::Mat &GetGoodPointsMask() const;
  HomogeneousMatrix GetHomogeneousMatrix() const;
  const cv::Mat &GetTriangulatedPoints() const;
  const std::vector<Eigen::Vector3d> &GetGoodTriangulatedPoints() const;
  bool IsGood() const;

private:
  cv::Mat r;
  cv::Mat t;
  int num_of_good_points;
  cv::Mat good_points_mask;
  cv::Mat _triangulated_points;
  std::vector<Eigen::Vector3d> _good_triangulated_points;

private:
  bool _has_similar_good;
  static constexpr int kNumOfGoodPointsThreshold{50};
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_PLAUSIBLE_TRANSFORMATION_H_
