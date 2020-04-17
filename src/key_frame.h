//
// Created by root on 4/11/20.
//

#ifndef CLEAN_SLAM_SRC_KEY_FRAME_H_
#define CLEAN_SLAM_SRC_KEY_FRAME_H_

#include "bound.h"
#include "octave_sigma_scales.h"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

namespace clean_slam {
class KeyFrame {
public:
  KeyFrame(const g2o::SE3Quat &Tcw, std::vector<Eigen::Vector3d> &&points_3d,
           const cv::Mat &descriptors, std::vector<Bound> &&distance_bounds);

  static KeyFrame Create(const g2o::SE3Quat &Tcw,
                         std::vector<Eigen::Vector3d> &&points_3d,
                         const cv::Mat &descriptors,
                         const std::vector<cv::KeyPoint> &key_points,
                         const OctaveScales &octave_scales);

  const g2o::SE3Quat &GetTcw() const;
  const std::vector<Eigen::Vector3d> &GetPoints3D() const;
  const cv::Mat &GetDescriptors() const;
  const std::vector<Bound> &GetDistanceBounds() const;

private:
  g2o::SE3Quat _Tcw;
  std::vector<Eigen::Vector3d> _points_3d;

  // descriptors are 500*32 8UC1 mat, each row is a descriptor
  // in the case of ORB, the descriptor is binary, meaning, 32*8 = 256bit
  // hamming distance needs to be used to compare descriptors
  cv::Mat _descriptors;
  std::vector<Bound> _distance_bounds;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_KEY_FRAME_H_
