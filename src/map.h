//
// Created by root on 5/3/20.
//

#ifndef CLEAN_SLAM_SRC_MAP_H_
#define CLEAN_SLAM_SRC_MAP_H_

#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

#include "bound.h"
#include "octave_scales.h"

namespace clean_slam {
class Map {
public:
  Map() = default;
  Map(std::vector<Eigen::Vector3d> &&points_3d, const cv::Mat &descriptors,
      std::vector<Bound> &&distance_bounds);

  static Map Create(const g2o::SE3Quat &Tcw,
                    std::vector<Eigen::Vector3d> &&points_3d,
                    const cv::Mat &descriptors,
                    const std::vector<cv::KeyPoint> &key_points,
                    const OctaveScales &octave_scales);

  void Construct(const g2o::SE3Quat &Tcw,
                 std::vector<Eigen::Vector3d> &&points_3d,
                 const cv::Mat &descriptors,
                 const std::vector<cv::KeyPoint> &key_points,
                 const OctaveScales &octave_scales);

  static std::vector<Bound>
  Calculate3DPointsDistanceBounds(const g2o::SE3Quat &Tcw,
                                  const std::vector<cv::KeyPoint> &key_points,
                                  std::vector<Eigen::Vector3d> &points_3d,
                                  const OctaveScales &octave_scales);
  const std::vector<Eigen::Vector3d> &GetPoints3D() const;
  const cv::Mat &GetDescriptors() const;
  const std::vector<int> &GetOctaves() const;

private:
  std::vector<Eigen::Vector3d> _points_3d;
  // descriptors are number_points*32 8UC1 mat, each row is a descriptor
  // in the case of ORB, the descriptor is binary, meaning, 32*8 = 256bit
  // hamming distance needs to be used to compare descriptors
  cv::Mat _descriptors;
  std::vector<Bound> _distance_bounds;
  std::vector<int> _octaves;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_MAP_H_
