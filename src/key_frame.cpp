//
// Created by root on 4/11/20.
//
#include <third_party/g2o/g2o/types/se3quat.h>

#include "key_frame.h"
namespace clean_slam {

KeyFrame::KeyFrame(const g2o::SE3Quat &Tcw,
                   std::vector<Eigen::Vector3d> &&points_3d,
                   const cv::Mat &descriptors)
    : _Tcw(Tcw), _points_3d(std::move(points_3d)), _descriptors(descriptors) {}

const g2o::SE3Quat &KeyFrame::GetTcw() const { return _Tcw; }

const std::vector<Eigen::Vector3d> &KeyFrame::GetPoints3D() const {
  return _points_3d;
}

const cv::Mat &KeyFrame::GetDescriptors() const { return _descriptors; }
} // namespace clean_slam