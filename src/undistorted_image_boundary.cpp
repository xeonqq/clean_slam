//
// Created by root on 4/17/20.
//
#include <iostream>
#include <opencv2/imgproc.hpp>

#include "cv_utils.h"
#include "undistorted_image_boundary.h"

namespace clean_slam {
UndistortedImageBoundary::UndistortedImageBoundary(
    const cv::Mat &camera_intrinsics, const cv::Mat &camera_distortion_coeffs)
    : _camera_distortion_coeffs(camera_distortion_coeffs),
      _camera_intrinsics(camera_intrinsics) {}

const ImageCorners &UndistortedImageBoundary::ComputeUndistortedCorners(
    const cv::Mat &image) const {
  const float h = image.rows;
  const float w = image.cols;
  std::array<cv::Point2f, 4> corners{};
  corners[TopLeft] = {0, 0};
  corners[BottomLeft] = {0, h};
  corners[TopRight] = {w, 0};
  corners[BottomRight] = {w, h};

  cv::undistortPoints(corners, _undistorted_corners, _camera_intrinsics,
                      _camera_distortion_coeffs, cv::noArray(),
                      _camera_intrinsics);

  //  std::cout << _undistorted_corners << std::endl;
  return _undistorted_corners;
}
const ImageCorners &UndistortedImageBoundary::GetUndistortedCorners() const {
  return _undistorted_corners;
}

} // namespace clean_slam
