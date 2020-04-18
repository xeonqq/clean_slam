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

const ImageCorners &
UndistortedImageBoundary::ComputeUndistortedCorners(const cv::Mat &image) {
  const float h = image.rows;
  const float w = image.cols;
  ImageCorners corners{};
  corners[TopLeft] = {0, 0};
  corners[BottomLeft] = {0, h};
  corners[TopRight] = {w, 0};
  corners[BottomRight] = {w, h};

  cv::undistortPoints(corners, _undistorted_corners, _camera_intrinsics,
                      _camera_distortion_coeffs, cv::noArray(),
                      _camera_intrinsics);

  //  std::cout << _undistorted_corners << std::endl;
  ComputeBounds();
  return _undistorted_corners;
}

void UndistortedImageBoundary::ComputeBounds() {
  const auto x_lb =
      std::max(_undistorted_corners[UndistortedImageBoundary::TopLeft].x,
               _undistorted_corners[UndistortedImageBoundary::BottomLeft].x);
  const auto x_hb =
      std::min(_undistorted_corners[UndistortedImageBoundary::TopRight].x,
               _undistorted_corners[UndistortedImageBoundary::BottomRight].x);
  _x_bounds = {x_lb, x_hb};

  const auto y_lb =
      std::max(_undistorted_corners[UndistortedImageBoundary::TopLeft].y,
               _undistorted_corners[UndistortedImageBoundary::TopRight].y);
  const auto y_hb =
      std::min(_undistorted_corners[UndistortedImageBoundary::BottomLeft].y,
               _undistorted_corners[UndistortedImageBoundary::BottomRight].y);

  _y_bounds = {y_lb, y_hb};
}

const ImageCorners &UndistortedImageBoundary::GetUndistortedCorners() const {
  return _undistorted_corners;
}
const Bound &UndistortedImageBoundary::GetXBounds() const { return _x_bounds; }
const Bound &UndistortedImageBoundary::GetYBounds() const { return _y_bounds; }

} // namespace clean_slam
