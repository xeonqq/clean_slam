//
// Created by root on 4/17/20.
//

#ifndef CLEAN_SLAM_SRC_UNDISTORTED_IMAGE_BOUNDARY_H_
#define CLEAN_SLAM_SRC_UNDISTORTED_IMAGE_BOUNDARY_H_

#include <array>
#include <opencv2/core/mat.hpp>
namespace clean_slam {
using ImageCorners = std::array<cv::Point2f, 4>;

class UndistortedImageBoundary {

public:
  enum ImageCornerPosition {
    TopLeft = 0,
    BottomLeft,
    TopRight,
    BottomRight,
  };

  UndistortedImageBoundary(const cv::Mat &camera_intrinsics,
                           const cv::Mat &camera_distortion_coeffs);

  const ImageCorners &ComputeUndistortedCorners(const cv::Mat &image);
  const ImageCorners &GetUndistortedCorners() const;
  const std::pair<float, float> &GetXBounds() const;
  const std::pair<float, float> &GetYBounds() const;

private:
  void ComputeBounds();

private:
  cv::Mat _camera_distortion_coeffs;
  cv::Mat _camera_intrinsics;
  ImageCorners _undistorted_corners;
  std::pair<float, float> _x_bounds;
  std::pair<float, float> _y_bounds;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_UNDISTORTED_IMAGE_BOUNDARY_H_
