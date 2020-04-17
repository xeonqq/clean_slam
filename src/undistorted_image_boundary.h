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
  UndistortedImageBoundary(const cv::Mat &camera_intrinsics,
                           const cv::Mat &camera_distortion_coeffs);
  enum ImageCornerPosition {
    TopLeft = 0,
    BottomLeft,
    TopRight,
    BottomRight,
  };

  const ImageCorners &ComputeUndistortedCorners(const cv::Mat &image) const;
  const ImageCorners &GetUndistortedCorners() const;

private:
  cv::Mat _camera_distortion_coeffs;
  cv::Mat _camera_intrinsics;
  ImageCorners _undistorted_corners;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_UNDISTORTED_IMAGE_BOUNDARY_H_
