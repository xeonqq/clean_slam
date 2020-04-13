#include "orb_extractor.h"
#include "cv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace clean_slam {

OrbExtractor::OrbExtractor(cv::Ptr<cv::Feature2D> &&detector,
                           const cv::Mat &camera_intrinsics,
                           const cv::Mat &camera_distortion_coeffs)
    : _detector(std::move(detector)), _camera_intrinsics(camera_intrinsics),
      _camera_distortion_coeffs(camera_distortion_coeffs) {}

OrbFeatures OrbExtractor::Detect(cv::Mat image) {
  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  _detector->detectAndCompute(image, cv::noArray(), key_points, descriptors);
  return OrbFeatures{std::move(key_points), descriptors};
}

OrbFeatures OrbExtractor::DetectAndUndistortKeyPoints(const cv::Mat &image) {
  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  _detector->detectAndCompute(image, cv::noArray(), key_points, descriptors);
  return OrbFeatures{std::move(key_points), descriptors, _camera_intrinsics,
                     _camera_distortion_coeffs};
}


OrbFeatures::OrbFeatures(std::vector<cv::KeyPoint> &&key_points,
                         const cv::Mat &descriptors)
    : _key_points{std::move(key_points)}, _camera_lens_has_disortion{false},
      _descriptors{descriptors} {}

OrbFeatures::OrbFeatures(std::vector<cv::KeyPoint> &&key_points,
                         const cv::Mat &descriptors,
                         const cv::Mat &camera_intrinsics,
                         const cv::Mat &camera_distortion_coeffs)
    : _key_points{std::move(key_points)}, _camera_lens_has_disortion{true},
      _descriptors{descriptors} {
  std::vector<cv::Point2f> undistorted_points;
  std::vector<cv::Point2f> points;
  cv::KeyPoint::convert(_key_points, points);
  cv::undistortPoints(points, undistorted_points, camera_intrinsics,
                      camera_distortion_coeffs, cv::noArray(),
                      camera_intrinsics);

  _undistorted_key_points.reserve(_key_points.size());
  std::transform(
      _key_points.begin(), _key_points.end(), undistorted_points.begin(),
      std::back_inserter(_undistorted_key_points),
      [](const auto &key_point, const auto &undistorted_point) {
        std::decay_t<decltype(key_point)> undistorted_key_point = key_point;
        undistorted_key_point.pt = undistorted_point;
        return undistorted_key_point;
      });
}

const std::vector<cv::KeyPoint> &OrbFeatures::GetUndistortedKeyPoints() const {
  if (_camera_lens_has_disortion)
    return _undistorted_key_points;
  else
    return _key_points;
}

const std::vector<cv::KeyPoint> &OrbFeatures::GetKeyPoints() const {
  return _key_points;
}

const cv::Mat &OrbFeatures::GetDescriptors() const { return _descriptors; }
} // namespace clean_slam
