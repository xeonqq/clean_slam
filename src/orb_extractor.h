//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_ORB_EXTRACTOR_H_
#define CLEAN_SLAM_SRC_ORB_EXTRACTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
namespace clean_slam {

class OrbFeatures {
public:
  OrbFeatures() = default;
  OrbFeatures(std::vector<cv::KeyPoint> &&key_points,
              const cv::Mat &descriptors);

  OrbFeatures(std::vector<cv::KeyPoint> &&key_points,
              const cv::Mat &descriptors, const cv::Mat &camera_intrinsics,
              const cv::Mat &distortion_coeffs);
  const std::vector<cv::KeyPoint> &GetKeyPoints() const;
  const cv::Mat &GetDescriptors() const;

  const std::vector<cv::KeyPoint> &GetUndistortedKeyPoints() const;

private:
  std::vector<cv::KeyPoint> _key_points;
  std::vector<cv::KeyPoint> _undistorted_key_points;
  bool _camera_lens_has_disortion;
  // descriptors are 500*32 8UC1 mat, each row is a descriptor
  // in the case of ORB, the descriptor is binary, meaning, 32*8 = 256bit
  // hamming distance needs to be used to compare descriptors
  cv::Mat _descriptors;
};

class OrbExtractor {
public:
  OrbExtractor();

  void SetCameraIntrinsicsAndDistortionCoeffs(
      const cv::Mat &camera_intrinsics,
      const cv::Mat &camera_distortion_coeffs);
  OrbFeatures Detect(cv::Mat image);
  OrbFeatures DetectAndUndistortKeyPoints(const cv::Mat &image);

private:
  cv::Ptr<cv::Feature2D> _detector;
  cv::Mat _camera_intrinsics;
  cv::Mat _camera_distortion_coeffs;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_ORB_EXTRACTOR_H_
