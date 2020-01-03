//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_

#include "frame.h"

namespace clean_slam {

class KeyPixelsPair {
public:
  KeyPixelsPair(const std::vector<cv::Point2f> &key_pixels_curr_frame,
                const std::vector<cv::Point2f> &key_pixels_prev_frame)
      : key_pixels_curr_frame(key_pixels_curr_frame),
        key_pixels_prev_frame(key_pixels_prev_frame) {}

  KeyPixelsPair(std::vector<cv::Point2f> &&key_pixels_curr_frame,
                std::vector<cv::Point2f> &&key_pixels_prev_frame)
      : key_pixels_curr_frame(std::move(key_pixels_curr_frame)),
        key_pixels_prev_frame(std::move(key_pixels_prev_frame)) {}
  const std::vector<cv::Point2f> &GetKeyPixelsCurrFrame() const {
    return key_pixels_curr_frame;
  }
  const std::vector<cv::Point2f> &GetKeyPixelsPrevFrame() const {
    return key_pixels_prev_frame;
  }

private:
  std::vector<cv::Point2f> key_pixels_curr_frame;
  std::vector<cv::Point2f> key_pixels_prev_frame;
};

template <typename Query>
std::vector<cv::Point2f>
GetKeyPointsInPixelFromMatches(const std::vector<cv::KeyPoint> &key_points,
                               const std::vector<cv::DMatch> &matches) {
  std::vector<cv::Point2f> key_points_in_pixel;
  for (const auto &match_idx : Query()(matches)) {
    key_points_in_pixel.push_back(key_points[match_idx].pt);
  }
  return key_points_in_pixel;
}

class OrbFeatureMatcher {
public:
  OrbFeatureMatcher();
  KeyPixelsPair Match(const OrbFeatures &orb_features, const Frame &prev_frame);
  const std::vector<cv::DMatch> &GetGoodMatches() const;

private:
  void ComputeGoodMatches(float descriptor_distance_threshold);

private:
  cv::FlannBasedMatcher _matcher;
  std::vector<cv::DMatch> _matches;
  std::vector<cv::DMatch> _good_matches;
};

} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_
