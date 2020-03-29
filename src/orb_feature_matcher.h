//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_

#include "frame.h"

namespace clean_slam {
using KeyPointsPair =
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>;

class PointsPair {
public:
  PointsPair(const std::vector<cv::Point2f> &key_pixels_curr_frame,
             const std::vector<cv::Point2f> &key_pixels_prev_frame)
      : key_pixels_curr_frame(key_pixels_curr_frame),
        key_pixels_prev_frame(key_pixels_prev_frame) {}

  PointsPair(std::vector<cv::Point2f> &&key_pixels_curr_frame,
             std::vector<cv::Point2f> &&key_pixels_prev_frame)
      : key_pixels_curr_frame(std::move(key_pixels_curr_frame)),
        key_pixels_prev_frame(std::move(key_pixels_prev_frame)) {}
  const std::vector<cv::Point2f> &GetPointsCurrFrame() const {
    return key_pixels_curr_frame;
  }
  const std::vector<cv::Point2f> &GetPointsPrevFrame() const {
    return key_pixels_prev_frame;
  }

private:
  std::vector<cv::Point2f> key_pixels_curr_frame;
  std::vector<cv::Point2f> key_pixels_prev_frame;
};

class OrbFeatureMatcher {
public:
  OrbFeatureMatcher();
  std::vector<cv::DMatch> Match(const Frame &curr_frame,
                                const Frame &prev_frame);
  static PointsPair
  GetMatchedPointsPair(const Frame &curr_frame, const Frame &prev_frame,
                       const std::vector<cv::DMatch> &matches);
  static PointsPair
  GetMatchedPointsPairUndistorted(const Frame &curr_frame,
                                  const Frame &prev_frame,
                                  const std::vector<cv::DMatch> &matches);

  static KeyPointsPair
  GetMatchedKeyPointsPairUndistorted(const Frame &curr_frame,
                                     const Frame &prev_frame,
                                     const std::vector<cv::DMatch> &matches);

private:
  std::vector<cv::DMatch>
  ComputeGoodMatches(const std::vector<cv::DMatch> &matches,
                     float descriptor_distance_threshold);

private:
  cv::FlannBasedMatcher _matcher;
};

} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_
