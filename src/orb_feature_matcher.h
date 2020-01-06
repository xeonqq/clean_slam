//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_

#include "frame.h"

namespace clean_slam {

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
  void Match(const Frame &curr_frame, const Frame &prev_frame);
  PointsPair GetMatchedPointsPair() const;
  PointsPair GetMatchedPointsPairUndistorted() const;

  const std::vector<cv::DMatch> &GetGoodMatches() const;

private:
  void ComputeGoodMatches(float descriptor_distance_threshold);

private:
  cv::FlannBasedMatcher _matcher;
  std::vector<cv::DMatch> _matches;
  std::vector<cv::DMatch> _good_matches;
  const Frame *_current_frame;
  const Frame *_previous_frame;
};

} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_SLAM_CORE_CPP_ORBFEATUREMATCHER_H_
