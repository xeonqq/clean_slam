//
// Created by root on 1/3/20.
//

#include "orb_feature_matcher.h"
#include "cv_utils.h"
namespace clean_slam {
OrbFeatureMatcher::OrbFeatureMatcher()
    : _matcher{cv::FlannBasedMatcher(
          cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2))} {}

void OrbFeatureMatcher::Match(const Frame &curr_frame,
                              const Frame &prev_frame) {
  _matches.clear();
  _current_frame = &curr_frame;
  _previous_frame = &prev_frame;
  _matcher.match(curr_frame.GetDescriptors(), prev_frame.GetDescriptors(),
                 _matches);

  double max_dist = 0;
  double min_dist = 100;
  //-- Quick calculation of max and min distances between keypoints
  for (const auto &m : _matches) {
    double dist = m.distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  ComputeGoodMatches(20);
}

void OrbFeatureMatcher::ComputeGoodMatches(
    float descriptor_distance_threshold) {
  // Retrieve matches have distance lower than descriptor distance threshold
  _good_matches.clear();
  std::copy_if(_matches.begin(), _matches.end(),
               std::back_inserter(_good_matches),
               [descriptor_distance_threshold](const auto &match) {
                 return match.distance < descriptor_distance_threshold;
               });
}

const std::vector<cv::DMatch> &OrbFeatureMatcher::GetGoodMatches() const {
  return _good_matches;
}

PointsPair OrbFeatureMatcher::GetMatchedPointsPair() const {
  std::vector<cv::Point2f> key_pixels_curr_frame;
  cv::KeyPoint::convert(_current_frame->GetKeyPoints(), key_pixels_curr_frame,
                        QueryIdxs{}(_good_matches));

  std::vector<cv::Point2f> key_pixels_prev_frame;
  cv::KeyPoint::convert(_previous_frame->GetKeyPoints(), key_pixels_prev_frame,
                        TrainIdxs{}(_good_matches));

  return PointsPair{std::move(key_pixels_curr_frame),
                    std::move(key_pixels_prev_frame)};
}

PointsPair OrbFeatureMatcher::GetMatchedPointsPairUndistorted() const {
  std::vector<cv::Point2f> key_pixels_curr_frame;
  cv::KeyPoint::convert(_current_frame->GetKeyPointsUndistorted(),
                        key_pixels_curr_frame, QueryIdxs{}(_good_matches));

  std::vector<cv::Point2f> key_pixels_prev_frame;
  cv::KeyPoint::convert(_previous_frame->GetKeyPointsUndistorted(),
                        key_pixels_prev_frame, TrainIdxs{}(_good_matches));

  return PointsPair{std::move(key_pixels_curr_frame),
                    std::move(key_pixels_prev_frame)};
}

} // namespace clean_slam
