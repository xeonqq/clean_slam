//
// Created by root on 1/3/20.
//

#include "orb_feature_matcher.h"
#include "cv_utils.h"
namespace clean_slam {
OrbFeatureMatcher::OrbFeatureMatcher()
    : _matcher{cv::FlannBasedMatcher(
          cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2))} {}

std::vector<cv::DMatch> OrbFeatureMatcher::Match(const Frame &curr_frame,
                                                 const Frame &prev_frame) {
  std::vector<cv::DMatch> matches;
  _matcher.match(curr_frame.GetDescriptors(), prev_frame.GetDescriptors(),
                 matches);

  /*  double max_dist = 0;
    double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for (const auto &m : matches) {
      double dist = m.distance;
      if (dist < min_dist)
        min_dist = dist;
      if (dist > max_dist)
        max_dist = dist;
    }*/

  //  printf("-- Max dist : %f \n", max_dist);
  //  printf("-- Min dist : %f \n", min_dist);

  return ComputeGoodMatches(matches, 20);
}

std::vector<cv::DMatch>
OrbFeatureMatcher::ComputeGoodMatches(const std::vector<cv::DMatch> &matches,
                                      float descriptor_distance_threshold) {
  // Retrieve matches have distance lower than descriptor distance threshold
  std::vector<cv::DMatch> good_matches;
  std::copy_if(matches.begin(), matches.end(), std::back_inserter(good_matches),
               [descriptor_distance_threshold](const auto &match) {
                 return match.distance < descriptor_distance_threshold;
               });
  return good_matches;
}

PointsPair OrbFeatureMatcher::GetMatchedPointsPair(
    const Frame &curr_frame, const Frame &prev_frame,
    const std::vector<cv::DMatch> &matches) {
  std::vector<cv::Point2f> key_pixels_curr_frame;
  cv::KeyPoint::convert(curr_frame.GetKeyPoints(), key_pixels_curr_frame,
                        QueryIdxs{}(matches));

  std::vector<cv::Point2f> key_pixels_prev_frame;
  cv::KeyPoint::convert(prev_frame.GetKeyPoints(), key_pixels_prev_frame,
                        TrainIdxs{}(matches));

  return PointsPair{std::move(key_pixels_curr_frame),
                    std::move(key_pixels_prev_frame)};
}

PointsPair OrbFeatureMatcher::GetMatchedPointsPairUndistorted(
    const Frame &curr_frame, const Frame &prev_frame,
    const std::vector<cv::DMatch> &matches) {
  std::vector<cv::Point2f> key_points_curr_frame;
  cv::KeyPoint::convert(curr_frame.GetKeyPointsUndistorted(),
                        key_points_curr_frame, QueryIdxs{}(matches));

  std::vector<cv::Point2f> key_points_prev_frame;
  cv::KeyPoint::convert(prev_frame.GetKeyPointsUndistorted(),
                        key_points_prev_frame, TrainIdxs{}(matches));

  return PointsPair{std::move(key_points_curr_frame),
                    std::move(key_points_prev_frame)};
}

KeyPointsPair OrbFeatureMatcher::GetMatchedKeyPointsPairUndistorted(
    const Frame &curr_frame, const Frame &prev_frame,
    const std::vector<cv::DMatch> &matches) {
  auto matched_key_points_curr =
      FilterByIndex(curr_frame.GetKeyPointsUndistorted(), QueryIdxs{}(matches));
  auto matched_key_points_prev =
      FilterByIndex(curr_frame.GetKeyPointsUndistorted(), TrainIdxs{}(matches));

  return {std::move(matched_key_points_prev),
          std::move(matched_key_points_curr)};
}

} // namespace clean_slam
