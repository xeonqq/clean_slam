//
// Created by root on 9/7/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_ARTIFACT_H_
#define CLEAN_SLAM_SRC_FRAME_ARTIFACT_H_

#include "map_point.h"
#include "octave_scales.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include <Eigen/src/Core/Matrix.h>
namespace clean_slam {

class Frame;
class FrameArtifact {
public:
  FrameArtifact(Frame *frame, std::vector<int> &&matched_key_points_idxs,
                std::vector<cv::DMatch> &&matches)
      : _frame(frame),
        _matched_key_points_idxs(std::move(matched_key_points_idxs)),
        _matches{std::move(matches)} {}

  size_t SearchUnmatchedKeyPointsByProjection(
      const OrbFeatureMatcher &matcher,
      const std::vector<MapPoint *> &map_points,
      const std::vector<Eigen::Vector2d> &projected_map_points,
      const std::vector<uint8_t> &map_points_octaves, int search_radius,
      const OctaveScales &octave_scales);

  size_t NumOfMatches() const { return _matched_key_points_idxs.size(); }
  Frame &GetFrame();
  const Frame &GetFrame() const;
  const std::vector<cv::DMatch> &GetMatches() const { return _matches; }

private:
  std::vector<cv::KeyPoint> GetUnmatchedKeyPoints() const;

  cv::Mat GetUnmatchedKeyPointsDescriptors() const;

private:
  Frame *_frame;
  std::vector<int> _matched_key_points_idxs;
  std::vector<cv::DMatch> _matches; // for debug only
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_FRAME_ARTIFACT_H_
