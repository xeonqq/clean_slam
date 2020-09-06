//
// Created by root on 9/2/20.
//

#include "frame.h"
#include "gtest/gtest.h"
#include <gmock/gmock-matchers.h>
namespace clean_slam {

TEST(Frame, GetKeyFramesForLocalMapping) {
  // Given
  OctaveScales octave_scales{1.2};
  Map map{octave_scales};
  cv::Mat point_descriptor = cv::Mat::zeros(1, 32, CV_8UC1);
  cv::Mat points_descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
  auto kf0 = map.AddKeyFrame({}, {{}}, point_descriptor);
  auto kf1 = map.AddKeyFrame(
      {}, {{}, {}}, points_descriptors); // 2 key points, one share with kf0 and
                                         // testing frame, one share with kf2
  auto kf2 = map.AddKeyFrame({}, {{}, {}}, points_descriptors);
  auto kf3 = map.AddKeyFrame(
      {}, {{}}, point_descriptor); // for noise, not a neighbor, not share map
                                   // point with testing frame
  map.AddKeyFramesWeight(kf0, kf1, 1);
  map.AddKeyFramesWeight(kf1, kf2, 1);
  map.AddKeyFramesWeight(kf2, kf3, 1);

  std::vector<Eigen::Vector3d> points_3d_kf0_1{{1, 1, 1}};
  auto map_points_kf0_1 = map.AddMapPoints(points_3d_kf0_1, {0}, kf0, {0}, kf1);

  std::vector<Eigen::Vector3d> points_3d_kf1_2{{1, 1, 2}};
  map.AddMapPoints(points_3d_kf1_2, {1}, kf1, {0}, kf2);

  std::vector<Eigen::Vector3d> points_3d_kf2_3{{2, 1, 2}};
  map.AddMapPoints(points_3d_kf2_3, {1}, kf2, {0}, kf3);

  Frame frame{{{}}, map_points_kf0_1, &map, {}, {}, kf1};

  // When
  auto key_frames_for_local_mapping = frame.GetKeyFramesToTrackLocalMap();

  EXPECT_THAT(key_frames_for_local_mapping,
              ::testing::UnorderedElementsAre(kf0, kf1, kf2));
}
} // namespace clean_slam