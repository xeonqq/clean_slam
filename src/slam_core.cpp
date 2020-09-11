//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include <boost/foreach.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <third_party/spdlog/spdlog.h>
#define DEBUG
namespace clean_slam {

const double kViewAngleCosThreshold = std::cos(M_PI / 3);

void DrawMatches(const cv::Mat &image, const Frame &frame,
                 const Frame &prev_frame,
                 const std::vector<cv::DMatch> &matches) {
  static int i = 0;
  static cv::Mat prev_image;
  ++i;
  cv::Mat out;
  try {
    //    const auto& kf = frame.GetRefKeyFrame();
    if (!prev_image.empty()) {
      cv::drawMatches(prev_image, prev_frame.GetMatchedKeyPoints(), image,
                      frame.GetUndistortedKeyPoints(), matches, out);
      std::string png_name = std::string("matches_by_motion_track_knn") +
                             std::to_string(i) + std::string(".png");
      cv::imwrite(png_name, out);
    }
    prev_image = image;
  } catch (std::exception &e) {
    std::cerr << "TrackException: " << e.what() << std::endl;
  }
}

SlamCore::SlamCore(const cv::Mat &camera_intrinsics,
                   const cv::Mat &camera_distortion_coeffs,
                   OrbExtractor *orb_extractor, Optimizer *optimizer,
                   OptimizerOnlyPose *optimizer_only_pose, IViewer *viewer,
                   const OctaveScales &octave_scale)
    : _camera_intrinsic(camera_intrinsics), _orb_extractor(orb_extractor),
      _optimizer{optimizer}, _optimizer_only_pose{optimizer_only_pose},
      _viewer(viewer), _undistorted_image_boundary{camera_intrinsics,
                                                   camera_distortion_coeffs},
      _octave_scales{octave_scale}, _map{_octave_scales} {}

void SlamCore::ProcessFirstImage(const cv::Mat &image, double timestamp) {
  _undistorted_image_boundary.ComputeUndistortedCorners(image);
}

void SlamCore::TrackByMotionModel(const cv::Mat &image, double timestamp) {
  ++_num_frames_since_last_key_frame;
  // const velocity model
  const auto &prev_frame = _frames.back();
  const auto &prev_prev_frame = *(++_frames.rbegin());
  const auto &prev_Tcw = prev_frame.GetTcw();
  g2o::SE3Quat velocity =
      clean_slam::GetVelocity(prev_Tcw, prev_prev_frame.GetTcw());
  auto Tcw = velocity * prev_Tcw;

  Frame frame{_orb_extractor->DetectAndUndistortKeyPoints(image), &_map, Tcw,
              timestamp, prev_frame.GetRefKfVertex()};

  // project the matched map points in the previous frame to current frame
  std::vector<Eigen::Vector2d> points_reprojected = frame.ReprojectPoints3d(
      GetMapPointsPositions(prev_frame.GetMatchedMapPointsRng()),
      _camera_intrinsic);

  cv::Mat mask = cv::Mat(points_reprojected.size(), 1, CV_8U);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  for (size_t i = 0; i < points_reprojected.size(); ++i) {
    mask.at<uint8_t>(i) =
        IsPointWithInBounds(points_reprojected[i], x_bounds, y_bounds);
  }
  const int search_radius = 7;
  auto matches =
      frame.SearchByProjection(_orb_feature_matcher, points_reprojected,
                               prev_frame, mask, search_radius, _octave_scales);
  if (matches.size() < 20) {
    spdlog::info(
        "Search again with larger radius, num matched map points < 20: {}",
        matches.size());
    matches = frame.SearchByProjection(_orb_feature_matcher, points_reprojected,
                                       prev_frame, mask, search_radius * 2,
                                       _octave_scales);
    spdlog::info("After search again , num matched map points: {}",
                 matches.size());
  }

  const int kNumMatchedMapPointsForBA = 20;
  if (frame.GetNumMatchedMapPoints() > kNumMatchedMapPointsForBA) {
    spdlog::info("Num matched map points: {}", frame.GetNumMatchedMapPoints());
    frame.OptimizePose(_optimizer_only_pose);
  }
  TrackLocalMap(frame);

  InsertKeyFrame(frame);

  if (frame.GetNumMatchedMapPoints() > kNumMatchedMapPointsForBA) {
#if 0
    DrawMatches(image, frame, prev_frame, matches);
#endif
    _frames.push_back(std::move(frame));
  } else {
    spdlog::warn("Num matched map points < {}, only: {}",
                 kNumMatchedMapPointsForBA, frame.GetNumMatchedMapPoints());
    return;
  }
  const auto &current_frame = _frames.back();
  _viewer->OnNotify(Content{Tcw, {}});
  _viewer->OnNotify(image, current_frame.GetOrbFeatures());
}
void SlamCore::InsertKeyFrame(Frame &frame) {
  if (_num_frames_since_last_key_frame > 20 &&
      (frame.GetNumMatchedMapPoints() <
       frame.GetRefKeyFrameNumKeyPoints() * 0.9) &&
      frame.GetOrbFeatures().NumKeyPoints() > 50) {

    spdlog::debug("Insert kf");
    const auto kf = _map.AddKeyFrame(frame);
    const auto &key_frame = _map.GetKeyFrame(kf);

    _num_frames_since_last_key_frame = 0;
  }
}

void SlamCore::TrackLocalMap(Frame &frame) {
  auto key_frames_to_track_local_map = frame.GetKeyFramesToTrackLocalMap();
  std::set<MapPoint *> map_points_to_project;
  for (auto kf_vertex : key_frames_to_track_local_map) {
    const auto &key_frame = _map.GetKeyFrame(kf_vertex);
    boost::copy(
        key_frame.GetMatchedMapPointsRng(),
        std::inserter(map_points_to_project, map_points_to_project.end()));
  }
  for (auto map_point : frame.GetMatchedMapPointsRng()) {
    map_points_to_project.erase(map_point);
  }

  std::vector<Eigen::Vector2d> points_reprojected = frame.ReprojectPoints3d(
      GetMapPointsPositions(map_points_to_project), _camera_intrinsic);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  std::vector<MapPoint *> valid_map_points;
  std::vector<Eigen::Vector2d> valid_map_points_reprojected;
  std::vector<uint8_t> valid_map_points_octaves;
  auto combined = boost::combine(points_reprojected, map_points_to_project);
  for (auto it = combined.begin(); it != combined.end(); ++it) {
    Eigen::Vector2d point_reprojected;
    MapPoint *map_point;
    boost::tie(point_reprojected, map_point) = *it;

    //  1. check points in undistorted range
    if (!IsPointWithInBounds(point_reprojected, x_bounds, y_bounds))
      continue;

    // 2. check viewing direction feasible
    auto camera_pose_in_world = frame.GetTcw().inverse();
    const auto viewing_direction =
        (map_point->GetPoint3D() - camera_pose_in_world.translation())
            .normalized();
    const auto proj = map_point->GetViewDirection().dot(viewing_direction);
    if (proj < kViewAngleCosThreshold)
      continue;

    //  3. check new depth of points (wrt to new camera pose) is within
    //  scale pyramid range
    const auto depth =
        (map_point->GetPoint3D() - camera_pose_in_world.translation()).norm();
    if (!map_point->IsObservableFromDistance(depth))
      continue;

    int predicted_octave_level =
        map_point->PredictOctaveScale(depth, _octave_scales);
    valid_map_points_octaves.push_back(predicted_octave_level);
    valid_map_points.push_back(map_point);
    valid_map_points_reprojected.push_back(point_reprojected);
  }
  auto matches = frame.SearchUnmatchedKeyPointsByProjection(
      _orb_feature_matcher, valid_map_points, valid_map_points_reprojected,
      valid_map_points_octaves, 7, _octave_scales);
  spdlog::info("Num additional matched map points track local map: {}",
               matches);
  if (frame.GetNumMatchedMapPoints() > 20) {
    spdlog::info("optimize in track local map with num map points: {}",
                 frame.GetNumMatchedMapPoints());
    frame.OptimizePose(_optimizer_only_pose);
  }
}

CameraTrajectory SlamCore::GetTrajectory() const {
  CameraTrajectory trajectory;
  boost::range::transform(
      _frames, std::back_inserter(trajectory), [](const auto &frame) {
        return StampedTransformation{frame.GetTcw(), frame.GetTimestamp()};
      });
  return trajectory;
}

} // namespace clean_slam
