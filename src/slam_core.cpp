//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include <boost/range/algorithm.hpp>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {

SlamCore::SlamCore(const cv::Mat &camera_intrinsics,
                   const cv::Mat &camera_distortion_coeffs,
                   OrbExtractor *orb_extractor, Optimizer *optimizer,
                   Viewer *viewer, const OctaveScales &octave_scale)
    : _camera_intrinsic(camera_intrinsics),
      _orb_extractor(orb_extractor), _optimizer{optimizer},
      _viewer(viewer), _undistorted_image_boundary{camera_intrinsics,
                                                   camera_distortion_coeffs},
      _octave_scales{octave_scale} {}

void SlamCore::ProcessFirstImage(const cv::Mat &image, double timestamp) {
  _undistorted_image_boundary.ComputeUndistortedCorners(image);
}

void SlamCore::TrackByMotionModel(const cv::Mat &image, double timestamp) {
  const auto orb_features = _orb_extractor->DetectAndUndistortKeyPoints(image);
  // const velocity model
  const auto prev_frame = _frames.back();
  const auto points_3d = prev_frame.GetPoints3DView();
  const auto &prev_Tcw = prev_frame.GetTcw();
  const auto Tcw = _velocity * prev_Tcw;
  const auto camera_pose_in_world = Tcw.inverse();
  // todo: project 3d points (along with its feature descriptor) to current
  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.resize(points_3d.size());
  clean_slam::ReprojectPoints3d(points_3d, std::begin(points_reprojected), Tcw,
                                _camera_intrinsic);
  std::vector<bool> mask(points_reprojected.size(), false);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  //  const auto &distance_bounds = _reference_key_frame->GetDistanceBounds();

  for (size_t i = 0; i < points_reprojected.size(); ++i) {
    const auto &point = points_reprojected[i];

    //  1. check points in undistorted range
    if (!IsPointWithInBounds(point, x_bounds, y_bounds))
      continue;

    //  2. check new depth of points (wrt to new camera pose) is within
    //  scale pyramid range
    const auto depth =
        (points_3d[i] - camera_pose_in_world.translation()).norm();
//    if (!distance_bounds[i].IsWithIn(depth))
//      continue;
//
//    int predicted_octave_level =
//        _octave_scales.MapDistanceToOctaveLevel(depth, distance_bounds[i]);

    mask[i] = true;
  }
  //  SearchByProjection(current_frame.GetKeyPoints(), points_reprojected,
  //                     _reference_key_frame->GetDescriptors(), mask);

  if (_viewer) {
    _viewer->OnNotify(Content{Tcw, {}});
    _viewer->OnNotify(image, orb_features);
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
