//
// Created by root on 5/3/20.
//

#ifndef CLEAN_SLAM_SRC_MAP_H_
#define CLEAN_SLAM_SRC_MAP_H_

#include <opencv2/core/mat.hpp>
#include <set>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

#include "bound.h"
#include "frame.h"
#include "graph.h"
#include "map_point.h"
#include "octave_scales.h"
#include "orb_extractor.h"
#include <mutex>

namespace clean_slam {
using namespace boost;

class Map {

public:
  Map() = default;
  Map(const OctaveScales &, const OrbFeatureMatcher &,
      const cv::Mat &camera_instrinsics);

  std::vector<MapPoint *>
  AddMapPoints(const std::vector<Eigen::Vector3d> &points_3d,
               const std::vector<int> &matched_key_points_indexes,
               const vertex_t &key_frame_vertex);

  std::vector<MapPoint *>
  AddMapPoints(const std::vector<Eigen::Vector3d> &points_3d,
               const std::vector<int> &matched_key_points_indexes0,
               const vertex_t &ref_key_frame_vertex0,
               const std::vector<int> &matched_key_points_indexes1,
               const vertex_t &key_frame_vertex1,
               boost::optional<edge_t> edge = {});

  MapPoint &AddMapPoint(const g2o::SE3Quat &Tcw,
                        const Eigen::Vector3d &point_3d,
                        const cv::Mat &descriptor,
                        const cv::KeyPoint &key_point);

  vertex_t AddKeyFrame(Frame &frame);

  const std::set<MapPoint> &GetMapPoints() const;

  const KeyFrame &GetKeyFrame(vertex_t kf_vertex) const;

  std::vector<vertex_t> GetNeighbors(vertex_t vertex) const;

  void LocalMapping(vertex_t vertex);

  const Graph &GetCovisibilityGraph() const;

  std::lock_guard<std::mutex> Lock() const {
    return std::lock_guard<std::mutex>(_mutex);
  }
  ~Map();

private:
  KeyFrame &GetKeyFrame(vertex_t kf_vertex);
  void AddKeyFramesWeight(const vertex_t &v0, const vertex_t &v1, int weight,
                          boost::optional<edge_t> edge);
  void ConnectKeyFrame(vertex_t vertex);
  std::vector<std::tuple<vertex_t, edge_t>>
  GetHighCovisibleKeyFrames(vertex_t vertex, size_t max_num_kfs) const;

private:
  const OctaveScales &_octave_scales;
  const OrbFeatureMatcher &_matcher;
  cv::Mat _camera_intrinsics;
  // todo: could be replace by a vector, map point id is the same as the index
  // of the vector
  std::set<MapPoint> _map_points;
  Graph _covisibility_graph;

  // for viewer, during visualization, graph can be updated
  mutable std::mutex _mutex;

}; // namespace clean_slam
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_MAP_H_
