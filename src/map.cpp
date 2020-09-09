//
// Created by root on 5/3/20.
//

#include "map.h"
#include "cv_algorithms.h"
#include "key_frame.h"
#include <boost/graph/graphviz.hpp>
#include <boost/range/algorithm/transform.hpp>

namespace clean_slam {
Map::Map(const OctaveScales &octave_scales) : _octave_scales{octave_scales} {}

vertex_t Map::AddKeyFrame(const g2o::SE3Quat &Tcw,
                          const std::vector<cv::KeyPoint> &key_points,
                          const cv::Mat &descriptors) {

  const auto vertex = add_vertex(_covisibility_graph);
  _covisibility_graph[vertex] = KeyFrame{Tcw, key_points, descriptors, vertex};
  return vertex;
}

vertex_t Map::AddKeyFrame(const g2o::SE3Quat &Tcw,
                          const OrbFeatures &orb_features) {
  const auto vertex = add_vertex(_covisibility_graph);
  _covisibility_graph[vertex] = KeyFrame{Tcw, orb_features, vertex};
  return vertex;
}

void Map::AddKeyFramesWeight(const vertex_t &v0, const vertex_t &v1,
                             int weight) {
  add_edge(v0, v1, weight, _covisibility_graph);
}

std::vector<MapPoint *>
Map::AddMapPoints(const std::vector<Eigen::Vector3d> &points_3d,
                  const std::vector<int> &matched_key_points_indexes,
                  const vertex_t &key_frame_vertex) {
  std::vector<MapPoint *> map_points;
  auto &key_frame = _covisibility_graph[key_frame_vertex];
  const auto &key_points = key_frame.GetKeyPoints();
  const auto &Tcw = key_frame.GetTcw();
  const auto &descriptors = key_frame.GetDescriptors();
  for (std::size_t i{0}; i < points_3d.size(); ++i) {
    const auto matched_key_point_index = matched_key_points_indexes[i];
    auto &map_point =
        AddMapPoint(Tcw, points_3d[i], descriptors.row(matched_key_point_index),
                    key_points[matched_key_point_index]);
    map_points.push_back(&map_point);
    key_frame.AddMatchedMapPoint(&map_point, matched_key_point_index);
  }
  return map_points;
}

std::vector<MapPoint *>
Map::AddMapPoints(const std::vector<Eigen::Vector3d> &points_3d,
                  const std::vector<int> &matched_key_points_indexes0,
                  const vertex_t &ref_key_frame_vertex0,
                  const std::vector<int> &matched_key_points_indexes1,
                  const vertex_t &key_frame_vertex1) {

  std::vector<MapPoint *> map_points;
  auto &key_frame0 = _covisibility_graph[ref_key_frame_vertex0];
  auto &key_frame1 = _covisibility_graph[key_frame_vertex1];

  const auto &key_points0 = key_frame0.GetKeyPoints();
  const auto &Tcw0 = key_frame0.GetTcw();
  const auto &descriptors0 = key_frame0.GetDescriptors();

  for (std::size_t i{0}; i < points_3d.size(); ++i) {
    const auto matched_key_point_index0 = matched_key_points_indexes0[i];
    const auto matched_key_point_index1 = matched_key_points_indexes1[i];
    auto &map_point = AddMapPoint(Tcw0, points_3d[i],
                                  descriptors0.row(matched_key_point_index0),
                                  key_points0[matched_key_point_index0]);
    map_points.push_back(&map_point);
    key_frame0.AddMatchedMapPoint(&map_point, matched_key_point_index0);
    key_frame1.AddMatchedMapPoint(&map_point, matched_key_point_index1);
  }
  return map_points;
}

MapPoint &Map::AddMapPoint(const g2o::SE3Quat &Tcw,
                           const Eigen::Vector3d &point_3d,
                           const cv::Mat &descriptor,
                           const cv::KeyPoint &key_point) {
  const auto bound =
      Calculate3DPointDistanceBound(Tcw, key_point, point_3d, _octave_scales);
  return const_cast<MapPoint &>(*_map_points
                                     .emplace(point_3d,
                                              ViewingDirection(Tcw, point_3d),
                                              descriptor, bound)
                                     .first);
}
const std::set<MapPoint> &Map::GetMapPoints() const { return _map_points; }

const KeyFrame &Map::GetKeyFrame(const vertex_t &kf_vertex) const {
  return _covisibility_graph[kf_vertex];
}

std::vector<vertex_t> Map::GetNeighbors(vertex_t vertex) const {
  graph_traits<Graph>::adjacency_iterator ai, a_end;
  tie(ai, a_end) = boost::adjacent_vertices(vertex, _covisibility_graph);
  return std::vector<vertex_t>(ai, a_end);
}

Map::~Map() {
  boost::dynamic_properties dp;
  dp.property("node_id", get(boost::vertex_index, _covisibility_graph));
  dp.property("label", get(boost::edge_weight, _covisibility_graph));
  std::ofstream f{"graph.dot"};
  write_graphviz_dp(f, _covisibility_graph, dp);
  std::cout
      << "\nTo visualize the graph run: dot -Tpng graph.dot > graph.png\n";
}
} // namespace clean_slam
