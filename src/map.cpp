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

vertex_t Map::AddKeyFrame(Frame &frame) {
  const auto vertex = add_vertex(_covisibility_graph);
  _covisibility_graph[vertex] = std::make_unique<KeyFrame>(frame, vertex);
  ConnectKeyFrame(vertex);
  LocalMapping(vertex);
  return vertex;
}

void Map::ConnectKeyFrame(vertex_t vertex) {
  auto &key_frame = *_covisibility_graph[vertex];
  std::map<vertex_t, size_t> kf_to_num_common_mp;
  for (auto *map_point : key_frame.GetMatchedMapPointsRng()) {
    for (auto *kf_observer : map_point->Observers()) {
      const auto kf_observer_vertex = kf_observer->GetVertex();
      if (kf_observer_vertex != vertex) {
        ++kf_to_num_common_mp[kf_observer_vertex];
      }
    }
  }
  for (auto [kf, num_common_mp] : kf_to_num_common_mp) {
    AddKeyFramesWeight(vertex, kf, num_common_mp);
  }
}

void Map::LocalMapping(vertex_t vertex) {
  // generate new map points
  auto high_covisible_neighbor_kfs = GetHighCovisibleKeyFrames(vertex, 10);
  auto &key_frame = GetKeyFrame(vertex);
  for (auto covisible_kf : high_covisible_neighbor_kfs) {
    auto covisible_key_frame = GetKeyFrame(covisible_kf);
    // match unmatched key point descriptors

    // check feasibility by epipolar constraint

    // triangulate new map points
  }
}

std::vector<vertex_t> Map::GetHighCovisibleKeyFrames(vertex_t vertex,
                                                     size_t max_num_kfs) const {
  std::vector<std::pair<vertex_t, EdgeProperty::value_type>>
      covisible_neighbor_kfs;
  std::vector<vertex_t> high_covisible_neighbor_kfs;
  const auto weight_map = get(edge_weight, _covisibility_graph);
  graph_traits<Graph>::out_edge_iterator ei, e_end;
  for (tie(ei, e_end) = out_edges(vertex, _covisibility_graph); ei != e_end;
       ++ei) {
    const auto num_common_map_points = get(weight_map, *ei);
    auto neighbor_kf = target(*ei, _covisibility_graph);
    covisible_neighbor_kfs.emplace_back(num_common_map_points, neighbor_kf);
  }
  auto range = std::min(max_num_kfs, covisible_neighbor_kfs.size());
  std::partial_sort(
      covisible_neighbor_kfs.begin(), covisible_neighbor_kfs.begin() + range,
      covisible_neighbor_kfs.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.second > rhs.second; });
  std::transform(covisible_neighbor_kfs.begin(),
                 covisible_neighbor_kfs.begin() + range,
                 std::back_inserter(high_covisible_neighbor_kfs),
                 [](const auto &pair) { return pair.first; });
  return high_covisible_neighbor_kfs;
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
  auto &key_frame = *_covisibility_graph[key_frame_vertex];
  const auto &key_points = key_frame.GetUndistortedKeyPoints();
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

  AddKeyFramesWeight(ref_key_frame_vertex0, key_frame_vertex1,
                     points_3d.size());
  std::vector<MapPoint *> map_points;
  auto &key_frame0 = *_covisibility_graph[ref_key_frame_vertex0];
  auto &key_frame1 = *_covisibility_graph[key_frame_vertex1];

  const auto &key_points0 = key_frame0.GetUndistortedKeyPoints();
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
  return *_covisibility_graph[kf_vertex];
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
