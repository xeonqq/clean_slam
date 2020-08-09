//
// Created by root on 8/2/20.
//

#include "covisibility_graph.h"
#include <boost/graph/graphviz.hpp>

namespace clean_slam {

CovisibilityGraph::CovisibilityGraph(const std::vector<Frame> *frames)
    : _frames(frames) {}

void CovisibilityGraph::AddEdge(const CovisibilityGraph::Node &src_node,
                                const CovisibilityGraph::Node &target_node,
                                int weight) {
  const auto src_vertex = add_vertex(_key_frame_graph);
  const auto target_vertex = add_vertex(_key_frame_graph);

  add_edge(src_vertex, target_vertex, weight, _key_frame_graph);

  _key_frame_graph[src_vertex] = src_node.first;
  _key_frame_graph[target_vertex] = target_node.first;

  _reference_key_frame = target_vertex;
  _reference_key_frame_image = target_node.second;
}

const Frame &CovisibilityGraph::GetReferenceKeyFrame() const {
  return (*_frames)[_key_frame_graph[_reference_key_frame]];
}

const cv::Mat &CovisibilityGraph::GetReferenceKeyFrameImage() const {
  return _reference_key_frame_image;
}

CovisibilityGraph::~CovisibilityGraph() {
  write_graphviz(std::cout, _key_frame_graph);
}

} // namespace clean_slam
