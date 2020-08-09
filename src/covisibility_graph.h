//
// Created by root on 8/2/20.
//

#ifndef CLEAN_SLAM_SRC_COVISIBILITY_GRAPH_H_
#define CLEAN_SLAM_SRC_COVISIBILITY_GRAPH_H_
#include "frame.h"
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
namespace clean_slam {

using namespace boost;

class CovisibilityGraph {
public:
  using Node = std::pair<size_t, cv::Mat>;
  typedef property<edge_weight_t, int> EdgeProperty;
  typedef size_t VertexProperty;
  typedef adjacency_list<vecS, vecS, undirectedS, VertexProperty, EdgeProperty>
      Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
  typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

  CovisibilityGraph(const std::vector<Frame> *frames);
  void AddEdge(const Node &src_node, const Node &target_node, int weight);

  const Frame &GetReferenceKeyFrame() const;
  const cv::Mat &GetReferenceKeyFrameImage() const;
  ~CovisibilityGraph();

private:
  Graph _key_frame_graph;
  const std::vector<Frame> *_frames;
  vertex_t _reference_key_frame;
  cv::Mat _reference_key_frame_image;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_COVISIBILITY_GRAPH_H_
