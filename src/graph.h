//
// Created by root on 8/31/20.
//

#ifndef CLEAN_SLAM_SRC_GRAPH_H_
#define CLEAN_SLAM_SRC_GRAPH_H_

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <memory>

namespace clean_slam {

class KeyFrame;

typedef boost::property<boost::edge_weight_t, int> EdgeProperty;
typedef std::unique_ptr<KeyFrame> VertexProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              VertexProperty, EdgeProperty>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_GRAPH_H_
