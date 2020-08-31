//
// Created by root on 8/31/20.
//

#ifndef CLEAN_SLAM_SRC_GRAPH_H_
#define CLEAN_SLAM_SRC_GRAPH_H_

#include "key_frame.h"
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace clean_slam {


typedef boost::property<boost::edge_weight_t, int> EdgeProperty;
typedef KeyFrame VertexProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              VertexProperty, EdgeProperty>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_GRAPH_H_
