//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWER_H_
#define CLEAN_SLAM_SRC_VIEWER_H_

#include "homogeneous_matrix.h"
#include "viewer_settings.h"
#include <pangolin/pangolin.h>
#include <queue>

namespace clean_slam {

struct Content {
  HomogeneousMatrix homogeneous_matrix;
  cv::Mat triangulated_points;
};

class Viewer {

public:
  Viewer(const ViewerSettings &viewer_settings);
  void Run();

  template <typename T> void OnNotify(T &&content) {
    std::lock_guard<std::mutex> lock(_mutex);
    _contents.push_back(std::forward<T>(content));
  }

private:
  ViewerSettings _viewer_settings_;
  std::vector<Content> _contents;
  std::mutex _mutex;

  //  cv::namedWindow("ORB-SLAM2: Current Frame");
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_VIEWER_H_
