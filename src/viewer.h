//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWER_H_
#define CLEAN_SLAM_SRC_VIEWER_H_

#include "homogeneous_matrix.h"
#include "viewer_settings.h"
#include <pangolin/pangolin.h>

namespace clean_slam {

class Viewer {

public:
  Viewer(const ViewerSettings &viewer_settings);
  void Run();
  template <typename T> void OnNotify(const T &data);

private:
  ViewerSettings viewer_settings_;

  //  cv::namedWindow("ORB-SLAM2: Current Frame");
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_VIEWER_H_
