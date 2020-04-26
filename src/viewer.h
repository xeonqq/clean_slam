//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWER_H_
#define CLEAN_SLAM_SRC_VIEWER_H_

#include "frame.h"
#include "homogeneous_matrix.h"
#include "settings.h"
#include <pangolin/pangolin.h>
#include <queue>

namespace clean_slam {

struct Content {
  HomogeneousMatrix homogeneous_matrix;
  std::vector<Eigen::Vector3d> triangulated_points;
};

class Viewer {

public:
  Viewer(const ViewerSettings &viewer_settings);
  void Run();

  void OnNotify(const Content &content) {
    std::lock_guard<std::mutex> lock(_mutex);
    _contents.push_back(content);
  }

  void OnNotify(const cv::Mat &image, const OrbFeatures &orb_features) {
    std::lock_guard<std::mutex> lock(_mutex);
    _image = image;
    _features = orb_features;
  }

private:
  std::mutex _mutex;

  ViewerSettings _viewer_settings_;
  std::vector<Content> _contents;
  cv::Mat _image;
  OrbFeatures _features;

  //  cv::namedWindow("ORB-SLAM2: Current Frame");
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_VIEWER_H_
