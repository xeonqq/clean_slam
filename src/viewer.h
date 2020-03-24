//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWER_H_
#define CLEAN_SLAM_SRC_VIEWER_H_

#include "homogeneous_matrix.h"
#include "viewer_settings.h"
#include <pangolin/pangolin.h>

namespace clean_slam {
template <typename Derived> class Observer {

public:
  template <typename... T> void Notify(T... args) {
    static_cast<Derived>(*this).Notify(std::forward<T>(args)...);
  };
};

class Viewer : public Observer<Viewer> {

public:
  Viewer(const ViewerSettings &viewer_settings);
  void Run();
  void Notify(const HomogeneousMatrix &camera_pose);

private:
  ViewerSettings viewer_settings_;

  //  cv::namedWindow("ORB-SLAM2: Current Frame");
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_VIEWER_H_
