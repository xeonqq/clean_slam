//
// Created by root on 4/10/20.
//

#include "slam_state_machine.h"
#include "third_party/tinyfsm.hpp"

FSM_INITIAL_STATE(clean_slam::SLAMStateMachine, clean_slam::ProcessFirstImage)
namespace clean_slam {

void ProcessFirstImage::react(const cv::Mat &image, double timestamp,
                              SlamCore &core) {
  core.ProcessFirstImage(image, timestamp);
  transit<Initialization>();
}

void Initialization::react(const cv::Mat &image, double timestamp,
                           SlamCore &core) {
  const bool initialized = core.InitializeCameraPose(image, timestamp);
  if (initialized)
    transit<TrackByMotion>();
}

void TrackByMotion::react(const cv::Mat &image, double timestamp,
                          SlamCore &core) {
  core.TrackByMotionModel(image, timestamp);
  //    transit<Initialization>();
}

} // namespace clean_slam
