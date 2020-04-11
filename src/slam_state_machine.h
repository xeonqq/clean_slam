//
// Created by root on 4/10/20.
//

#ifndef CLEAN_SLAM_SRC_SLAM_STATE_MACHINE_H_
#define CLEAN_SLAM_SRC_SLAM_STATE_MACHINE_H_

#include "slam_core.h"
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <third_party/spdlog/spdlog.h>
#include <tinyfsm.hpp>
namespace clean_slam {

struct TrackByMotion; // forward declaration
struct InitializeCameraPose;
// ----------------------------------------------------------------------------
// 1. Event Declarations
//

// ----------------------------------------------------------------------------
// 2. State Machine Base Class Declaration
//
struct SLAMStateMachine : tinyfsm::Fsm<SLAMStateMachine> {
  //  virtual void react(Toggle const &) { };

  // alternative: enforce handling of Toggle in all states (pure virtual)
  // virtual void react(Toggle const &) = 0;
  //  virtual void
  //  react(const ImageStamped &image_stamped) = 0; //{
  //  transit<TrackByMotion>(); };

  virtual void react(const cv::Mat &image, double timestamp, SlamCore &core){};

  virtual void entry(void){}; /* entry actions in some states */
  void exit(void){};          /* no exit actions */

  // alternative: enforce entry actions in all states (pure virtual)
  // virtual void entry(void) = 0;
};

// ----------------------------------------------------------------------------
// 3. State Declarations
//

struct Initialization : SLAMStateMachine {
  void entry() override { spdlog::info("State: Initialization"); }

  void react(const cv::Mat &image, double timestamp, SlamCore &core) {
    const bool initialized = core.InitializeCameraPose(image, timestamp);
    if (initialized)
      transit<TrackByMotion>();
  };
};

struct TrackByMotion : SLAMStateMachine {
  void entry() override { spdlog::info("State: TrackByMotion"); };
  void react(const cv::Mat &image, double timestamp, SlamCore &core){
      //    transit<Initialization>();
  };
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_STATE_MACHINE_H_
