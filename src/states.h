//
// Created by root on 4/26/20.
//

#ifndef CLEAN_SLAM_SRC_STATES_H_
#define CLEAN_SLAM_SRC_STATES_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/msm/front/states.hpp>
#include <third_party/spdlog/spdlog.h>

#include "cv_algorithms.h"
#include "map_initializer.h"

namespace clean_slam {

using state = boost::msm::front::state<>;

struct Initialized {};

struct NewImage {
  cv::Mat image;
  double timestamp;
};

struct Init : public state {
  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("Exit: FirstImageProcess..");
  }
};

struct MapInitialization : public state {
  template <class Event, class Fsm>
  void on_entry(Event const &event, Fsm &fsm) {
    spdlog::info("Entering: Initialization..");
    _map_initializer = std::make_unique<MapInitializer>(
        fsm._orb_extractor, &fsm._orb_feature_matcher, fsm._optimizer,
        fsm._camera_intrinsic, &fsm._map, fsm._octave_scales, &fsm._frames,
        fsm._viewer);
    _map_initializer->ProcessFirstImage(event.image, event.timestamp);
  }

  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("Exit: Initialization...");
    _map_initializer.release();
  }

  template <class Event, class Fsm>
  void InitializeCameraPose(const Event &event, Fsm &fsm) {
    auto initialized =
        _map_initializer->InitializeCameraPose(event.image, event.timestamp);
    if (initialized) {
      fsm.process_event(Initialized{});
    }
  }

private:
  std::unique_ptr<MapInitializer> _map_initializer;
};

struct MotionTrack : public state {
  template <class Event, class Fsm>
  void on_entry(Event const &event, Fsm &fsm) {
    spdlog::info("enter: TrackByMotion");
  }
  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("exit: TrackByMotion");
  }
};

struct KeyFrameInsertion : public state {
  template <class Event, class Fsm>
  void on_entry(Event const &event, Fsm &fsm) {
    spdlog::info("enter: KeyFrameInsertion");
  }
  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("exit: KeyFrameInsertion");
  }
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_STATES_H_
