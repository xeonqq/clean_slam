//
// Created by root on 4/26/20.
//

#ifndef CLEAN_SLAM_SRC_STATES_H_
#define CLEAN_SLAM_SRC_STATES_H_

#include <boost/msm/front/states.hpp>
#include <third_party/spdlog/spdlog.h>

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
        fsm._orb_extractor, fsm._optimizer, fsm._camera_intrinsic, &fsm._map,
        fsm._octave_scales, fsm._viewer);
    _map_initializer->ProcessFirstImage(event.image, event.timestamp);
  }

  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("Exit: Initialization...");
    _map_initializer.release();
  }

  template <class Event, class Fsm>
  void InitializeCameraPose(const Event &event, Fsm &fsm) {
    if (_map_initializer->InitializeCameraPose(event.image, event.timestamp)) {
      fsm._key_frames.push_back(_map_initializer->GetKeyFrameOwnership());
      fsm._reference_key_frame = &fsm._key_frames.back();
      fsm._previous_frame = &fsm._key_frames.back();
      fsm._velocity = _map_initializer->GetVelocity();
      const auto &stamped_transformations =
          _map_initializer->GetStampedTransformations();
      fsm._trajectory.push_back(stamped_transformations[0]);
      fsm._trajectory.push_back(stamped_transformations[1]);
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
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_STATES_H_
