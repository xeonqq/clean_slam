//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "camera_motion_estimator.h"
#include "frame.h"
#include "key_frame.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include "stamped_transformation.h"
#include "undistorted_image_boundary.h"
#include "viewer.h"

#include <Eigen/Core>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <opencv2/core/core.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {
using state = boost::msm::front::state<>;

using CameraTrajectory = std::vector<StampedTransformation>;

struct NewImage {
  cv::Mat image;
  double timestamp;
};

struct Initialized {};

struct Init : public state {
  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("exit: FirstImageProcess");
  }
};

struct MapInitialization : public state {
  template <class Event, class Fsm>
  void on_entry(Event const &event, Fsm &fsm) {
    spdlog::info("entering: Initialization");
  }
};

struct MotionTrack : public state {
  template <class Event, class Fsm> void on_exit(Event const &event, Fsm &fsm) {
    spdlog::info("enter: TrackByMotion");
  }
};
class SlamCore : public boost::msm ::front::state_machine_def<SlamCore> {
public:
  SlamCore(const cv::Mat &camera_intrinsics,
           const cv::Mat &camera_distortion_coeffs, OrbExtractor *orb_extractor,
           Optimizer *optimizer, Viewer *viewer,
           const OctaveScales &octave_scale);
  void ProcessFirstImage(const cv::Mat &image, double timestamp);
  bool InitializeCameraPose(const cv::Mat &image, double timestamp);
  void TrackByMotionModel(const cv::Mat &image, double timestamp);
  const CameraTrajectory &GetTrajectory() const;

  struct ProcessFirstImageAction {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &event, FSM &fsm, SourceState &, TargetState &) {
      fsm.ProcessFirstImage(event.image, event.timestamp);
    }
  };

  struct InitializeCameraPoseAction {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &event, FSM &fsm, SourceState &src,
                    TargetState &) {
      bool initialized = fsm.InitializeCameraPose(event.image, event.timestamp);
      if (initialized)
        fsm.process_event(Initialized{});
    }
  };

  struct TrackByMotionModelAction {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &event, FSM &fsm, SourceState &, TargetState &) {
      fsm.TrackByMotionModel(event.image, event.timestamp);
    }
  };

  struct transition_table
      : boost::mpl::vector<
            boost::msm::front::Row<Init, NewImage, MapInitialization,
                                   ProcessFirstImageAction>,
            boost::msm::front::Row<
                MapInitialization, NewImage, boost::msm::front::none,
                InitializeCameraPoseAction>, // internal transition
            boost::msm::front::Row<MapInitialization, Initialized, MotionTrack>,
            boost::msm::front::Row<MotionTrack, NewImage, MotionTrack,
                                   TrackByMotionModelAction>> {};
  using initial_state = Init;

private:
  cv::Mat _camera_intrinsic;
  OrbExtractor *_orb_extractor;
  Optimizer *_optimizer;
  Viewer *_viewer;

  CameraMotionEstimator _camera_motion_estimator;
  UndistortedImageBoundary _undistorted_image_boundary;

  const OctaveScales &_octave_scales;

  Frame _previous_frame;
  OrbFeatureMatcher _orb_feature_matcher;
  CameraTrajectory _trajectory;
  g2o::SE3Quat _velocity;

  std::vector<KeyFrame> _key_frames;
  const KeyFrame *_reference_key_frame;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
