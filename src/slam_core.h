//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "camera_motion_estimator.h"
#include "covisibility_graph.h"
#include "frame.h"
#include "map.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include "stamped_transformation.h"
#include "states.h"
#include "undistorted_image_boundary.h"
#include "viewer.h"

#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <opencv2/core/core.hpp>

namespace clean_slam {

using namespace boost;
using CameraTrajectory = std::vector<StampedTransformation>;

class SlamCore : public boost::msm ::front::state_machine_def<SlamCore> {
public:
  friend MapInitialization;
  friend MotionTrack;
  SlamCore(const cv::Mat &camera_intrinsics,
           const cv::Mat &camera_distortion_coeffs, OrbExtractor *orb_extractor,
           Optimizer *optimizer, OptimizerOnlyPose *optimizer_only_pose,
           Viewer *viewer, const OctaveScales &octave_scale);
  void ProcessFirstImage(const cv::Mat &image, double timestamp);
  void TrackByMotionModel(const cv::Mat &image, double timestamp);
  CameraTrajectory GetTrajectory() const;

  struct ProcessFirstImageAction {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &event, FSM &fsm, SourceState &, TargetState &) {
      fsm.ProcessFirstImage(event.image, event.timestamp);
    }
  };

  struct InitializeCameraPoseAction {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const &event, FSM &fsm, SourceState &source_state,
                    TargetState &) {
      source_state.InitializeCameraPose(event, fsm);
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
            boost::msm::front::Row<MotionTrack, NewImage,
                                   boost::msm::front::none,
                                   TrackByMotionModelAction>> {};
  using initial_state = Init;

  template <class FSM, class Event>
  void exception_caught(Event const &event, FSM &, std::exception &e) {
    std::cout << e.what() << std::endl;
    std::cout << typeid(Event).name() << std::endl;
    BOOST_ASSERT(false);
  }

private:
  cv::Mat _camera_intrinsic;
  OrbExtractor *_orb_extractor;
  Optimizer *_optimizer;
  OptimizerOnlyPose *_optimizer_only_pose;
  Viewer *_viewer;

  OrbFeatureMatcher _orb_feature_matcher;
  UndistortedImageBoundary _undistorted_image_boundary;

  const OctaveScales &_octave_scales;

  Map _map;

  std::vector<Frame> _frames;

  typedef property<edge_weight_t, int> EdgeProperty;
  typedef adjacency_list<vecS, vecS, undirectedS, no_property, EdgeProperty>
      Graph;
  CovisibilityGraph _key_frame_graph;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
