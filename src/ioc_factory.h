//
// Created by root on 4/13/20.
//

#ifndef CLEAN_SLAM_SRC_IOC_FACTORY_H_
#define CLEAN_SLAM_SRC_IOC_FACTORY_H_
#include "dataset_loader.h"
#include "map.h"
#include "octave_scales.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "slam_core.h"
#include "viewer.h"

#include <boost/msm/back/state_machine.hpp>
#include <thread>

namespace clean_slam {
class IocFactory {
public:
  IocFactory(const DatasetLoader *dataset_loader, bool use_gui);
  std::unique_ptr<boost::msm::back::state_machine<SlamCore>> CreateSlamCore();
  boost::optional<std::thread> CreateViewerThread();

private:
  const bool _use_gui;
  const DatasetLoader *_dataset_loader;
  OrbExtractor _orb_extractor;
  std::unique_ptr<IViewer> _viewer;
  OctaveScales _octave_scales;
  Optimizer _optimizer;
  OptimizerOnlyPose _optimizer_only_pose;
  OrbFeatureMatcher _orb_feature_matcher;
  Map _map;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_IOC_FACTORY_H_
