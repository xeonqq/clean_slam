//
// Created by root on 4/13/20.
//

#ifndef CLEAN_SLAM_SRC_IOC_FACTORY_H_
#define CLEAN_SLAM_SRC_IOC_FACTORY_H_
#include "dataset_loader.h"
#include "slam_core.h"

#include <boost/msm/back/state_machine.hpp>
#include <thread>

namespace clean_slam {
class IocFactory {
public:
  IocFactory(const DatasetLoader *dataset_loader);
  std::unique_ptr<boost::msm::back::state_machine<SlamCore>> CreateSlamCore();
  std::thread CreateViewerThread();

private:
  const DatasetLoader *dataset_loader;
  OrbExtractor _orb_extractor;
  Viewer _viewer;
  OctaveScales _octave_scales;
  Optimizer _optimizer;
  OptimizerOnlyPose _optimizer_only_pose;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_IOC_FACTORY_H_
