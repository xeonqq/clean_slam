//
// Created by root on 4/13/20.
//

#ifndef CLEAN_SLAM_SRC_IOC_FACTORY_H_
#define CLEAN_SLAM_SRC_IOC_FACTORY_H_
#include "dataset_loader.h"
#include "slam_core.h"
#include <thread>

namespace clean_slam {
class IocFactory {
public:
  IocFactory(const DatasetLoader *dataset_loader);
  std::unique_ptr<SlamCore> CreateSlamCore();
  std::thread CreateViewerThread();

private:
  const DatasetLoader *dataset_loader;
  OrbExtractor _orb_extractor;
  Viewer _viewer;
  OctaveSigmaScales _octave_sigma_scales;
  Optimizer _optimizer;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_IOC_FACTORY_H_
