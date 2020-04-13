//
// Created by root on 3/24/20.
//

#ifndef CLEAN_SLAM_SRC_SETTINGS_H_
#define CLEAN_SLAM_SRC_SETTINGS_H_
namespace clean_slam {
struct ViewerSettings {
  float view_point_x;
  float view_point_y;
  float view_point_z;
  float view_point_f;
  float display_interval_ms;
};

struct OrbExtractorSettings {
  int num_features;
  float scale_factor;
  int num_levels;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SETTINGS_H_
