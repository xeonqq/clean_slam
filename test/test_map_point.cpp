//
// Created by root on 8/16/20.
//
#include "key_frame.h"
#include "map_point.h"
#include "gtest/gtest.h"
namespace clean_slam {

TEST(MapPoint, OnMapPointDelete) {
  KeyFrame key_frame;
  {
    MapPoint map_point;
    key_frame.AddMatchedMapPoint(&map_point);
    EXPECT_EQ(map_point.NumOfObservers(), 1);
    EXPECT_EQ(key_frame.NumberOfMatchedMapPoints(), 1);
  }
  EXPECT_EQ(key_frame.NumberOfMatchedMapPoints(), 0);
}

TEST(MapPoint, OnKeyFrameDelete) {
  MapPoint map_point;
  {
    KeyFrame key_frame;
    key_frame.AddMatchedMapPoint(&map_point);
    EXPECT_EQ(map_point.NumOfObservers(), 1);
    EXPECT_EQ(key_frame.NumberOfMatchedMapPoints(), 1);
  }
  EXPECT_EQ(map_point.NumOfObservers(), 0);
}

TEST(MapPoint, OnUpdateEvent) {
  MapPoint map_point{{0, 0, 0}, {}, {}, {}};
  Eigen::Quaterniond r{1, 0, 0, 0};
  Eigen::Vector3d t1{1, 0, 0};
  KeyFrame key_frame1{{r, t1}, {}, {}, {}};
  Eigen::Vector3d t2{0, 1, 0};
  KeyFrame key_frame2{{r, t2}, {}, {}, {}};
  key_frame1.AddMatchedMapPoint(&map_point);
  key_frame2.AddMatchedMapPoint(&map_point);
  EXPECT_EQ(map_point.NumOfObservers(), 2);
  map_point.Update();
  Eigen::Vector3d expected_dir{0.7071067, 0.7071067, 0};
  std::cout << map_point.GetViewDirection() << std::endl;
  EXPECT_TRUE(map_point.GetViewDirection().isApprox(expected_dir, 1e-4));
}
} // namespace clean_slam
