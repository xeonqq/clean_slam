//
// Created by root on 8/16/20.
//
#include "key_frame.h"
#include "map_point.h"
#include "gtest/gtest.h"
#include <frame.h>
namespace clean_slam {

TEST(MapPoint, OnMapPointDelete) {
  Frame frame;
  KeyFrame key_frame{frame, 0};
  {
    MapPoint map_point;
    key_frame.AddMatchedMapPoint(&map_point, 0);
    EXPECT_EQ(map_point.NumOfObservers(), 1);
    EXPECT_EQ(key_frame.GetNumMatchedMapPoints(), 1);
  }
  EXPECT_EQ(key_frame.GetNumMatchedMapPoints(), 0);
}

TEST(MapPoint, OnKeyFrameDelete) {
  MapPoint map_point;
  Frame frame;
  {
    KeyFrame key_frame{frame, 0};
    key_frame.AddMatchedMapPoint(&map_point, 0);
    EXPECT_EQ(map_point.NumOfObservers(), 1);
    EXPECT_EQ(key_frame.GetNumMatchedMapPoints(), 1);
  }
  EXPECT_EQ(map_point.NumOfObservers(), 0);
}

TEST(MapPoint, OnUpdateEvent) {
  MapPoint map_point{{0, 0, 0}, {}, {}, {}};
  Eigen::Quaterniond r{1, 0, 0, 0};
  Eigen::Vector3d t1{1, 0, 0};
  Frame frame1{{}, {}, {r, t1}, {}};
  KeyFrame key_frame1{frame1, 0};
  Eigen::Vector3d t2{0, 1, 0};
  Frame frame2{{}, {}, {r, t2}, {}};
  KeyFrame key_frame2{frame2, 1};
  key_frame1.AddMatchedMapPoint(&map_point, 0);
  key_frame2.AddMatchedMapPoint(&map_point, 0);
  EXPECT_EQ(map_point.NumOfObservers(), 2);
  map_point.Update();
  Eigen::Vector3d expected_dir{0.7071067, 0.7071067, 0};
  //  std::cout << map_point.GetViewDirection() << std::endl;
  EXPECT_TRUE(map_point.GetViewDirection().isApprox(expected_dir, 1e-4));
}
} // namespace clean_slam
