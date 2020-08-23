//
// Created by root on 8/16/20.
//
#include "key_frame.h"
#include "map_point.h"
#include "gtest/gtest.h"
namespace clean_slam {

TEST(Observerable, OnMapPointDelete) {
  KeyFrame key_frame;
  {
    MapPoint map_point;
    key_frame.AddMatchedMapPoint(&map_point);
    EXPECT_EQ(map_point.NumOfObservers(), 1);
    EXPECT_EQ(key_frame.NumberOfMatchedMapPoints(), 1);
  }
  EXPECT_EQ(key_frame.NumberOfMatchedMapPoints(), 0);
}

TEST(Observerable, OnKeyFrameDelete) {
  MapPoint map_point;
  {
    KeyFrame key_frame;
    key_frame.AddMatchedMapPoint(&map_point);
    EXPECT_EQ(map_point.NumOfObservers(), 1);
    EXPECT_EQ(key_frame.NumberOfMatchedMapPoints(), 1);
  }
  EXPECT_EQ(map_point.NumOfObservers(), 0);
}
} // namespace clean_slam
