//
// Created by root on 12/25/19.
//

#include "dataset_loader.h"
#include "gtest/gtest.h"
namespace clean_slam {

TEST(DatasetLoaderTest, load_dataset) {
  // arrange
  DatasetLoader loader;
  // act
  loader.LoadFreiburgRgb(DATASET_DIR +
                         std::string("/rgbd_dataset_freiburg1_xyz"));

  // assert
  EXPECT_EQ(loader.GetImageFiles().size(), 798);
}
} // namespace clean_slam
