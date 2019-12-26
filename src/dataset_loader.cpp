//
// Created by root on 12/26/19.
//

#include "dataset_loader.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace clean_slam {

bool IsCommentLine(const std::string &s) { return s.find('#') == 0; }

void DatasetLoader::LoadFreiburgRgb(const std::string &dataset_folder_name) {
  std::string image_inventory_filename = dataset_folder_name + "/rgb.txt";
  std::ifstream f;
  f.open(image_inventory_filename.c_str());
  while (!f.eof()) {
    std::string s;
    getline(f, s);
    if (!s.empty() && !IsCommentLine(s)) {
      std::stringstream ss;
      ss << s;
      double t;
      std::string sRGB;
      ss >> t;
      ss >> sRGB;
      _image_files.emplace_back(sRGB, t);
    }
  }
}

const std::vector<ImageFile> &DatasetLoader::GetImageFiles() const {
  return _image_files;
}

} // namespace clean_slam
