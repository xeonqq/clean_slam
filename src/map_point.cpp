//
// Created by root on 8/23/20.
//
#include "map_point.h"
namespace clean_slam {

size_t MapPoint::NumOfObservers() const { return _events.num_slots(); }

MapPoint::~MapPoint() { _events(this); }

} // namespace clean_slam
