//
// Created by root on 5/12/20.
//
#include "frame.h"
namespace clean_slam {

Points3DView::iterator Points3DView::begin() const {
  return Points3DView::iterator{_points_3d, _indexes, 0};
}

Points3DView::iterator Points3DView::end() const {
  return Points3DView::iterator{_points_3d, _indexes, _indexes.size()};
}

Points3DView Frame::GetPoints3DView() const {
  return Points3DView(_map->GetPoints3D(), _map_point_indexes);
}
} // namespace clean_slam
