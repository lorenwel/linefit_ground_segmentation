#include "ground_segmentation/bin.h"

#include <limits>

Bin::Bin() : min_z(std::numeric_limits<double>::max()), has_point_(false) {}

Bin::Bin(const Bin& bin) : min_z(std::numeric_limits<double>::max()),
                                           has_point_(false) {}

void Bin::addPoint(const pcl::PointXYZ& point) {
  has_point_ = true;
  if (point.z < min_z) {
    min_z = point.z;
    min_z_range = sqrt(point.x * point.x + point.y * point.y);
  }
}

Bin::MinZPoint Bin::getMinZPoint() {
  MinZPoint point;

  if (has_point_) {
    point.z = min_z;
    point.d = min_z_range;
  }

  return point;
}
