#include "ground_segmentation/bin.h"

#include <limits>

Bin::Bin() : min_z(std::numeric_limits<double>::max()), has_point_(false)
{
}

Bin::Bin(const Bin& bin) : min_z(std::numeric_limits<double>::max()), has_point_(false)
{
}

void Bin::addPoint(const velodyne_pointcloud::PointXYZIR& point)
{
  const double d = sqrt(point.x * point.x + point.y * point.y);
  addPoint(d, point.z, point.intensity, point.ring);
}

void Bin::addPoint(const float& d, const float& z, const float& intensity, const uint16_t& ring)
{
  has_point_ = true;
  if (z < min_z) {
    min_z = z;
    min_z_range = d;
    min_z_intensity = intensity;
    min_z_ring = ring;
  }
}

Bin::MinZPoint Bin::getMinZPoint()
{
  MinZPoint point;

  if (has_point_) {
    point.z = min_z;
    point.d = min_z_range;
    point.intensity = min_z_intensity;
    point.ring = min_z_ring;
  }

  return point;
}
