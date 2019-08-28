#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_

#include <atomic>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>

class Bin
{
 public:
  struct MinZPoint
  {
    MinZPoint() : z(0), d(0)
    {
    }
    MinZPoint(const float& d, const float& z, const float& intensity, const uint16_t& ring)
      : z(z), d(d), intensity(intensity), ring(ring)
    {
    }
    bool operator==(const MinZPoint& comp)
    {
      return z == comp.z && d == comp.d;
    }

    float z;
    float d;

    float intensity;
    uint16_t ring;
  };

 private:
  std::atomic<bool> has_point_;
  std::atomic<float> min_z;
  std::atomic<float> min_z_range;
  std::atomic<float> min_z_intensity;
  std::atomic<uint16_t> min_z_ring;

 public:
  Bin();

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin& segment);

  void addPoint(const velodyne_pointcloud::PointXYZIR& point);

  void addPoint(const float& d, const float& z, const float& intensity, const uint16_t& ring);

  MinZPoint getMinZPoint();

  inline bool hasPoint()
  {
    return has_point_;
  }
};

#endif /* GROUND_SEGMENTATION_BIN_H_ */
