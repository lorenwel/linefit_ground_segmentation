#ifndef GROUND_SEGMENTATION_H_
#define GROUND_SEGMENTATION_H_

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ground_segmentation/segment.h"

struct GroundSegmentationParams {
  GroundSegmentationParams() :
      r_min_square(0.3 * 0.3),
      r_max_square(20*20),
      n_bins(30),
      n_segments(180),
      max_slope(1),
      n_threads(4) {}

  // Minimum range of segmentation.
  double r_min_square;
  // Maximum range of segmentation.
  double r_max_square;
  // Number of radial bins.
  int n_bins;
  // Number of angular segments.
  int n_segments;
  // Max slope to be considered ground line.
  double max_slope;
  // Number of threads.
  int n_threads;
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;

class GroundSegmentation {

  const GroundSegmentationParams params_;

  // Access with segments_[segment][bin].
  std::vector<Segment> segments_;

  void insertPoints(const PointCloud& cloud);

  void insertionThread(const PointCloud& cloud,
                       const size_t start_index,
                       const size_t end_index);

  void getMinZPoints(PointCloud* out_cloud);

  void getLines(std::list<PointLine>* lines);

  pcl::PointXYZ minZPointTo3d(const Bin::MinZPoint& min_z_point, const double& angle);

public:

  GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams());

  void segment(const PointCloud& cloud, std::vector<int>* segmentation);

};

#endif // GROUND_SEGMENTATION_H_
