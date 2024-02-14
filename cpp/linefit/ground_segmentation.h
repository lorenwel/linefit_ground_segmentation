#pragma once
#include <mutex>

#include "segment.h"
#include "mics.h"

struct GroundSegmentationParams {
  GroundSegmentationParams() :
      r_min_square(0.3 * 0.3),
      r_max_square(20*20),
      n_bins(30),
      n_segments(180),
      max_dist_to_line(0.15),
      min_slope(0),
      max_slope(1),
      n_threads(4),
      max_error_square(0.01),
      long_threshold(2.0),
      max_long_height(0.1),
      max_start_height(0.2),
      sensor_height(0.2),
      line_search_angle(0.2) {}

  // Minimum range of segmentation.
  double r_min_square;
  // Maximum range of segmentation.
  double r_max_square;
  // Number of radial bins.
  int n_bins;
  // Number of angular segments.
  int n_segments;
  // Maximum distance to a ground line to be classified as ground.
  double max_dist_to_line;
  // Min slope to be considered ground line.
  double min_slope;
  // Max slope to be considered ground line.
  double max_slope;
  // Max error for line fit.
  double max_error_square;
  // Distance at which points are considered far from each other.
  double long_threshold;
  // Maximum slope for
  double max_long_height;
  // Maximum heigh of starting line to be labelled ground.
  double max_start_height;
  // Height of sensor above ground.
  double sensor_height;
  // How far to search for a line in angular direction [rad].
  double line_search_angle;
  // Number of threads.
  int n_threads;
};

typedef std::vector<Eigen::Vector3d> PointCloud;

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> PointLine;

class GroundSegmentation {

  const GroundSegmentationParams params_;

  // Access with segments_[segment][bin].
  std::vector<Segment> segments_;

  // Bin index of every point.
  std::vector<std::pair<int, int> > bin_index_;

  // 2D coordinates (d, z) of every point in its respective segment.
  std::vector<Bin::MinZPoint> segment_coordinates_;

  void assignCluster(std::vector<bool>* segmentation);

  void assignClusterThread(const unsigned int& start_index,
                           const unsigned int& end_index,
                           std::vector<bool>* segmentation);

  void insertPoints(const PointCloud& cloud);

  void insertionThread(const PointCloud& cloud,
                       const size_t start_index,
                       const size_t end_index);

  void getMinZPoints(PointCloud* out_cloud);

  void lineFitThread(const unsigned int start_index, const unsigned int end_index,
                     std::list<PointLine> *lines, std::mutex* lines_mutex);

  Eigen::Vector3d minZPointTo3d(const Bin::MinZPoint& min_z_point, const double& angle);

  void getMinZPointCloud(PointCloud* cloud);

  void resetSegments();

public:

  GroundSegmentation(){};
  // GroundSegmentation(const GroundSegmentationParams& params = GroundSegmentationParams());
  // virtual ~GroundSegmentation() = default;

  std::vector<bool> segment(const std::vector<std::vector<float>> points);

};
