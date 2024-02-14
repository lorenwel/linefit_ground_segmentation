
// #include <toml.hpp>
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
#include <iostream>


#include "ground_segmentation.h"


using namespace std::chrono_literals;


// GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
//     params_(params),
//     segments_(params.n_segments, Segment(params.n_bins,
//                                          params.min_slope,
//                                          params.max_slope,
//                                          params.max_error_square,
//                                          params.long_threshold,
//                                          params.max_long_height,
//                                          params.max_start_height,
//                                          params.sensor_height)) {
  
// }

std::vector<bool> GroundSegmentation::segment(const std::vector<std::vector<float>> points) {
  // TODO: Maybe there is a better way to convert the points to Eigen::Vector3d
  PointCloud cloud;
  for (auto point : points) {
    cloud.push_back(Eigen::Vector3d(point[0], point[1], point[2]));
  }
  std::cout << "Segmenting cloud with " << cloud.size() << " points...\n";

  std::vector<bool> labels(cloud.size(), false);
  bin_index_.resize(cloud.size());
  segment_coordinates_.resize(cloud.size());
  resetSegments();
  insertPoints(cloud);
  assignCluster(&labels);
  std::cout << "Segmentation done.\n";
  return labels;
}

void GroundSegmentation::lineFitThread(const unsigned int start_index,
                                       const unsigned int end_index,
                                       std::list<PointLine> *lines, std::mutex* lines_mutex) {
  const bool visualize = lines;
  const double seg_step = 2*M_PI / params_.n_segments;
  double angle = -M_PI + seg_step/2 + seg_step * start_index;
  for (unsigned int i = start_index; i < end_index; ++i) {
    segments_[i].fitSegmentLines();
    // Convert lines to 3d if we want to.
    if (visualize) {
      std::list<Segment::Line> segment_lines;
      segments_[i].getLines(&segment_lines);
      for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter) {
        const Eigen::Vector3d start = minZPointTo3d(line_iter->first, angle);
        const Eigen::Vector3d end = minZPointTo3d(line_iter->second, angle);
        lines_mutex->lock();
        lines->emplace_back(start, end);
        lines_mutex->unlock();
      }

      angle += seg_step;
    }
  }
}

void GroundSegmentation::getMinZPointCloud(PointCloud* cloud) {
  cloud->reserve(params_.n_segments * params_.n_bins);
  const double seg_step = 2*M_PI / params_.n_segments;
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      const Eigen::Vector3d min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
      cloud->push_back(min);
    }

    angle += seg_step;
  }
}

void GroundSegmentation::resetSegments() {
  segments_ = std::vector<Segment>(params_.n_segments, Segment(params_.n_bins,
                                                               params_.min_slope,
                                                               params_.max_slope,
                                                               params_.max_error_square,
                                                               params_.long_threshold,
                                                               params_.max_long_height,
                                                               params_.max_start_height,
                                                               params_.sensor_height));
}

Eigen::Vector3d GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point,
                                                const double &angle) {
  Eigen::Vector3d point;
  point[0] = cos(angle) * min_z_point.d;
  point[1] = sin(angle) * min_z_point.d;
  point[2] = min_z_point.z;
  return point;
}

void GroundSegmentation::assignCluster(std::vector<bool>* segmentation) {
  std::vector<std::thread> thread_vec(params_.n_threads);
  const size_t cloud_size = segmentation->size();
  for (unsigned int i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = cloud_size / params_.n_threads * i;
    const unsigned int end_index = cloud_size / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::assignClusterThread, this,
                                start_index, end_index, segmentation);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }
}

void GroundSegmentation::assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<bool> *segmentation) {
  const double segment_step = 2*M_PI/params_.n_segments;
  for (unsigned int i = start_index; i < end_index; ++i) {
    Bin::MinZPoint point_2d = segment_coordinates_[i];
    const int segment_index = bin_index_[i].first;
    if (segment_index >= 0) {
      double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      // Search neighboring segments.
      int steps = 1;
      while (dist < 0 && steps * segment_step < params_.line_search_angle) {
        // Fix indices that are out of bounds.
        int index_1 = segment_index + steps;
        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
        int index_2 = segment_index - steps;
        while (index_2 < 0) index_2 += params_.n_segments;
        // Get distance to neighboring lines.
        const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);
        if (dist_1 >= 0) {
          dist = dist_1;
        }
        if (dist_2 >= 0) {
          // Select smaller distance if both segments return a valid distance.
          if (dist < 0 || dist_2 < dist) {
            dist = dist_2;
          }
        }
        ++steps;
      }
      if (dist < params_.max_dist_to_line && dist != -1) {
        segmentation->at(i) = true;
      }
    }
  }
}

void GroundSegmentation::getMinZPoints(PointCloud* out_cloud) {
  const double seg_step = 2*M_PI / params_.n_segments;
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;
  const double r_min = sqrt(params_.r_min_square);
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    double dist = r_min + bin_step/2;
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      Eigen::Vector3d point;
      if (bin_iter->hasPoint()) {
        Bin::MinZPoint min_z_point(bin_iter->getMinZPoint());
        point.x() = cos(angle) * min_z_point.d;
        point.y() = sin(angle) * min_z_point.d;
        point.z() = min_z_point.z;

        out_cloud->push_back(point);
      }
      dist += bin_step;
    }
    angle += seg_step;
  }
}

void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  std::vector<std::thread> threads(params_.n_threads);
  const size_t points_per_thread = cloud.size() / params_.n_threads;
  // Launch threads.
  for (unsigned int i = 0; i < params_.n_threads - 1; ++i) {
    const size_t start_index = i * points_per_thread;
    const size_t end_index = (i+1) * points_per_thread;
    threads[i] = std::thread(&GroundSegmentation::insertionThread, this,
                             cloud, start_index, end_index);
  }
  // Launch last thread which might have more points than others.
  const size_t start_index = (params_.n_threads - 1) * points_per_thread;
  const size_t end_index = cloud.size();
  threads[params_.n_threads - 1] =
      std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);
  // Wait for threads to finish.
  for (auto it = threads.begin(); it != threads.end(); ++it) {
    it->join();
  }
}

void GroundSegmentation::insertionThread(const PointCloud& cloud,
                                         const size_t start_index,
                                         const size_t end_index) {
  const double segment_step = 2*M_PI / params_.n_segments;
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;
  const double r_min = sqrt(params_.r_min_square);
  for (unsigned int i = start_index; i < end_index; ++i) {
    Eigen::Vector3d point(cloud[i]);
    const double range_square = point.x() * point.x() + point.y() * point.y();
    const double range = sqrt(range_square);
    if (range_square < params_.r_max_square && range_square > params_.r_min_square) {
      const double angle = std::atan2(point.y(), point.x());
      const unsigned int bin_index = (range - r_min) / bin_step;
      const unsigned int segment_index = (angle + M_PI) / segment_step;
      const unsigned int segment_index_clamped = segment_index == params_.n_segments ? 0 : segment_index;
      segments_[segment_index_clamped][bin_index].addPoint(range, point.z());
      bin_index_[i] = std::make_pair(segment_index_clamped, bin_index);
    }
    else {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);
    }
    segment_coordinates_[i] = Bin::MinZPoint(range, point.z());
  }
}
