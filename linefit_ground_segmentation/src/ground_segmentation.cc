#include "ground_segmentation/ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>

void GroundSegmentation::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                             const std::string& id) {
  viewer_->addPointCloud(cloud, id, 0);
}

void GroundSegmentation::visualizeLines(const std::list<PointLine>& lines) {
  size_t counter = 0;
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    viewer_->addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
}

void GroundSegmentation::visualize(const std::list<PointLine>& lines,
                                   const PointCloud::ConstPtr& min_cloud,
                                   const PointCloud::ConstPtr& ground_cloud,
                                   const PointCloud::ConstPtr& obstacle_cloud) {
  viewer_->setBackgroundColor (0, 0, 0);
  viewer_->addCoordinateSystem (1.0);
  viewer_->initCameraParameters ();
  viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);
  visualizePointCloud(min_cloud, "min_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             0.0f, 1.0f, 0.0f,
                                             "min_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2.0f,
                                             "min_cloud");
  visualizePointCloud(ground_cloud, "ground_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             1.0f, 0.0f, 0.0f,
                                             "ground_cloud");
  visualizePointCloud(obstacle_cloud, "obstacle_cloud");
  visualizeLines(lines);
  while (!viewer_->wasStopped ()){
      viewer_->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
    params_(params),
    segments_(params.n_segments, Segment(params.n_bins,
                                         params.max_slope,
                                         params.max_error_square,
                                         params.long_threshold,
                                         params.max_long_height,
                                         params.max_start_height,
                                         params.sensor_height)) {
  if (params.visualize) viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
}

void GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation) {
  std::cout << "Segmenting cloud with " << cloud.size() << " points...\n";
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  segmentation->clear();
  segmentation->resize(cloud.size(), 0);
  bin_index_.resize(cloud.size());
  segment_coordinates_.resize(cloud.size());

  insertPoints(cloud);
  std::list<PointLine> lines;
  if (params_.visualize) {
    getLines(&lines);
  }
  else {
    getLines(NULL);
  }
  assignCluster(segmentation);

  if (params_.visualize) {
    // Visualize.
    PointCloud::Ptr obstacle_cloud(new PointCloud());
    // Get cloud of ground points.
    PointCloud::Ptr ground_cloud(new PointCloud());
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (segmentation->at(i) == 1) ground_cloud->push_back(cloud[i]);
      else obstacle_cloud->push_back(cloud[i]);
    }
    PointCloud::Ptr min_cloud(new PointCloud());
    getMinZPointCloud(min_cloud.get());
    visualize(lines, min_cloud, ground_cloud, obstacle_cloud);
  }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = end - start;
  std::cout << "Done! Took " << fp_ms.count() << "ms\n";
}

void GroundSegmentation::getLines(std::list<PointLine> *lines) {
  std::mutex line_mutex;
  std::vector<std::thread> thread_vec(params_.n_threads);
  unsigned int i;
  for (i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = params_.n_segments / params_.n_threads * i;
    const unsigned int end_index = params_.n_segments / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::lineFitThread, this,
                                start_index, end_index, lines, &line_mutex);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }
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
        const pcl::PointXYZ start = minZPointTo3d(line_iter->first, angle);
        const pcl::PointXYZ end = minZPointTo3d(line_iter->second, angle);
        lines_mutex->lock();
        lines->emplace_back(start, end);
        lines_mutex->unlock();
      }

      angle += seg_step;
    }
  }
}

void GroundSegmentation::getMinZPointCloud(PointCloud* cloud) {
  const double seg_step = 2*M_PI / params_.n_segments;
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      const pcl::PointXYZ min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
      cloud->push_back(min);
    }

    angle += seg_step;
  }
}

pcl::PointXYZ GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point,
                                                const double &angle) {
  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;
  point.y = sin(angle) * min_z_point.d;
  point.z = min_z_point.z;
  return point;
}

void GroundSegmentation::assignCluster(std::vector<int>* segmentation) {
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
                                             std::vector<int> *segmentation) {
  const double segment_step = 2*M_PI/params_.n_segments;
  for (unsigned int i = start_index; i < end_index; ++i) {
    Bin::MinZPoint point_2d = segment_coordinates_[i];
    const int segment_index = bin_index_[i].first;
    if (segment_index >= 0) {
      double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      // Search neighboring segments.
      int steps = 1;
      while (dist == -1 && steps * segment_step < params_.line_search_angle) {
        // Fix indices that are out of bounds.
        int index_1 = segment_index + steps;
        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
        int index_2 = segment_index - steps;
        while (index_2 < 0) index_2 += params_.n_segments;
        // Get distance to neighboring lines.
        const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);
        // Select larger distance if both segments return a valid distance.
        if (dist_1 > dist) {
          dist = dist_1;
        }
        if (dist_2 > dist) {
          dist = dist_2;
        }
        ++steps;
      }
      if (dist < params_.max_dist_to_line && dist != -1) {
        segmentation->at(i) = 1;
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
      pcl::PointXYZ point;
      if (bin_iter->hasPoint()) {
        Bin::MinZPoint min_z_point(bin_iter->getMinZPoint());
        point.x = cos(angle) * min_z_point.d;
        point.y = sin(angle) * min_z_point.d;
        point.z = min_z_point.z;

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
    const size_t end_index = (i+1) * points_per_thread - 1;
    threads[i] = std::thread(&GroundSegmentation::insertionThread, this,
                             cloud, start_index, end_index);
  }
  // Launch last thread which might have more points than others.
  const size_t start_index = (params_.n_threads - 1) * points_per_thread;
  const size_t end_index = cloud.size() - 1;
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
    pcl::PointXYZ point(cloud[i]);
    const double range_square = point.x * point.x + point.y * point.y;
    const double range = sqrt(range_square);
    if (range_square < params_.r_max_square && range_square > params_.r_min_square) {
      const double angle = std::atan2(point.y, point.x);
      const unsigned int bin_index = (range - r_min) / bin_step;
      const unsigned int segment_index = (angle + M_PI) / segment_step;
      segments_[segment_index == params_.n_segments ? 0 : segment_index][bin_index].addPoint(range, point.z);
      bin_index_[i] = std::make_pair(segment_index, bin_index);
    }
    else {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);
    }
    segment_coordinates_[i] = Bin::MinZPoint(range, point.z);
  }
}
