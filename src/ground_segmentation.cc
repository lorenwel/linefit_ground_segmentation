#include "ground_segmentation/ground_segmentation.h"

#include <cmath>
#include <list>
#include <memory>
#include <thread>

#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

void visualizeMesh(const pcl::PolygonMesh& mesh) {
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPolygonMesh(mesh,"meshes",0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void visualizePointCloud(const PointCloud& cloud) {
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  PointCloud::ConstPtr ptr(&cloud);
  viewer->addPointCloud(ptr,"meshes",0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void visualizeLines(const std::list<PointLine>& lines) {
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  size_t counter = 0;
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    viewer->addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
    params_(params),
    segments_(params.n_segments, params.n_bins) {
}

void GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation) {
  std::cout << "Got point cloud with " << cloud.size() << " points\n";

  insertPoints(cloud);
  std::list<PointLine> lines;
  getLines(&lines);
  visualizeLines(lines);

//  PointCloud mesh_cloud;
//  getMinZPoints(&mesh_cloud);
  // For debugging: Generate mesh of low points.
//  visualizePointCloud(mesh_cloud);
}

void GroundSegmentation::getLines(std::list<PointLine> *lines) {
  const double seg_step = 2*M_PI / params_.n_segments;
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    seg_iter->fitSegmentLines();
    std::list<Segment::Line> segment_lines;
    seg_iter->getLines(&segment_lines);
    for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter) {
      const pcl::PointXYZ start = minZPointTo3d(line_iter->first, angle);
      const pcl::PointXYZ end = minZPointTo3d(line_iter->second, angle);
      lines->emplace_back(start, end);
    }

    angle += seg_step;
  }
}

pcl::PointXYZ GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point, const double &angle) {
  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;
  point.y = sin(angle) * min_z_point.d;
  point.z = min_z_point.z;
  return point;
}

void GroundSegmentation::getMinZPoints(PointCloud* out_cloud) {
  const double seg_step = 2*M_PI / params_.n_segments;
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    double dist = sqrt(params_.r_min_square) + bin_step/2;
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
  std::cout << "Got " << out_cloud->size() << " min z points.\n";
}

void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  std::vector<std::thread> threads(params_.n_threads);
  const size_t points_per_thread = cloud.size() / params_.n_threads;
  // Launch threads.
  for (unsigned int i = 0; i < params_.n_threads - 1; ++i) {
    const size_t start_index = i * points_per_thread;
    const size_t end_index = (i+1) * points_per_thread - 1;
    threads[i] = std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);
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
  for (unsigned int i = start_index; i < end_index; ++i) {
    pcl::PointXYZ point(cloud[i]);
    const double range_square = point.x * point.x + point.y * point.y;
    if (range_square < params_.r_max_square && range_square > params_.r_min_square) {
      const double angle = std::atan2(point.y, point.x);
      const unsigned int bin_index = (sqrt(range_square) - sqrt(params_.r_min_square)) / bin_step;
      const unsigned int segment_index = (angle + M_PI) / segment_step;
      segments_[segment_index][bin_index].addPoint(point);
    }
  }
}
