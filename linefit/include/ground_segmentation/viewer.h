#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

#include "ground_segmentation/typedefs.h"


class Viewer {
public:
  Viewer();
  ~Viewer();

  void visualize(const std::list<PointLine>& lines,
                 const PointCloud::ConstPtr& min_cloud,
                 const PointCloud::ConstPtr& ground_cloud,
                 const PointCloud::ConstPtr& obstacle_cloud);

protected:

  // Visualizer.
  pcl::visualization::PCLVisualizer viewer_;
  std::thread view_thread_;
  std::mutex viewer_mutex_;
  std::atomic<bool> redraw_{true};

  void visualizeLines(const std::list<PointLine>& lines);

  void visualizePointCloud(const PointCloud::ConstPtr& cloud,
                           const std::string& id);

  void addEmptyPointCloud(const std::string& id);

  void drawThread();

};
