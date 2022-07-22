#pragma once

#include <atomic>
#include <list>
#include <mutex>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

#include "ground_segmentation/typedefs.h"


class Viewer {
public:
  Viewer();
  ~Viewer();

  void visualize(const std::list<PointLine>& lines,
                 const PointCloudConstPtr& min_cloud,
                 const PointCloudConstPtr& ground_cloud,
                 const PointCloudConstPtr& obstacle_cloud);

protected:

  // Visualizer.
  pcl::visualization::PCLVisualizer viewer_;
  std::thread view_thread_;
  std::mutex viewer_mutex_;
  std::atomic<bool> redraw_{true};

  void visualizeLines(const std::list<PointLine>& lines);

  void visualizePointCloud(const PointCloudConstPtr& cloud,
                           const std::string& id);

  void addEmptyPointCloud(const std::string& id);

  void drawThread();

};
