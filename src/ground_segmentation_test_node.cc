#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>

#include "ground_segmentation/ground_segmentation.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");

  std::string cloud_file;
  if (nh.getParam("point_cloud_file", cloud_file)) {
    std::cout << "Point cloud file is \"" << cloud_file << "\"\n";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPLYFile(cloud_file, cloud);

    GroundSegmentationParams params;
    nh.param("visualize", params.visualize, params.visualize);
    nh.param("n_bins", params.n_bins, params.n_bins);
    nh.param("n_segments", params.n_segments, params.n_segments);
    nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
    nh.param("max_slope", params.max_slope, params.max_slope);
    nh.param("long_threshold", params.long_threshold, params.long_threshold);
    nh.param("max_long_height", params.max_long_height, params.max_long_height);
    nh.param("max_start_height", params.max_start_height, params.max_start_height);
    nh.param("sensor_height", params.sensor_height, params.sensor_height);
    nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
    nh.param("n_threads", params.n_threads, params.n_threads);
    // Params that need to be squared.
    double r_min, r_max, max_fit_error;
    if (nh.getParam("r_min", r_min)) {
      params.r_min_square = r_min*r_min;
    }
    if (nh.getParam("r_max", r_max)) {
      params.r_max_square = r_max*r_max;
    }
    if (nh.getParam("max_fit_error", max_fit_error)) {
      params.max_error_square = max_fit_error * max_fit_error;
    }

    GroundSegmentation segmenter(params);
    std::vector<int> labels;

    segmenter.segment(cloud, &labels);
  }
  else {
    std::cerr << "No point cloud file given\n";
  }
}
