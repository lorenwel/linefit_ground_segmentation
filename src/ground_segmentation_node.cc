#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl_ros/point_cloud.h>

#include "ground_segmentation/ground_segmentation.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");
  google::InitGoogleLogging(argv[0]);
//  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("~");

  std::string cloud_file;
  if (nh.getParam("point_cloud_file", cloud_file)) {
    std::cout << "Point cloud file is \"" << cloud_file << "\"\n";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPLYFile(cloud_file, cloud);

    GroundSegmentationParams params;
    nh.param("r_min_square", params.r_min_square, params.r_min_square);
    nh.param("r_max_square", params.r_max_square, params.r_max_square);
    nh.param("n_bins", params.n_bins, params.n_bins);
    nh.param("n_segments", params.n_segments, params.n_segments);
    nh.param("max_slope", params.max_slope, params.max_slope);
    nh.param("n_threads", params.n_threads, params.n_threads);

    GroundSegmentation segmenter(params);
    std::vector<int> labels;
    segmenter.segment(cloud, &labels);
  }
  else {
    std::cerr << "No point cloud file given\n";
  }
}
