#include <rclcpp/rclcpp.hpp>
#include <pcl/io/ply_io.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationTestNode : public rclcpp::Node {
public:
  explicit SegmentationTestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      SegmentationTestNode("ground_segmentation", "", options) {
  }

  explicit SegmentationTestNode(const std::string& node_name, const std::string& namespace_,
                                const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node(node_name, namespace_, options) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    try {
      const auto cloud_file = declare_parameter<std::string>("point_cloud_file");
      pcl::io::loadPLYFile(cloud_file, cloud);
    } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException& e) {
      RCLCPP_ERROR(get_logger(), "No point cloud file given");
      rclcpp::shutdown();
      return;
    }

    GroundSegmentationParams params;

    params.visualize = declare_parameter<bool>("visualize", params.visualize);
    params.n_bins = declare_parameter<int>("n_bins", params.n_bins);
    params.n_segments = declare_parameter<int>("n_segments", params.n_segments);
    params.max_dist_to_line = declare_parameter<double>("max_dist_to_line", params.max_dist_to_line);
    params.max_slope = declare_parameter<double>("max_slope", params.max_slope);
    params.min_slope = declare_parameter<double>("min_slope", params.min_slope);
    params.long_threshold = declare_parameter<double>("long_threshold", params.long_threshold);
    params.max_long_height = declare_parameter<double>("max_long_height", params.max_long_height);
    params.max_start_height = declare_parameter<double>("max_start_height", params.max_start_height);
    params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
    params.line_search_angle = declare_parameter<double>("line_search_angle", params.line_search_angle);
    params.n_threads = declare_parameter<int>("n_threads", params.n_threads);
    params.debug = declare_parameter<bool>("debug", params.debug);

    // Params that need to be squared.
    const auto r_min = declare_parameter<double>("r_min", std::sqrt(params.r_min_square));
    const auto r_max = declare_parameter<double>("r_max", std::sqrt(params.r_max_square));
    const auto max_fit_error = declare_parameter<double>("max_fit_error", std::sqrt(params.max_error_square));
    params.r_min_square = r_min * r_min;
    params.r_max_square = r_max * r_max;
    params.max_error_square = max_fit_error * max_fit_error;

    GroundSegmentation segmenter(params);
    std::vector<int> labels;

    std::setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Disable stdout buffering
    segmenter.segment(cloud, &labels);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SegmentationTestNode>());
  rclcpp::shutdown();
  return 0;
}
