#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode : public rclcpp::Node {
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callback_handles_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  GroundSegmentationParams params_;
  std::string gravity_aligned_frame_;

public:
  explicit SegmentationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      SegmentationNode("ground_segmentation", "", options) {
  }

  explicit SegmentationNode(const std::string& node_name, const std::string& namespace_,
                            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node(node_name, namespace_, options) {
    params_.visualize = declare_parameter<bool>("visualize", params_.visualize);
    params_.n_bins = declare_parameter<int>("n_bins", params_.n_bins);
    params_.n_segments = declare_parameter<int>("n_segments", params_.n_segments);
    params_.max_dist_to_line = declare_parameter<double>("max_dist_to_line", params_.max_dist_to_line);
    params_.max_slope = declare_parameter<double>("max_slope", params_.max_slope);
    params_.min_slope = declare_parameter<double>("min_slope", params_.min_slope);
    params_.long_threshold = declare_parameter<double>("long_threshold", params_.long_threshold);
    params_.max_long_height = declare_parameter<double>("max_long_height", params_.max_long_height);
    params_.max_start_height = declare_parameter<double>("max_start_height", params_.max_start_height);
    params_.sensor_height = declare_parameter<double>("sensor_height", params_.sensor_height);
    params_.line_search_angle = declare_parameter<double>("line_search_angle", params_.line_search_angle);
    params_.n_threads = declare_parameter<int>("n_threads", params_.n_threads);
    params_.debug = declare_parameter<bool>("debug", params_.debug);

    // Params that need to be squared.
    const auto r_min = declare_parameter<double>("r_min", std::sqrt(params_.r_min_square));
    const auto r_max = declare_parameter<double>("r_max", std::sqrt(params_.r_max_square));
    const auto max_fit_error = declare_parameter<double>("max_fit_error", std::sqrt(params_.max_error_square));
    params_.r_min_square = r_min * r_min;
    params_.r_max_square = r_max * r_max;
    params_.max_error_square = max_fit_error * max_fit_error;

    gravity_aligned_frame_ = declare_parameter<std::string>("gravity_aligned_frame", "");

    const auto input_topic = declare_parameter<std::string>("input_topic", "input_cloud");
    const auto ground_topic = declare_parameter<std::string>("ground_output_topic", "ground_cloud");
    const auto obstacle_topic = declare_parameter<std::string>("obstacle_output_topic", "obstacle_cloud");
    const auto latch = declare_parameter<bool>("latch", false);

    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "visualize", [this](const rclcpp::Parameter& p) { params_.visualize = p.as_bool(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "n_bins", [this](const rclcpp::Parameter& p) { params_.n_bins = p.as_int(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "n_segments", [this](const rclcpp::Parameter& p) { params_.n_segments = p.as_int(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "max_dist_to_line", [this](const rclcpp::Parameter& p) { params_.max_dist_to_line = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "max_slope", [this](const rclcpp::Parameter& p) { params_.max_slope = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "min_slope", [this](const rclcpp::Parameter& p) { params_.min_slope = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "long_threshold", [this](const rclcpp::Parameter& p) { params_.long_threshold = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "max_long_height", [this](const rclcpp::Parameter& p) { params_.max_long_height = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "max_start_height", [this](const rclcpp::Parameter& p) { params_.max_start_height = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "sensor_height", [this](const rclcpp::Parameter& p) { params_.sensor_height = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "line_search_angle", [this](const rclcpp::Parameter& p) { params_.line_search_angle = p.as_double(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "n_threads", [this](const rclcpp::Parameter& p) { params_.n_threads = p.as_int(); }));
    parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(
        "debug", [this](const rclcpp::Parameter& p) { params_.debug = p.as_bool(); }));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(1);
    if (latch) {
      qos.transient_local();
    }
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic, qos);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic, qos);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 1, std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1), subscription_options);
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    const auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> labels;

    if (!gravity_aligned_frame_.empty()) {
      geometry_msgs::msg::TransformStamped tf_stamped;
      try {
        tf_stamped = tf_buffer_->lookupTransform(gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
        // Remove translation part.
        tf_stamped.transform.translation.x = 0;
        tf_stamped.transform.translation.y = 0;
        tf_stamped.transform.translation.z = 0;
        const auto tf = tf2::transformToEigen(tf_stamped);
        pcl::transformPointCloud(*cloud, *cloud, tf.cast<float>());
        cloud->header.frame_id = gravity_aligned_frame_;
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "Failed to transform point cloud into gravity frame: %s",
                             ex.what());
      }
    }

    GroundSegmentation segmenter(params_);
    segmenter.segment(*cloud, &labels);

    const auto ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const auto obstacle_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ground_cloud->header = cloud->header;
    obstacle_cloud->header = cloud->header;
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (labels[i] == 1) {
        ground_cloud->push_back(cloud->at(i));
      } else {
        obstacle_cloud->push_back(cloud->at(i));
      }
    }

    sensor_msgs::msg::PointCloud2 ground_cloud_msg, obstacle_cloud_msg;
    pcl::toROSMsg(*ground_cloud, ground_cloud_msg);
    pcl::toROSMsg(*obstacle_cloud, obstacle_cloud_msg);
    ground_pub_->publish(ground_cloud_msg);
    obstacle_pub_->publish(obstacle_cloud_msg);
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(SegmentationNode)