#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <velodyne_pointcloud/point_types.h>

#include "ground_segmentation/ground_segmentation.h"
#include "utils/params/params.h"

// Data for segmented cloud (label:= ground=1u / non ground=0u)
struct PointXYZIRL
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  uint16_t label;                 ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

// clang-format off
// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))
// clang-format on

pcl::PointCloud<PointXYZIRL>::Ptr all_points(new pcl::PointCloud<PointXYZIRL>());

class SegmentationNode
{
  ros::Publisher ground_pub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher combined_pub_;
  GroundSegmentationParams params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  params::VehicleParams vehicle_params_;

 public:
  SegmentationNode(ros::NodeHandle& nh, const std::string& ground_topic,
                   const std::string& obstacle_topic, const std::string& all_points_topic,
                   const GroundSegmentationParams& params, const bool& latch = false)
    : params_(params), tf_buffer_(), tf_listener_(tf_buffer_)
  {
    params::loadVehicleParams(&nh, &vehicle_params_);

    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic, 1, latch);
    obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic, 1, latch);
    combined_pub_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 1, latch);
  }

  bool hitOnVehicle(const Eigen::Vector3f& p_veh) const
  {
    return p_veh(0) >= -vehicle_params_.rear_axle_to_trailer_rear - 0.1 &&
           p_veh(0) <= vehicle_params_.rear_axle_to_front_bumper + 01 &&
           p_veh(1) >= -vehicle_params_.mirror_to_mirror / 2.0 - 0.1 &&
           p_veh(1) <= vehicle_params_.mirror_to_mirror / 2.0 + 0.1;
  }

  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Look up transform to vehicle_frame
    Eigen::Isometry3f sens_to_veh = Eigen::Isometry3f::Identity();
    try {
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
          "vehicle_frame", cloud.header.frame_id, ros::Time(1.0e-6 * cloud.header.stamp));
      sens_to_veh = tf2::transformToEigen(transform).cast<float>();
    } catch (tf2::TransformException& e) {
      ROS_WARN("%s", e.what());
      return;
    }

    GroundSegmentation segmenter(params_);
    std::vector<int> labels;

    // Remove duplicates
    size_t num_duplicates = 0;
    std::set<std::tuple<float, float, float>> pnts_set;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pruned_cloud;

    for (const auto& p : cloud.points) {
      // Remove points on the truck
      Eigen::Vector3f p_veh(p.x, p.y, p.z);
      p_veh = sens_to_veh * p_veh;

      if (hitOnVehicle(p_veh)) {
        // On ego, ignore
      } else {
        auto tup = std::make_tuple(p.x, p.y, p.z);
        if (pnts_set.count(tup) == 0) {
          pnts_set.insert(tup);
          pruned_cloud.points.push_back(p);
        } else {
          num_duplicates++;
        }
      }
    }
    size_t num_acutal_pnts = pruned_cloud.points.size();

    // The segmentor requires three points to form a line, near the truck we don't see the ground
    // so add fake points in concentric rings around the footprint of the sensor
    std::vector<float> fake_r = { 1.2f * (float)std::sqrt(params_.r_min_square),
                                  1.2f * (float)std::sqrt(params_.r_min_square) + 0.25f,
                                  1.2f * (float)std::sqrt(params_.r_min_square) + 0.5f };
    Eigen::VectorXf fake_azimuth =
        Eigen::VectorXf::LinSpaced(params_.n_segments * 2, 0.0f, 2 * M_PI - 1e-5);

    for (int i = 0; i < fake_azimuth.rows(); ++i) {
      float azimuth = fake_azimuth(i);
      float cs = std::cos(azimuth);
      float sn = std::sin(azimuth);
      for (float r : fake_r) {
        velodyne_pointcloud::PointXYZIR fake_p;
        fake_p.x = r * cs;
        fake_p.y = r * sn;
        fake_p.z = -params_.sensor_height;
        fake_p.intensity = 0.0f;
        fake_p.ring = 0u;
        pruned_cloud.points.push_back(fake_p);
      }
    }

    pruned_cloud.header = cloud.header;
    pruned_cloud.height = 1;
    pruned_cloud.width = pruned_cloud.points.size();
    ROS_DEBUG("Removed %lu duplicates out of %lu", num_duplicates, cloud.points.size());

    segmenter.segment(pruned_cloud, &labels);

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_cloud;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> obstacle_cloud;
    all_points->clear();

    ground_cloud.header = pruned_cloud.header;
    obstacle_cloud.header = pruned_cloud.header;
    all_points->header = pruned_cloud.header;
    all_points->points.reserve(num_acutal_pnts);

    for (size_t i = 0; i < num_acutal_pnts; ++i) {
      PointXYZIRL pnt;
      pnt.x = pruned_cloud[i].x;
      pnt.y = pruned_cloud[i].y;
      pnt.z = pruned_cloud[i].z;
      pnt.intensity = pruned_cloud[i].intensity;
      pnt.ring = pruned_cloud[i].ring;
      pnt.label = (labels[i] == 1 ? 1u : 0u);
      all_points->points.push_back(pnt);

      if (labels[i] == 1) {
        ground_cloud.push_back(pruned_cloud[i]);
      } else {
        obstacle_cloud.push_back(pruned_cloud[i]);
      }
    }
    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(obstacle_cloud);

    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*all_points, all_points_msg);

    combined_pub_.publish(all_points_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_segmentation");

  ros::NodeHandle nh("~");

  // Do parameter stuff.
  GroundSegmentationParams params;
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
    params.r_min_square = r_min * r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max * r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic, all_points_topic;
  bool latch;
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param<std::string>("all_points_topic", all_points_topic, "all_points");
  nh.param("latch", latch, false);

  // Start node.
  SegmentationNode node(nh, ground_topic, obstacle_topic, all_points_topic, params, latch);
  ros::Subscriber cloud_sub;
  cloud_sub = nh.subscribe(input_topic, 1, &SegmentationNode::scanCallback, &node);
  ros::spin();
}
