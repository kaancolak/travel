#include "travel/travel_node.hpp"

namespace travel {

Travel::Travel(const rclcpp::NodeOptions &node_options)
    : Node("travel", node_options) {
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  pc_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS());
  pc_ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground", rclcpp::SensorDataQoS());
  pc_nonground_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "nonground", rclcpp::SensorDataQoS());

  pc_cloud_subscriber_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/sensing/lidar/concatenated/pointcloud", rclcpp::SensorDataQoS(),
          std::bind(&Travel::point_cloud_callback, this, _1));

  // Patchwork parameters
  max_range = this->declare_parameter("max_range", 10.0);
  min_range = this->declare_parameter("min_range", 0.5);
  resolution = this->declare_parameter("resolution", 2.0);
  num_iter = this->declare_parameter("num_iter", 3);
  num_lpr = this->declare_parameter("num_lpr", 3);
  num_min_pts = this->declare_parameter("num_min_pts", 3);
  th_seeds = this->declare_parameter("th_seeds", 1.0);
  th_dist = this->declare_parameter("th_dist", 0.1);
  th_outlier = this->declare_parameter("th_outlier", 0.1);
  th_normal = this->declare_parameter("th_normal", 0.1);
  th_weight = this->declare_parameter("th_weight", 0.1);
  th_lcc_normal_similiarity = this->declare_parameter("th_lcc_normal", 0.1);
  th_lcc_planar_model_dist = this->declare_parameter("th_lcc_planar", 0.1);
  th_obstacle = this->declare_parameter("th_obstacle", 0.1);
  refine_mode = this->declare_parameter("refine_mode", false);
  visualization = this->declare_parameter("visualization", false);

  show_parameters();

  TravelGroundSeg_ = std::make_shared<TravelGroundSeg<PointType>>();
  TravelGroundSeg_->setParams(
      max_range, min_range, resolution, num_iter, num_lpr, num_min_pts,
      th_seeds, th_dist, th_outlier, th_normal, th_weight,
      th_lcc_normal_similiarity, th_lcc_planar_model_dist, th_obstacle,
      refine_mode, visualization);
}

void Travel::point_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {

  RCLCPP_INFO(this->get_logger(), "Received point cloud");

  pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);

  pcl::fromROSMsg(*point_cloud, *input_cloud);

  pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr nonground_cloud(
      new pcl::PointCloud<PointType>);

  double elapsed_time = 0.0;
  TravelGroundSeg_->estimateGround(*input_cloud, *ground_cloud,
                                   *nonground_cloud, elapsed_time);

  RCLCPP_INFO(this->get_logger(), "Total time: %f", elapsed_time);

  sensor_msgs::msg::PointCloud2 ground_cloud_msg;
  pcl::toROSMsg(*ground_cloud, ground_cloud_msg);
  ground_cloud_msg.header.frame_id = "base_link";
  ground_cloud_msg.header.stamp = point_cloud->header.stamp;

  pc_ground_publisher_->publish(ground_cloud_msg);

  sensor_msgs::msg::PointCloud2 nonground_cloud_msg;
  pcl::toROSMsg(*nonground_cloud, nonground_cloud_msg);
  nonground_cloud_msg.header.frame_id = "base_link";
  nonground_cloud_msg.header.stamp = point_cloud->header.stamp;
  pc_nonground_publisher_->publish(nonground_cloud_msg);
}

void Travel::show_parameters() {
  RCLCPP_INFO(this->get_logger(), "max_range: %f", max_range);
  RCLCPP_INFO(this->get_logger(), "min_range: %f", min_range);
  RCLCPP_INFO(this->get_logger(), "resolution: %f", resolution);
  RCLCPP_INFO(this->get_logger(), "num_iter: %d", num_iter);
  RCLCPP_INFO(this->get_logger(), "num_lpr: %d", num_lpr);
  RCLCPP_INFO(this->get_logger(), "num_min_pts: %d", num_min_pts);
  RCLCPP_INFO(this->get_logger(), "th_seeds: %f", th_seeds);
  RCLCPP_INFO(this->get_logger(), "th_dist: %f", th_dist);
  RCLCPP_INFO(this->get_logger(), "th_outlier: %f", th_outlier);
  RCLCPP_INFO(this->get_logger(), "th_normal: %f", th_normal);
  RCLCPP_INFO(this->get_logger(), "th_weight: %f", th_weight);
  RCLCPP_INFO(this->get_logger(), "th_lcc_normal_similiarity: %f",
              th_lcc_normal_similiarity);
  RCLCPP_INFO(this->get_logger(), "th_lcc_planar_model_dist: %f",
              th_lcc_planar_model_dist);
  RCLCPP_INFO(this->get_logger(), "th_obstacle: %f", th_obstacle);
  RCLCPP_INFO(this->get_logger(), "refine_mode: %d", refine_mode);
  RCLCPP_INFO(this->get_logger(), "visualization: %d", visualization);
}

} // namespace travel

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(travel::Travel)