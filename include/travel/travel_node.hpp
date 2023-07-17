#ifndef PC_WS_SYNTHETIC_POINT_CLOUD_NODE_HPP
#define PC_WS_SYNTHETIC_POINT_CLOUD_NODE_HPP

#include <boost/shared_ptr.hpp>
#include <chrono>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "travel/utils/utils.hpp"
#include "travel/tgs.hpp"

namespace travel {

using PointType = pcl::PointXYZI;
using namespace std;

class Travel : public rclcpp::Node {
public:
  explicit Travel(const rclcpp::NodeOptions &node_options);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_nonground_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pc_cloud_subscriber_;

  void point_cloud_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);

  void show_parameters();

  std::shared_ptr<TravelGroundSeg<PointType>> TravelGroundSeg_;

  // Patchwork parameters
  double max_range;
  double min_range;
  double resolution;
  int num_iter;
  int num_lpr;
  int num_min_pts;
  double th_seeds;
  double th_dist;
  double th_outlier;
  double th_normal;
  double th_weight;
  double th_lcc_normal_similiarity;
  double th_lcc_planar_model_dist;
  double th_obstacle;
  bool refine_mode;
  bool visualization;

};
} // namespace travel
#endif // PC_WS_SYNTHETIC_POINT_CLOUD_NODE_HPP
