#ifndef POINTCLOUD_SERVER_FILTER_NODE_HPP
#define POINTCLOUD_SERVER_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pointcloud_server/LidarPoint.h"
#include <string>
#include <vector>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct PassThroughFilter
{
  std::string field;
  double min;
  double max;
  bool use_min;
  bool use_max;
};

namespace pointcloud_server {

class FilterNode : public rclcpp::Node
{
public:
  FilterNode();
  
private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadParameters();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string filter_frame_;
  std::string output_frame_;
  double radius_min_;
  double radius_max_;
  std::vector<PassThroughFilter> pass_through_filters_;
  std::vector<int> allowed_labels_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace pointcloud_server

#endif // POINTCLOUD_SERVER_FILTER_NODE_HPP
