#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pointcloud_server/LidarPoint.h"

namespace freespace_detection
{

class FreespaceDetectionNode : public rclcpp::Node
{
public:
  FreespaceDetectionNode();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // TF
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Sub/Pub
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Parameters
  std::string lidar_frame_;
  double sampling_dist_;
  double sampling_thresh_;
};

}  // namespace freespace_detection
