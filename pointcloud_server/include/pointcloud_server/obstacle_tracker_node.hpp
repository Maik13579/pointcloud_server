#ifndef POINTCLOUD_SERVER_OBSTACLE_TRACKER_NODE_HPP
#define POINTCLOUD_SERVER_OBSTACLE_TRACKER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace pointcloud_server {

struct BoundingBox {
  Eigen::Vector3f center;
  Eigen::Vector3f dimensions;
  Eigen::Quaternionf orientation;
};

struct TrackedCluster {
  int id;
  Eigen::Vector3f centroid;
  int last_update;
  // History of positions and times for velocity estimation:
  std::vector<Eigen::Vector3f> position_history;
  std::vector<rclcpp::Time> time_history;
};

class ObstacleTrackerNode : public rclcpp::Node
{
public:
  ObstacleTrackerNode();
  explicit ObstacleTrackerNode(const rclcpp::NodeOptions & options);
  
private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  BoundingBox computeBoundingBox(const std::vector<Eigen::Vector3f>& points);
  int associateCluster(const Eigen::Vector3f& centroid, const rclcpp::Time & current_time);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Clustering parameters
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double tracking_distance_tolerance_;
  int stale_frame_threshold_;
  double prediction_time_;  // seconds to predict into the future
  int velocity_history_size_;  // number of past positions to store

  // Tracker state
  std::vector<TrackedCluster> tracked_clusters_;
  int frame_count_;
  int next_id_;
};

} // namespace pointcloud_server

#endif // POINTCLOUD_SERVER_OBSTACLE_TRACKER_NODE_HPP
