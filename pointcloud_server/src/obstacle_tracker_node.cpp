#include "pointcloud_server/obstacle_tracker_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <algorithm>
#include <sstream>

namespace pointcloud_server {

ObstacleTrackerNode::ObstacleTrackerNode()
: ObstacleTrackerNode(rclcpp::NodeOptions())
{
}

ObstacleTrackerNode::ObstacleTrackerNode(const rclcpp::NodeOptions & options)
: Node("obstacle_tracker", options), frame_count_(0), next_id_(1)
{
  // Declare clustering, tracking, prediction, and velocity history parameters
  this->declare_parameter<double>("cluster_tolerance", 0.5);
  this->declare_parameter<int>("min_cluster_size", 10);
  this->declare_parameter<int>("max_cluster_size", 25000);
  this->declare_parameter<double>("tracking_distance_tolerance", 0.5);
  this->declare_parameter<int>("stale_frame_threshold", 5);
  this->declare_parameter<double>("prediction_time", 1.0);
  this->declare_parameter<int>("velocity_history_size", 5);

  cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
  min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
  max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
  tracking_distance_tolerance_ = this->get_parameter("tracking_distance_tolerance").as_double();
  stale_frame_threshold_ = this->get_parameter("stale_frame_threshold").as_int();
  prediction_time_ = this->get_parameter("prediction_time").as_double();
  velocity_history_size_ = this->get_parameter("velocity_history_size").as_int();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input", 10, std::bind(&ObstacleTrackerNode::pointCloudCallback, this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_markers", 10);
}

int ObstacleTrackerNode::associateCluster(const Eigen::Vector3f & centroid, const rclcpp::Time & current_time)
{
  int assigned_id = next_id_;
  double min_dist = tracking_distance_tolerance_;
  for (auto & cluster : tracked_clusters_) {
    double dist = (cluster.centroid - centroid).norm();
    if (dist < min_dist) {
      assigned_id = cluster.id;
      // Update history: add new position and time
      cluster.position_history.push_back(centroid);
      cluster.time_history.push_back(current_time);
      // Limit history size
      if (cluster.position_history.size() > static_cast<size_t>(velocity_history_size_)) {
        cluster.position_history.erase(cluster.position_history.begin());
        cluster.time_history.erase(cluster.time_history.begin());
      }
      // Update current centroid and last update
      cluster.centroid = centroid;
      cluster.last_update = frame_count_;
      return assigned_id;
    }
  }
  // No matching cluster: create new one
  {
    TrackedCluster new_cluster;
    new_cluster.id = next_id_++;
    new_cluster.centroid = centroid;
    new_cluster.last_update = frame_count_;
    new_cluster.position_history.push_back(centroid);
    new_cluster.time_history.push_back(current_time);
    tracked_clusters_.push_back(new_cluster);
  }
  return assigned_id;
}

BoundingBox ObstacleTrackerNode::computeBoundingBox(const std::vector<Eigen::Vector3f>& points)
{
  // Convert vector of points to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto & pt : points) {
    pcl::PointXYZ p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    cloud->points.push_back(p);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  // Compute the oriented bounding box using PCL's MomentOfInertiaEstimation
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

  BoundingBox box;
  box.center = Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z);
  box.dimensions = Eigen::Vector3f(max_point_OBB.x - min_point_OBB.x,
                                   max_point_OBB.y - min_point_OBB.y,
                                   max_point_OBB.z - min_point_OBB.z);
  box.orientation = Eigen::Quaternionf(rotational_matrix_OBB);
  return box;
}

void ObstacleTrackerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  frame_count_++;
  rclcpp::Time current_time = msg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty())
    return;

  // Perform Euclidean clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  visualization_msgs::msg::MarkerArray marker_array;

  for (const auto & indices : cluster_indices) {
    std::vector<Eigen::Vector3f> cluster_points;
    for (const auto & idx : indices.indices) {
      const auto & pt = cloud->points[idx];
      cluster_points.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
    }
    if (cluster_points.empty())
      continue;
    
    BoundingBox box = computeBoundingBox(cluster_points);
    int track_id = associateCluster(box.center, current_time);

    // Create cube marker for the bounding box
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "obstacle_tracker";
    marker.id = track_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = box.center.x();
    marker.pose.position.y = box.center.y();
    marker.pose.position.z = box.center.z();
    marker.pose.orientation.x = box.orientation.x();
    marker.pose.orientation.y = box.orientation.y();
    marker.pose.orientation.z = box.orientation.z();
    marker.pose.orientation.w = box.orientation.w();
    marker.scale.x = box.dimensions.x();
    marker.scale.y = box.dimensions.y();
    marker.scale.z = box.dimensions.z();
    marker.color.a = 0.5;
    marker.color.r = float((track_id * 53) % 255) / 255.0f;
    marker.color.g = float((track_id * 97) % 255) / 255.0f;
    marker.color.b = float((track_id * 193) % 255) / 255.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(marker);

    // Create text marker to display cluster ID
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = msg->header;
    text_marker.ns = "obstacle_tracker_id";
    text_marker.id = track_id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = box.center.x();
    text_marker.pose.position.y = box.center.y();
    text_marker.pose.position.z = box.center.z() + box.dimensions.z() / 2.0 + 0.2;
    text_marker.scale.z = 0.4;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = std::to_string(track_id);
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(text_marker);

    // Compute velocity using history (interpolating over the stored positions)
    Eigen::Vector3f velocity(0, 0, 0);
    for (const auto & cluster : tracked_clusters_) {
      if (cluster.id == track_id) {
        if (cluster.position_history.size() >= 2) {
          double dt = (cluster.time_history.back() - cluster.time_history.front()).seconds();
          if (dt > 0.001) {
            velocity = (cluster.position_history.back() - cluster.position_history.front()) / dt;
          }
        }
        break;
      }
    }
    if (velocity.norm() > 0.01) {
      // Arrow marker to show movement direction
      visualization_msgs::msg::Marker arrow_marker;
      arrow_marker.header = msg->header;
      arrow_marker.ns = "obstacle_tracker_velocity";
      arrow_marker.id = track_id;
      arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
      arrow_marker.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = box.center.x();
      start_point.y = box.center.y();
      start_point.z = box.center.z();
      // Predicted future position using constant velocity model
      Eigen::Vector3f predicted = box.center + velocity * prediction_time_;
      end_point.x = predicted.x();
      end_point.y = predicted.y();
      end_point.z = predicted.z();
      arrow_marker.points.push_back(start_point);
      arrow_marker.points.push_back(end_point);
      arrow_marker.scale.x = 0.05; // shaft diameter
      arrow_marker.scale.y = 0.1;  // head diameter
      arrow_marker.scale.z = 0.0;
      arrow_marker.color.a = 1.0;
      arrow_marker.color.r = 1.0;
      arrow_marker.color.g = 0.0;
      arrow_marker.color.b = 0.0;
      arrow_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(arrow_marker);

      // Predicted cube marker for future bounding box position
      visualization_msgs::msg::Marker future_marker;
      future_marker.header = msg->header;
      future_marker.ns = "obstacle_tracker_future";
      future_marker.id = track_id;
      future_marker.type = visualization_msgs::msg::Marker::CUBE;
      future_marker.action = visualization_msgs::msg::Marker::ADD;
      future_marker.pose.position.x = predicted.x();
      future_marker.pose.position.y = predicted.y();
      future_marker.pose.position.z = predicted.z();
      future_marker.pose.orientation.x = box.orientation.x();
      future_marker.pose.orientation.y = box.orientation.y();
      future_marker.pose.orientation.z = box.orientation.z();
      future_marker.pose.orientation.w = box.orientation.w();
      future_marker.scale.x = box.dimensions.x();
      future_marker.scale.y = box.dimensions.y();
      future_marker.scale.z = box.dimensions.z();
      future_marker.color.a = 0.3;
      future_marker.color.r = arrow_marker.color.r;
      future_marker.color.g = arrow_marker.color.g;
      future_marker.color.b = arrow_marker.color.b;
      future_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(future_marker);
    }
  }

  // Remove stale tracks based on stale_frame_threshold_
  tracked_clusters_.erase(
    std::remove_if(tracked_clusters_.begin(), tracked_clusters_.end(),
      [this](const TrackedCluster & tc) { return (frame_count_ - tc.last_update) > stale_frame_threshold_; }),
    tracked_clusters_.end());

  marker_pub_->publish(marker_array);
}

} // namespace pointcloud_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_server::ObstacleTrackerNode)
