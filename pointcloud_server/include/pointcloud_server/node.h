#pragma once

#include <rclcpp/rclcpp.hpp>
#include "pointcloud_server/RollingGrid.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Include service definitions
#include "pointcloud_server_interfaces/srv/add.hpp"
#include "pointcloud_server_interfaces/srv/build_kd_tree.hpp"
#include "pointcloud_server_interfaces/srv/build_sub_map.hpp"
#include "pointcloud_server_interfaces/srv/clear.hpp"
#include "pointcloud_server_interfaces/srv/clear_points.hpp"
#include "pointcloud_server_interfaces/srv/empty_around_point.hpp"
#include "pointcloud_server_interfaces/srv/get.hpp"
#include "pointcloud_server_interfaces/srv/get_sub_map.hpp"
#include "pointcloud_server_interfaces/srv/knn_search.hpp"
#include "pointcloud_server_interfaces/srv/label_new_points.hpp"
#include "pointcloud_server_interfaces/srv/reset.hpp"
#include "pointcloud_server_interfaces/srv/roll.hpp"
#include "pointcloud_server_interfaces/srv/set_grid_size.hpp"
#include "pointcloud_server_interfaces/srv/set_leaf_size.hpp"
#include "pointcloud_server_interfaces/srv/set_voxel_resolution.hpp"

namespace pointcloud_server
{

class PointcloudServerNode : public rclcpp::Node
{
public:
  // Default constructor and a constructor for components
  PointcloudServerNode();
  explicit PointcloudServerNode(const rclcpp::NodeOptions & options);

  ~PointcloudServerNode() = default;

private:
  // Rolling grid instance
  std::shared_ptr<LidarSlam::RollingGrid> rolling_grid_;

  // Parameters
  std::string frame_id_;
  int grid_size_;
  double voxel_resolution_;
  double leaf_size_;
  unsigned int nb_points_;
  unsigned int min_frames_per_voxel_;
  LidarSlam::SamplingMode sampling_;
  double decaying_threshold_;
  double publish_frequency_;

  // Publishers for the map and submap
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_publisher_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Publisher / Subscriber for add / labelNewPoints
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr add_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr label_new_points_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr label_new_point_publisher_;

  // Params for add / labelNewPoints via publisher/subscriber
  bool roll_option_;
  bool expand_option_;

  // Service servers
  rclcpp::Service<pointcloud_server_interfaces::srv::Add>::SharedPtr add_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::BuildKdTree>::SharedPtr build_kd_tree_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::BuildSubMap>::SharedPtr build_submap_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::Clear>::SharedPtr clear_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::ClearPoints>::SharedPtr clear_points_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::EmptyAroundPoint>::SharedPtr empty_around_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::Get>::SharedPtr get_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::GetSubMap>::SharedPtr get_submap_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::KnnSearch>::SharedPtr knn_search_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::LabelNewPoints>::SharedPtr label_new_points_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::Reset>::SharedPtr reset_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::Roll>::SharedPtr roll_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::SetGridSize>::SharedPtr set_grid_size_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::SetLeafSize>::SharedPtr set_leaf_size_service_;
  rclcpp::Service<pointcloud_server_interfaces::srv::SetVoxelResolution>::SharedPtr set_voxel_resolution_service_;

  void addCallbackPubSub(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void labelNewPointsCallbackPubSub(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Service callbacks
  void addCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::Add::Request> request,
                   std::shared_ptr<pointcloud_server_interfaces::srv::Add::Response> response);
  void buildKdTreeCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::BuildKdTree::Request> request,
                           std::shared_ptr<pointcloud_server_interfaces::srv::BuildKdTree::Response> response);
  void buildSubMapCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::BuildSubMap::Request> request,
                           std::shared_ptr<pointcloud_server_interfaces::srv::BuildSubMap::Response> response);
  void clearCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::Clear::Request> request,
                     std::shared_ptr<pointcloud_server_interfaces::srv::Clear::Response> response);
  void clearPointsCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::ClearPoints::Request> request,
                           std::shared_ptr<pointcloud_server_interfaces::srv::ClearPoints::Response> response);
  void emptyAroundPointCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::EmptyAroundPoint::Request> request,
                                std::shared_ptr<pointcloud_server_interfaces::srv::EmptyAroundPoint::Response> response);
  void getCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::Get::Request> request,
                   std::shared_ptr<pointcloud_server_interfaces::srv::Get::Response> response);
  void getSubMapCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::GetSubMap::Request> request,
                         std::shared_ptr<pointcloud_server_interfaces::srv::GetSubMap::Response> response);
  void knnSearchCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::KnnSearch::Request> request,
                         std::shared_ptr<pointcloud_server_interfaces::srv::KnnSearch::Response> response);
  void labelNewPointsCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::LabelNewPoints::Request> request,
                              std::shared_ptr<pointcloud_server_interfaces::srv::LabelNewPoints::Response> response);
  void resetCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::Reset::Request> request,
                     std::shared_ptr<pointcloud_server_interfaces::srv::Reset::Response> response);
  void rollCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::Roll::Request> request,
                    std::shared_ptr<pointcloud_server_interfaces::srv::Roll::Response> response);
  void setGridSizeCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::SetGridSize::Request> request,
                           std::shared_ptr<pointcloud_server_interfaces::srv::SetGridSize::Response> response);
  void setLeafSizeCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::SetLeafSize::Request> request,
                           std::shared_ptr<pointcloud_server_interfaces::srv::SetLeafSize::Response> response);
  void setVoxelResolutionCallback(const std::shared_ptr<pointcloud_server_interfaces::srv::SetVoxelResolution::Request> request,
                                  std::shared_ptr<pointcloud_server_interfaces::srv::SetVoxelResolution::Response> response);

  // Timer callback to publish map and submap
  void publishTimerCallback();

  // Helper to load parameters from the ROS parameter server.
  void loadParameters();
};

}  // namespace pointcloud_server
