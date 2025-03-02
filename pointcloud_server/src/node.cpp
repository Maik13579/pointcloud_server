#include "pointcloud_server/node.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pointcloud_server/LidarPoint.h"
#include <Eigen/Core>

namespace pointcloud_server
{

PointcloudServerNode::PointcloudServerNode()
: Node("pointcloud_server_node")
{
  // Load parameters from the parameter server
  loadParameters();

  // Create the rolling grid
  rolling_grid_ = std::make_shared<LidarSlam::RollingGrid>();
  rolling_grid_->SetGridSize(grid_size_);
  rolling_grid_->SetVoxelResolution(voxel_resolution_);
  rolling_grid_->SetLeafSize(leaf_size_);
  rolling_grid_->SetMinFramesPerVoxel(min_frames_per_voxel_);
  rolling_grid_->SetDecayingThreshold(decaying_threshold_);
  rolling_grid_->SetSampling(sampling_);

  // Create publishers for map and submap
  map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);
  submap_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("submap", 10);

  // Create timer for periodic publishing based on publish_frequency_
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency_)),
    std::bind(&PointcloudServerNode::publishTimerCallback, this)
  );

  // Create service servers with stub callbacks
  add_service_ = this->create_service<pointcloud_server_interfaces::srv::Add>(
    "add", std::bind(&PointcloudServerNode::addCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  build_kd_tree_service_ = this->create_service<pointcloud_server_interfaces::srv::BuildKdTree>(
    "build_kd_tree", std::bind(&PointcloudServerNode::buildKdTreeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  build_submap_service_ = this->create_service<pointcloud_server_interfaces::srv::BuildSubMap>(
    "build_submap", std::bind(&PointcloudServerNode::buildSubMapCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  clear_service_ = this->create_service<pointcloud_server_interfaces::srv::Clear>(
    "clear", std::bind(&PointcloudServerNode::clearCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  clear_points_service_ = this->create_service<pointcloud_server_interfaces::srv::ClearPoints>(
    "clear_points", std::bind(&PointcloudServerNode::clearPointsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  empty_around_service_ = this->create_service<pointcloud_server_interfaces::srv::EmptyAroundPoint>(
    "empty_around_point", std::bind(&PointcloudServerNode::emptyAroundPointCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  get_service_ = this->create_service<pointcloud_server_interfaces::srv::Get>(
    "get", std::bind(&PointcloudServerNode::getCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  get_submap_service_ = this->create_service<pointcloud_server_interfaces::srv::GetSubMap>(
    "get_submap", std::bind(&PointcloudServerNode::getSubMapCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  knn_search_service_ = this->create_service<pointcloud_server_interfaces::srv::KnnSearch>(
    "knn_search", std::bind(&PointcloudServerNode::knnSearchCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  label_new_points_service_ = this->create_service<pointcloud_server_interfaces::srv::LabelNewPoints>(
    "label_new_points", std::bind(&PointcloudServerNode::labelNewPointsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  reset_service_ = this->create_service<pointcloud_server_interfaces::srv::Reset>(
    "reset", std::bind(&PointcloudServerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  roll_service_ = this->create_service<pointcloud_server_interfaces::srv::Roll>(
    "roll", std::bind(&PointcloudServerNode::rollCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  set_grid_size_service_ = this->create_service<pointcloud_server_interfaces::srv::SetGridSize>(
    "set_grid_size", std::bind(&PointcloudServerNode::setGridSizeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  set_leaf_size_service_ = this->create_service<pointcloud_server_interfaces::srv::SetLeafSize>(
    "set_leaf_size", std::bind(&PointcloudServerNode::setLeafSizeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  set_voxel_resolution_service_ = this->create_service<pointcloud_server_interfaces::srv::SetVoxelResolution>(
    "set_voxel_resolution", std::bind(&PointcloudServerNode::setVoxelResolutionCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Load a map from a PCD file if the "map_path" parameter is non-empty.
  this->declare_parameter<std::string>("map_path", "");
  std::string map_path = this->get_parameter("map_path").as_string();
  if (!map_path.empty()) {
    pcl::PointCloud<LidarSlam::LidarPoint>::Ptr map_cloud(new pcl::PointCloud<LidarSlam::LidarPoint>);
    if (pcl::io::loadPCDFile(map_path, *map_cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Loaded map from %s", map_path.c_str());
      // Add the loaded cloud to the grid. Set roll=false since the cloud is already aligned.
      rolling_grid_->Add(map_cloud, false);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map from %s", map_path.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Pointcloud Server Node has been initialized.");
}

void PointcloudServerNode::loadParameters()
{
  // Declare and get parameters with default values
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<int>("GridSize", 50);
  this->declare_parameter<double>("VoxelResolution", 10.0);
  this->declare_parameter<double>("LeafSize", 0.2);
  this->declare_parameter<int>("MinFramesPerVoxel", 0);
  this->declare_parameter<double>("DecayingThreshold", -1.0);
  this->declare_parameter<double>("PublishFrequency", 1.0);
  // Sampling mode as an integer: 0-FIRST, 1-LAST, 2-MAX_INTENSITY, 3-CENTER_POINT, 4-CENTROID
  this->declare_parameter<int>("Sampling", 2);

  frame_id_ = this->get_parameter("frame_id").as_string();
  grid_size_ = this->get_parameter("GridSize").as_int();
  voxel_resolution_ = this->get_parameter("VoxelResolution").as_double();
  leaf_size_ = this->get_parameter("LeafSize").as_double();
  min_frames_per_voxel_ = this->get_parameter("MinFramesPerVoxel").as_int();
  decaying_threshold_ = this->get_parameter("DecayingThreshold").as_double();
  publish_frequency_ = this->get_parameter("PublishFrequency").as_double();
  int sampling_int = this->get_parameter("Sampling").as_int();

  switch(sampling_int)
  {
    case 0:
      sampling_ = LidarSlam::SamplingMode::FIRST;
      break;
    case 1:
      sampling_ = LidarSlam::SamplingMode::LAST;
      break;
    case 3:
      sampling_ = LidarSlam::SamplingMode::CENTER_POINT;
      break;
    case 4:
      sampling_ = LidarSlam::SamplingMode::CENTROID;
      break;
    case 2:
    default:
      sampling_ = LidarSlam::SamplingMode::MAX_INTENSITY;
      break;
  }
}

void PointcloudServerNode::publishTimerCallback()
{
  // Publish the full map if there is at least one subscriber
  if(map_publisher_->get_subscription_count() > 0)
  {
    auto pcl_map = rolling_grid_->Get();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*pcl_map, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    map_publisher_->publish(msg);
  }
  // Publish the submap if there is at least one subscriber
  if(submap_publisher_->get_subscription_count() > 0)
  {
    rolling_grid_->BuildSubMap();
    auto pcl_submap = rolling_grid_->GetSubMap();
    sensor_msgs::msg::PointCloud2 sub_msg;
    pcl::toROSMsg(*pcl_submap, sub_msg);
    sub_msg.header.stamp = this->now();
    sub_msg.header.frame_id = frame_id_;
    submap_publisher_->publish(sub_msg);
  }
}

// --- Service Callback Implementations (stubs) ---
void PointcloudServerNode::addCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Add::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Add::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Add service call");

  // Convert the incoming ROS PointCloud2 message to a PCL point cloud.
  pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
  pcl::fromROSMsg(request->pointcloud, pcl_cloud);
  auto cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(pcl_cloud);

  try {
    rolling_grid_->Add(cloud_ptr, request->roll);
  } catch(const std::exception & e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Add: %s", e.what());
    return;
  }

  response->success = true;
  response->message = "";
}

void PointcloudServerNode::buildKdTreeCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::BuildKdTree::Request> /*request*/,
  std::shared_ptr<pointcloud_server_interfaces::srv::BuildKdTree::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received BuildKdTree service call");
  try {
    rolling_grid_->BuildKdTree();
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in BuildKdTree: %s", e.what());
  }
}

void PointcloudServerNode::buildSubMapCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::BuildSubMap::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::BuildSubMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received BuildSubMap service call");

  // Mode constants: MODE_ALL = 0, MODE_BOUNDINGBOX = 1, MODE_POINTCLOUD = 2
  if (request->mode == 0)  // MODE_ALL: use the default build (with moving objects rejection)
  {
    try {
      rolling_grid_->BuildSubMap();
      response->success = true;
      response->message = "";
    } catch(const std::exception & e) {
      response->success = false;
      response->message = e.what();
      RCLCPP_ERROR(this->get_logger(), "Error in BuildSubMap: %s", e.what());
    }
  }
  else if (request->mode == 1)  // MODE_BOUNDINGBOX: use bounding box limits
  {
    // Convert geometry_msgs/Point to Eigen::Array3f
    Eigen::Array3f min_pt, max_pt;
    min_pt << request->min_point.x, request->min_point.y, request->min_point.z;
    max_pt << request->max_point.x, request->max_point.y, request->max_point.z;
    int min_nb = static_cast<int>(request->min_nb_points);
    try {
      rolling_grid_->BuildSubMap(min_pt, max_pt, min_nb);
      response->success = true;
      response->message = "";
    } catch(const std::exception & e) {
      response->success = false;
      response->message = e.what();
      RCLCPP_ERROR(this->get_logger(), "Error in BuildSubMap: %s", e.what());
    }
  }
  else if (request->mode == 2)  // MODE_POINTCLOUD: use a reference point cloud
  {
    // Convert the ROS PointCloud2 message to a PCL point cloud
    pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
    pcl::fromROSMsg(request->pc, pcl_cloud);
    int min_nb = static_cast<int>(request->min_nb_points);
    try {
      rolling_grid_->BuildSubMap(pcl_cloud, min_nb);
      response->success = true;
      response->message = "";
    } catch(const std::exception & e) {
      response->success = false;
      response->message = e.what();
      RCLCPP_ERROR(this->get_logger(), "Error in BuildSubMap: %s", e.what());
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown mode [%d] received, using default BuildSubMap.", request->mode);
    response->success = false;
    response->message = "Unknown mode received, using default BuildSubMap.";
    return;
  }

  response->success = true;
  response->message = "";
}


void PointcloudServerNode::clearCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Clear::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Clear::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Clear service call");
  try {
    rolling_grid_->Clear();
    response->success = true;
    response->message = "";
  } catch(const std::exception & e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Clear: %s", e.what());
  }
}

void PointcloudServerNode::clearPointsCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::ClearPoints::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::ClearPoints::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received ClearPoints service call");
  
  // Use current ROS time if the request's current_time is zero or negative.
  double current_time = request->current_time;
  if (current_time <= 0.0) {
    current_time = this->now().seconds();
  }
  
  bool clear_old = request->clear_old_points;
  
  try {
    rolling_grid_->ClearPoints(current_time, clear_old);
    response->success = true;
    response->message = "";
  } catch(const std::exception & e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in ClearPoints: %s", e.what());
  }
}


void PointcloudServerNode::emptyAroundPointCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::EmptyAroundPoint::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::EmptyAroundPoint::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received EmptyAroundPoint service call");
  try {
    Eigen::Array3f pos;
    pos << request->position.x, request->position.y, request->position.z;
    rolling_grid_->EmptyAroundPoint(request->dist_threshold, pos);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in EmptyAroundPoint: %s", e.what());
  }
}


void PointcloudServerNode::getCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Get::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Get::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Get service call");
  try {
    auto pcl_map = rolling_grid_->Get(request->clean);
    pcl::toROSMsg(*pcl_map, response->cloud);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Get: %s", e.what());
  }
}


void PointcloudServerNode::getSubMapCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::GetSubMap::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::GetSubMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received GetSubMap service call");
  try {
    rolling_grid_->BuildSubMap();
    auto pcl_submap = rolling_grid_->GetSubMap();
    pcl::toROSMsg(*pcl_submap, response->cloud);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in GetSubMap: %s", e.what());
  }
}


void PointcloudServerNode::knnSearchCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::KnnSearch::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::KnnSearch::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received KnnSearch service call");
  try {
    // Convert the query point from ROS message to an Eigen vector.
    Eigen::Vector3f query;
    query << request->query_point.x, request->query_point.y, request->query_point.z;
    int k = static_cast<int>(request->knearest);

    // Containers for search results.
    std::vector<int> indices;
    std::vector<float> distances;

    // Prepare a double array query (the KD-tree function expects a const double*).
    double query_arr[3] = { static_cast<double>(query.x()),
                            static_cast<double>(query.y()),
                            static_cast<double>(query.z()) };

    // Call the KD-tree search (using the version that takes double[3]).
    bool search_success = (rolling_grid_->KnnSearch(query_arr, k, indices, distances) > 0);
    if (!search_success) {
      response->success = false;
      response->message = "KnnSearch failed in the KD-tree.";
      return;
    }

    // Retrieve the submap and extract only the k nearest neighbors.
    auto submap = rolling_grid_->GetSubMap();
    // Create a new point cloud for the neighbors using the correct type.
    pcl::PointCloud<LidarSlam::LidarPoint>::Ptr neighbor_cloud(new pcl::PointCloud<LidarSlam::LidarPoint>);
    for (const int idx : indices) {
      if (idx >= 0 && idx < static_cast<int>(submap->size()))
        neighbor_cloud->push_back((*submap)[idx]);
    }
    sensor_msgs::msg::PointCloud2 neighbor_msg;
    pcl::toROSMsg(*neighbor_cloud, neighbor_msg);
    neighbor_msg.header.stamp = this->now();
    neighbor_msg.header.frame_id = frame_id_;
    response->cloud = neighbor_msg;

    // Convert indices to unsigned ints if the service expects them.
    std::vector<unsigned int> uindices(indices.begin(), indices.end());
    response->knn_indices = uindices;
    response->knn_distances = distances;
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in KnnSearch: %s", e.what());
  }
}

void PointcloudServerNode::labelNewPointsCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::LabelNewPoints::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::LabelNewPoints::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received LabelNewPoints service call");
  try {
    // Convert the incoming ROS PointCloud2 message to a PCL cloud of LidarPoints.
    pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
    pcl::fromROSMsg(request->pointcloud, pcl_cloud);
    auto cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(pcl_cloud);
    
    // Label unknown points in the cloud.
    rolling_grid_->LabelNewPoints(cloud_ptr, request->expand);

    // Convert the modified PCL cloud back to a ROS message.
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_ptr, output_msg);
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = frame_id_;
    response->pointcloud = output_msg;

    // If store_in_labels is true, extract labels from each point.
    if (request->store_in_labels) {
      std::vector<uint16_t> labels;
      for (const auto & pt : *cloud_ptr) {
        labels.push_back(pt.label);
      }
      response->labels = labels;
    } else {
      // If not storing in labels, leave the labels field empty.
      response->labels.clear();
    }
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in LabelNewPoints: %s", e.what());
  }
}



void PointcloudServerNode::resetCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Reset::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Reset::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Reset service call");
  try {
    Eigen::Vector3f pos(request->position.x, request->position.y, request->position.z);
    rolling_grid_->Reset(pos);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Reset: %s", e.what());
  }
}


void PointcloudServerNode::rollCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Roll::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Roll::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Roll service call");
  try {
    Eigen::Array3f min_point, max_point;
    min_point << request->min_point.x, request->min_point.y, request->min_point.z;
    max_point << request->max_point.x, request->max_point.y, request->max_point.z;
    rolling_grid_->Roll(min_point, max_point);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Roll: %s", e.what());
  }
}


void PointcloudServerNode::setGridSizeCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::SetGridSize::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::SetGridSize::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received SetGridSize service call");
  try {
    grid_size_ = request->size;
    rolling_grid_->SetGridSize(grid_size_);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in SetGridSize: %s", e.what());
  }
}

void PointcloudServerNode::setLeafSizeCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::SetLeafSize::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::SetLeafSize::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received SetLeafSize service call");
  try {
    leaf_size_ = request->ls;
    rolling_grid_->SetLeafSize(leaf_size_);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in SetLeafSize: %s", e.what());
  }
}


void PointcloudServerNode::setVoxelResolutionCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::SetVoxelResolution::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::SetVoxelResolution::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received SetVoxelResolution service call");
  try {
    voxel_resolution_ = request->resolution;
    rolling_grid_->SetVoxelResolution(voxel_resolution_);
    response->success = true;
    response->message = "";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in SetVoxelResolution: %s", e.what());
  }
}


} // namespace pointcloud_server