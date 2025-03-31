#include "pointcloud_server/node.h"
#include "pointcloud_server/LidarPoint.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/common/common.h"
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp> 
#include <cmath>
#include <exception>

#include <rclcpp_components/register_node_macro.hpp>


namespace pointcloud_server
{

PointcloudServerNode::PointcloudServerNode()
: PointcloudServerNode(rclcpp::NodeOptions()) {}

PointcloudServerNode::PointcloudServerNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_server", options)
{
  // Load parameters from the parameter server
  loadParameters();

  // Create the rolling grid
  rolling_grid_ = std::make_shared<LidarSlam::RollingGrid>();
  rolling_grid_->SetGridSize(grid_size_);
  rolling_grid_->SetVoxelResolution(voxel_resolution_);
  rolling_grid_->SetLeafSize(leaf_size_);
  rolling_grid_->SetMinProbabilityPerVoxel(min_probability_per_voxel_);
  rolling_grid_->SetDecayingThreshold(decaying_threshold_);
  rolling_grid_->SetSampling(sampling_);

  // Create publishers for map and submap
  map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map", 10);
  submap_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/submap", 10);

  // Create timer for periodic publishing based on publish_frequency_
  if (publish_frequency_ > 0.0){
    publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency_)),
    std::bind(&PointcloudServerNode::publishTimerCallback, this));
  }

  if (!only_services_){
    // Create publisher and subscriber for add / labelNewPoints
    add_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/add", 10,
      std::bind(&PointcloudServerNode::addCallbackPubSub, this, std::placeholders::_1)
    );

    label_new_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/label_new_points_input", 10,
      std::bind(&PointcloudServerNode::labelNewPointsCallbackPubSub, this, std::placeholders::_1)
    );
    label_new_point_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/label_new_points_output", 10);

    freespace_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/freespace", 10,
      std::bind(&PointcloudServerNode::freespaceCallback, this, std::placeholders::_1)
    );

    freespace_label_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/freespace_label_input", 10,
      std::bind(&PointcloudServerNode::freespaceLabelCallback, this, std::placeholders::_1)
    );
    freespace_label_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/freespace_label_output", 10);
  }


  // Create service servers with stub callbacks
  add_service_ = this->create_service<pointcloud_server_interfaces::srv::Add>(
    "~/add", std::bind(&PointcloudServerNode::addCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  build_kd_tree_service_ = this->create_service<pointcloud_server_interfaces::srv::BuildKdTree>(
    "~/build_kd_tree", std::bind(&PointcloudServerNode::buildKdTreeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  build_submap_service_ = this->create_service<pointcloud_server_interfaces::srv::BuildSubMap>(
    "~/build_submap", std::bind(&PointcloudServerNode::buildSubMapCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  clear_service_ = this->create_service<pointcloud_server_interfaces::srv::Clear>(
    "~/clear", std::bind(&PointcloudServerNode::clearCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  clear_points_service_ = this->create_service<pointcloud_server_interfaces::srv::ClearPoints>(
    "~/clear_points", std::bind(&PointcloudServerNode::clearPointsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  empty_around_service_ = this->create_service<pointcloud_server_interfaces::srv::EmptyAroundPoint>(
    "~/empty_around_point", std::bind(&PointcloudServerNode::emptyAroundPointCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  get_service_ = this->create_service<pointcloud_server_interfaces::srv::Get>(
    "~/get", std::bind(&PointcloudServerNode::getCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  get_submap_service_ = this->create_service<pointcloud_server_interfaces::srv::GetSubMap>(
    "~/get_submap", std::bind(&PointcloudServerNode::getSubMapCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  knn_search_service_ = this->create_service<pointcloud_server_interfaces::srv::KnnSearch>(
    "~/knn_search", std::bind(&PointcloudServerNode::knnSearchCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  label_new_points_service_ = this->create_service<pointcloud_server_interfaces::srv::LabelNewPoints>(
    "~/label_new_points", std::bind(&PointcloudServerNode::labelNewPointsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  registration_service_ = this->create_service<pointcloud_server_interfaces::srv::Registration>(
    "~/registration", std::bind(&PointcloudServerNode::registrationCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  reset_service_ = this->create_service<pointcloud_server_interfaces::srv::Reset>(
    "~/reset", std::bind(&PointcloudServerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  roll_service_ = this->create_service<pointcloud_server_interfaces::srv::Roll>(
    "~/roll", std::bind(&PointcloudServerNode::rollCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  save_service_ = this->create_service<pointcloud_server_interfaces::srv::Save>(
    "~/save", std::bind(&PointcloudServerNode::saveCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  set_grid_size_service_ = this->create_service<pointcloud_server_interfaces::srv::SetGridSize>(
    "~/set_grid_size", std::bind(&PointcloudServerNode::setGridSizeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  set_leaf_size_service_ = this->create_service<pointcloud_server_interfaces::srv::SetLeafSize>(
    "~/set_leaf_size", std::bind(&PointcloudServerNode::setLeafSizeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  set_voxel_resolution_service_ = this->create_service<pointcloud_server_interfaces::srv::SetVoxelResolution>(
    "~/set_voxel_resolution", std::bind(&PointcloudServerNode::setVoxelResolutionCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Load a map from a PCD file if the "map_path" parameter is non-empty.
  if (!map_path_.empty()) {
    pcl::PointCloud<LidarSlam::LidarPoint>::Ptr map_cloud(new pcl::PointCloud<LidarSlam::LidarPoint>);
    if (pcl::io::loadPCDFile(map_path_, *map_cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Loaded map from %s", map_path_.c_str());
      rolling_grid_->Add(map_cloud, true);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map from %s", map_path_.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Pointcloud Server Node has been initialized.");
}

void PointcloudServerNode::loadParameters()
{
  // Declare and get parameters with default values
  this->declare_parameter<std::string>("map_path", "");
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<bool>("only_services", false); //only start services
  this->declare_parameter<int>("GridSize", 50);
  this->declare_parameter<double>("VoxelResolution", 10.0);
  this->declare_parameter<double>("LeafSize", 0.2);
  this->declare_parameter<double>("MinProbabilityPerVoxel", 0.0);
  this->declare_parameter<double>("DecayingThreshold", -1.0);
  this->declare_parameter<double>("PublishFrequency", 1.0);
  // Sampling mode as an integer: 0-FIRST, 1-LAST, 2-MAX_INTENSITY, 3-CENTER_POINT, 4-CENTROID
  this->declare_parameter<int>("Sampling", 2);
  this->declare_parameter<bool>("ExpandOption", true);
  this->declare_parameter<bool>("RollOption", true);
  this->declare_parameter<bool>("ProbabilityToIntensity", false); //override intensity with probability

  map_path_ = this->get_parameter("map_path").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  only_services_ = this->get_parameter("only_services").as_bool();
  grid_size_ = this->get_parameter("GridSize").as_int();
  voxel_resolution_ = this->get_parameter("VoxelResolution").as_double();
  leaf_size_ = this->get_parameter("LeafSize").as_double();
  min_probability_per_voxel_ = this->get_parameter("MinProbabilityPerVoxel").as_double();
  decaying_threshold_ = this->get_parameter("DecayingThreshold").as_double();
  publish_frequency_ = this->get_parameter("PublishFrequency").as_double();
  int sampling_int = this->get_parameter("Sampling").as_int();
  expand_option_ = this->get_parameter("ExpandOption").as_bool();
  roll_option_ = this->get_parameter("RollOption").as_bool();
  probability_to_intensity_ = this->get_parameter("ProbabilityToIntensity").as_bool();

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
    auto pcl_map = rolling_grid_->Get(min_probability_per_voxel_, probability_to_intensity_);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*pcl_map, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    map_publisher_->publish(msg);
  }
  // Publish the submap if there is at least one subscriber
  if(submap_publisher_->get_subscription_count() > 0)
  {
    auto pcl_submap = rolling_grid_->GetSubMap();
    sensor_msgs::msg::PointCloud2 sub_msg;
    pcl::toROSMsg(*pcl_submap, sub_msg);
    sub_msg.header.stamp = this->now();
    sub_msg.header.frame_id = frame_id_;
    submap_publisher_->publish(sub_msg);
  }
}

void PointcloudServerNode::addCallbackPubSub(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert the incoming message to a PCL cloud.
  pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  auto cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(pcl_cloud);

  try {
    // Use the roll_option_ parameter to control the behavior.
    rolling_grid_->Add(cloud_ptr, roll_option_);

    // Clear old points
    double currentTime = this->now().seconds();
    rolling_grid_->ClearPoints(currentTime);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in addCallbackPubSub: %s", e.what());
  }
}

void PointcloudServerNode::labelNewPointsCallbackPubSub(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert the incoming message to a PCL cloud.
  pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  auto cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(pcl_cloud);

  try {
    // Use the expand_option_ parameter to control the behavior.
    rolling_grid_->LabelNewPoints(cloud_ptr, expand_option_);
    // Publish the labeled cloud on the label_new_points_output topic.
    if (label_new_point_publisher_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*cloud_ptr, out_msg);
      out_msg.header.stamp = this->now();
      out_msg.header.frame_id = frame_id_;
      label_new_point_publisher_->publish(out_msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in labelNewPointsCallbackPubSub: %s", e.what());
  }
}

void PointcloudServerNode::freespaceCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
   // Convert the incoming message to a PCL cloud.
  pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  auto cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(pcl_cloud);

  try {
    rolling_grid_->IncrementDynamic(cloud_ptr);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in freespaceCallback: %s", e.what());
  }
}

void PointcloudServerNode::freespaceLabelCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert the incoming message to a PCL cloud.
  pcl::PointCloud<LidarSlam::LidarPoint> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  auto cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(pcl_cloud);
  try {
    rolling_grid_->LabelNewPoints(cloud_ptr, false); // Dont expand
    // Publish the labeled cloud on the freespace_label topic.
    if (freespace_label_publisher_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*cloud_ptr, out_msg);
      out_msg.header.stamp = msg->header.stamp;
      out_msg.header.frame_id = frame_id_;
      freespace_label_publisher_->publish(out_msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error in freespaceLabelCallback: %s", e.what());
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
    auto pcl_map = rolling_grid_->Get(request->p, request->probability_to_intensity);
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


void PointcloudServerNode::registrationCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Registration::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Registration::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Registration service call");
  try {
    // Convert input ROS PointCloud2 to a PCL cloud.
    pcl::PointCloud<LidarSlam::LidarPoint> source_cloud;
    pcl::fromROSMsg(request->source_cloud, source_cloud);
    if (source_cloud.empty()) {
      throw std::runtime_error("Input source cloud is empty.");
    }
    auto source_cloud_ptr = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>(source_cloud);

    // Compute the bounding box of the source cloud.
    LidarSlam::LidarPoint min_pt, max_pt;
    pcl::getMinMax3D(*source_cloud_ptr, min_pt, max_pt);
    Eigen::Array3f minPoint(min_pt.x, min_pt.y, min_pt.z);
    Eigen::Array3f maxPoint(max_pt.x, max_pt.y, max_pt.z);

    // Expand bounding box using the first max_correspondence_distance (for submap extraction).
    float expand_bb = request->max_correspondence_distances[0];
    minPoint -= Eigen::Array3f(expand_bb, expand_bb, expand_bb);
    maxPoint += Eigen::Array3f(expand_bb, expand_bb, expand_bb);

    // Build submap from the current grid based on the expanded bounding box.
    rolling_grid_->BuildSubMap(minPoint, maxPoint, 50);
    auto submap = rolling_grid_->GetSubMap();
    if (!submap || submap->empty()) {
      throw std::runtime_error("Submap is empty.");
    }
    // Use the submap as the target cloud.
    pcl::PointCloud<LidarSlam::LidarPoint>::Ptr target_cloud(new pcl::PointCloud<LidarSlam::LidarPoint>);
    *target_cloud = *submap;

    // Multi-scale ICP registration.
    Eigen::Matrix4f cumulativeTransform = Eigen::Matrix4f::Identity();
    pcl::IterativeClosestPoint<LidarSlam::LidarPoint, LidarSlam::LidarPoint> icp;
    // Set target once for ICP.
    icp.setInputTarget(target_cloud);
    pcl::PointCloud<LidarSlam::LidarPoint>::Ptr current_source(new pcl::PointCloud<LidarSlam::LidarPoint>);
    *current_source = *source_cloud_ptr;

    for (size_t scale = 0; scale < request->max_correspondence_distances.size(); ++scale) {
      icp.setInputSource(current_source);
      icp.setMaximumIterations(request->max_iterations);
      icp.setTransformationEpsilon(request->transform_epsilon);
      icp.setMaxCorrespondenceDistance(request->max_correspondence_distances[scale]);

      pcl::PointCloud<LidarSlam::LidarPoint> icp_result;
      icp.align(icp_result);
      if (!icp.hasConverged()) {
        throw std::runtime_error("ICP did not converge at scale " + std::to_string(scale));
      }
      cumulativeTransform = icp.getFinalTransformation() * cumulativeTransform;
      pcl::transformPointCloud(*source_cloud_ptr, *current_source, cumulativeTransform);
    }

    // Get the fitness score (MSE) from ICP.
    float mse = static_cast<float>(icp.getFitnessScore());

    // Build the KD-tree on the submap to compute the overlap ratio.
    rolling_grid_->BuildKdTree();
    int counter = 0;
    int total_points = current_source->size();
    float threshold_overlap = request->max_correspondence_distances.back();
    for (const auto &pt : *current_source) {
      double query[3] = { static_cast<double>(pt.x),
                          static_cast<double>(pt.y),
                          static_cast<double>(pt.z) };
      std::vector<int> indices;
      std::vector<float> distances;
      int found = rolling_grid_->KnnSearch(query, 1, indices, distances);
      if (found > 0 && distances[0] <= threshold_overlap)
        counter++;
    }
    float overlap = (total_points > 0) ? static_cast<float>(counter) / static_cast<float>(total_points) : 0.0f;

    // Convert the final transformed cloud to a ROS PointCloud2 message.
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*current_source, output_msg);
    output_msg.header.frame_id = frame_id_;
    output_msg.header.stamp = this->now();
    response->transformed_cloud = output_msg;

    // Convert cumulative transformation to a ROS Transform.
    Eigen::Matrix4d mat = cumulativeTransform.cast<double>();
    Eigen::Affine3d affine(mat);
    geometry_msgs::msg::Pose pose = tf2::toMsg(affine);
    geometry_msgs::msg::Transform final_tf;
    final_tf.translation.x = pose.position.x;
    final_tf.translation.y = pose.position.y;
    final_tf.translation.z = pose.position.z;
    final_tf.rotation = pose.orientation;
    response->final_transform = final_tf;

    response->fitness_score = mse;
    response->overlap = overlap;
    response->success = true;
    response->message = "";

    // Optionally add the transformed cloud to the map.
    if (request->add_cloud) {
      rolling_grid_->Add(current_source, true);
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Registration: %s", e.what());
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

void PointcloudServerNode::saveCallback(
  const std::shared_ptr<pointcloud_server_interfaces::srv::Save::Request> request,
  std::shared_ptr<pointcloud_server_interfaces::srv::Save::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received Save service call");
  try {
    // Get the full map
    auto pcl_map = rolling_grid_->Get(min_probability_per_voxel_);

    // If the request path is empty, use the member variable map_path_.
    // If map_path_ is also empty, throw an error.
    std::string save_path = request->path;
    if (save_path.empty()) {
      if (!map_path_.empty()) {
        save_path = map_path_;
      } else {
        response->success = false;
        response->message = "No save path provided and map_path_ parameter is empty.";
        RCLCPP_ERROR(this->get_logger(), "Save failed: no save path provided.");
        return;
      }
    }

    // Save the point cloud to a binary PCD file.
    int result = pcl::io::savePCDFileBinary(save_path, *pcl_map);
    if (result == 0) {
      response->success = true;
      response->message = "Saved point cloud to " + save_path;
      RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", save_path.c_str());
    } else {
      response->success = false;
      response->message = "Failed to save point cloud to " + save_path;
      RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", save_path.c_str());
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Error in Save: %s", e.what());
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

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_server::PointcloudServerNode)
