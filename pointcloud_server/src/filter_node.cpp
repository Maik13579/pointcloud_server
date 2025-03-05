#include "pointcloud_server/filter_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pcl_ros/transforms.hpp>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <map>

namespace pointcloud_server {

FilterNode::FilterNode()
: Node("lidar_filter"), tf_buffer_(this->get_clock())
{
  tf_buffer_.setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);
  loadParameters();
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input", 10, std::bind(&FilterNode::pointCloudCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/filtered", 10);
}

void FilterNode::loadParameters()
{
  // Declare parameters.
  this->declare_parameter<std::string>("filter_frame", "");
  this->declare_parameter<std::string>("output_frame", "");
  this->declare_parameter<double>("radius_filter.min_dist", 0.0);
  this->declare_parameter<double>("radius_filter.max_dist", 10.0);
  this->declare_parameter<std::string>("label_filter", "");
  this->declare_parameter<std::string>("min_filter", "");
  this->declare_parameter<std::string>("max_filter", "");

  // Load basic parameters.
  filter_frame_ = this->get_parameter("filter_frame").as_string();
  output_frame_ = this->get_parameter("output_frame").as_string();
  radius_min_ = this->get_parameter("radius_filter.min_dist").as_double();
  radius_max_ = this->get_parameter("radius_filter.max_dist").as_double();
  std::string label_filter_str = this->get_parameter("label_filter").as_string();
  std::string min_filter_str = this->get_parameter("min_filter").as_string();
  std::string max_filter_str = this->get_parameter("max_filter").as_string();

  RCLCPP_INFO(this->get_logger(), "[Params] Filter Frame: %s", filter_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "[Params] Output Frame: %s", output_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "[Params] Radius Filter: min = %.3f, max = %.3f", radius_min_, radius_max_);
  RCLCPP_INFO(this->get_logger(), "[Params] Label Filter: %s", label_filter_str.c_str());

  // Process label_filter parameter.
  if (!label_filter_str.empty()) {
    std::istringstream ss(label_filter_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        int label = std::stoi(token);
        allowed_labels_.push_back(label);
      } catch (...) { }
    }
    std::string labels;
    for (auto l : allowed_labels_) {
      labels += std::to_string(l) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "[Params] Allowed Labels: %s", labels.c_str());
  }

  // If min_filter or max_filter is empty, treat it as "~,~,~,~"
  if (min_filter_str.empty())
    min_filter_str = "~,~,~,~";
  if (max_filter_str.empty())
    max_filter_str = "~,~,~,~";

  // Split the comma-separated strings into tokens.
  std::vector<std::string> tokens_min;
  std::vector<std::string> tokens_max;
  {
    std::istringstream iss(min_filter_str);
    std::string token;
    while(std::getline(iss, token, ',')) {
      tokens_min.push_back(token);
    }
  }
  {
    std::istringstream iss(max_filter_str);
    std::string token;
    while(std::getline(iss, token, ',')) {
      tokens_max.push_back(token);
    }
  }

  // Expecting 4 tokens: x, y, z, intensity.
  std::vector<std::string> fields = {"x", "y", "z", "intensity"};
  if (tokens_min.size() != 4 || tokens_max.size() != 4) {
    RCLCPP_WARN(this->get_logger(), "[Params] min_filter or max_filter does not have 4 tokens; pass-through filtering disabled.");
  } else {
    for (size_t i = 0; i < 4; i++) {
      bool use_min = false, use_max = false;
      double min_val = 0.0, max_val = 0.0;
      if (tokens_min[i] != "~") {
        try {
          min_val = std::stod(tokens_min[i]);
          use_min = true;
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "[Params] Failed to parse min_filter token for field %s", fields[i].c_str());
        }
      }
      if (tokens_max[i] != "~") {
        try {
          max_val = std::stod(tokens_max[i]);
          use_max = true;
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "[Params] Failed to parse max_filter token for field %s", fields[i].c_str());
        }
      }
      if (use_min || use_max) {
        PassThroughFilter filter { fields[i], min_val, max_val, use_min, use_max };
        pass_through_filters_.push_back(filter);
        RCLCPP_INFO(this->get_logger(),
                    "[PassThrough] Field: %s | min: %s%.2f | max: %s%.2f",
                    fields[i].c_str(),
                    use_min ? "" : "(disabled) ",
                    min_val,
                    use_max ? "" : "(disabled) ",
                    max_val);
      }
    }
  }
}

void FilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::string input_frame = msg->header.frame_id;
  std::string proc_frame = input_frame;
  
  // Convert incoming ROS point cloud to PCL format.
  pcl::PointCloud<LidarSlam::LidarPoint>::Ptr cloud(new pcl::PointCloud<LidarSlam::LidarPoint>());
  pcl::fromROSMsg(*msg, *cloud);

  // Skip processing if cloud is empty.
  if(cloud->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }

  // Transform to filter frame if needed.
  if (!filter_frame_.empty() && filter_frame_ != input_frame)
  {
    try
    {
      auto transform = tf_buffer_.lookupTransform(filter_frame_, input_frame, rclcpp::Time(0), std::chrono::milliseconds(100));
      Eigen::Matrix4f eigen_transform = tf2::transformToEigen(transform).matrix().cast<float>();
      pcl::PointCloud<LidarSlam::LidarPoint>::Ptr cloud_trans(new pcl::PointCloud<LidarSlam::LidarPoint>());
      try {
        pcl::transformPointCloud(*cloud, *cloud_trans, eigen_transform);
      } catch (std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Exception during transformPointCloud (filter frame): %s", ex.what());
        return;
      }
      cloud = cloud_trans;
      proc_frame = filter_frame_;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform error from %s to %s: %s", 
                   input_frame.c_str(), filter_frame_.c_str(), ex.what());
      return;
    }
  }

  // Create a new cloud for filtered points.
  pcl::PointCloud<LidarSlam::LidarPoint>::Ptr filtered(new pcl::PointCloud<LidarSlam::LidarPoint>());
  
  // Process each point.
  for (size_t idx = 0; idx < cloud->points.size(); idx++)
  {
    auto & pt = cloud->points[idx];
    
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;
    
    bool pass = true;
    for (const auto & filter : pass_through_filters_)
    {
      double value = 0.0;
      if (filter.field == "x")
        value = pt.x;
      else if (filter.field == "y")
        value = pt.y;
      else if (filter.field == "z")
        value = pt.z;
      else if (filter.field == "intensity")
        value = pt.intensity;
      else
        continue;
      
      if (!std::isfinite(value)) {
        pass = false;
        break;
      }
      if (filter.use_min && value < filter.min) {
        pass = false;
        break;
      }
      if (filter.use_max && value > filter.max) {
        pass = false;
        break;
      }
    }
    if (!pass)
      continue;
    
    double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if ((radius_min_ > 0.0 && dist < radius_min_) || (radius_max_ > 0.0 && dist > radius_max_))
      continue;
    
    if (!allowed_labels_.empty()) {
      if (std::find(allowed_labels_.begin(), allowed_labels_.end(), pt.label) == allowed_labels_.end())
        continue;
    }
    
    filtered->points.push_back(pt);
  }

  if (filtered->points.empty())
    return;

  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;

  // Transform filtered cloud to target frame if necessary.
  std::string target_frame = (!output_frame_.empty()) ? output_frame_ : input_frame;
  if (target_frame != proc_frame)
  {
    try
    {
      auto transform = tf_buffer_.lookupTransform(target_frame, proc_frame, rclcpp::Time(0), std::chrono::milliseconds(100));
      Eigen::Matrix4f eigen_transform = tf2::transformToEigen(transform).matrix().cast<float>();
      pcl::PointCloud<LidarSlam::LidarPoint>::Ptr cloud_trans(new pcl::PointCloud<LidarSlam::LidarPoint>());
      try {
        pcl::transformPointCloud(*filtered, *cloud_trans, eigen_transform);
      } catch (std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Exception during transformPointCloud (target frame): %s", ex.what());
        return;
      }
      filtered = cloud_trans;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform error from %s to %s: %s", 
                   proc_frame.c_str(), target_frame.c_str(), ex.what());
      return;
    }
  }
  filtered->header = cloud->header;
  filtered->header.frame_id = target_frame;
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*filtered, out_msg);
  out_msg.header.stamp = this->now();
  pub_->publish(out_msg);
}

} // namespace pointcloud_server
