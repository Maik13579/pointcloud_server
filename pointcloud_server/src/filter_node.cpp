#include "pointcloud_server/filter_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pcl_ros/transforms.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <map>

#include <omp.h>

namespace pointcloud_server {

FilterNode::FilterNode()
: Node("lidar_filter"), tf_buffer_(this->get_clock())
{
  tf_buffer_.setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);

  // Declare a param for the number of threads
  this->declare_parameter<int>("max_threads", 1);
  int max_threads = this->get_parameter("max_threads").as_int();
  if (max_threads > 0)
  {
    omp_set_num_threads(max_threads);
    RCLCPP_INFO(this->get_logger(), "OpenMP threads set to %d", max_threads);
  }

  loadParameters();
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input", 10, std::bind(&FilterNode::pointCloudCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/filtered", 10);
}

void FilterNode::loadParameters()
{
  // Declare parameters
  this->declare_parameter<bool>("voxel_filter.enabled", false);
  this->declare_parameter<double>("voxel_filter.leaf_size", 0.05);

  this->declare_parameter<std::string>("filter_frame", "");
  this->declare_parameter<std::string>("output_frame", "");
  this->declare_parameter<double>("radius_filter.min_dist", 0.0);
  this->declare_parameter<double>("radius_filter.max_dist", 10.0);
  this->declare_parameter<std::string>("label_filter", "");
  this->declare_parameter<std::string>("min_filter", "");
  this->declare_parameter<std::string>("max_filter", "");

  this->declare_parameter<bool>("use_latest_tf", true);


  // Load voxel parameters
  voxel_filter_enabled_ = this->get_parameter("voxel_filter.enabled").as_bool();
  voxel_leaf_size_ = this->get_parameter("voxel_filter.leaf_size").as_double();
  RCLCPP_INFO(this->get_logger(), "[Params] Voxel Filter: enabled=%d, leaf_size=%.3f",
              voxel_filter_enabled_, voxel_leaf_size_);

  // Load basic parameters
  filter_frame_ = this->get_parameter("filter_frame").as_string();
  output_frame_ = this->get_parameter("output_frame").as_string();
  radius_min_ = this->get_parameter("radius_filter.min_dist").as_double();
  radius_max_ = this->get_parameter("radius_filter.max_dist").as_double();
  std::string label_filter_str = this->get_parameter("label_filter").as_string();
  std::string min_filter_str = this->get_parameter("min_filter").as_string();
  std::string max_filter_str = this->get_parameter("max_filter").as_string();

  use_latest_tf_ = this->get_parameter("use_latest_tf").as_bool();

  RCLCPP_INFO(this->get_logger(), "[Params] Filter Frame: %s", filter_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "[Params] Output Frame: %s", output_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "[Params] Radius Filter: min = %.3f, max = %.3f",
              radius_min_, radius_max_);
  RCLCPP_INFO(this->get_logger(), "[Params] Label Filter: %s", label_filter_str.c_str());
  RCLCPP_INFO(this->get_logger(), "[Params] use_latest_tf: %s",
              use_latest_tf_ ? "true" : "false");
  

  // Process label_filter parameter
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

  // Split the comma-separated strings into tokens
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

  // Expecting 4 tokens: x, y, z, intensity
  std::vector<std::string> fields = {"x", "y", "z", "intensity"};
  if (tokens_min.size() != 4 || tokens_max.size() != 4) {
    RCLCPP_WARN(this->get_logger(),
                "[Params] min_filter or max_filter does not have 4 tokens; pass-through filtering disabled.");
  } else {
    for (size_t i = 0; i < 4; i++) {
      bool use_min = false, use_max = false;
      double min_val = 0.0, max_val = 0.0;
      if (tokens_min[i] != "~") {
        try {
          min_val = std::stod(tokens_min[i]);
          use_min = true;
        } catch (...) {
          RCLCPP_WARN(this->get_logger(),
                      "[Params] Failed to parse min_filter token for field %s", fields[i].c_str());
        }
      }
      if (tokens_max[i] != "~") {
        try {
          max_val = std::stod(tokens_max[i]);
          use_max = true;
        } catch (...) {
          RCLCPP_WARN(this->get_logger(),
                      "[Params] Failed to parse max_filter token for field %s", fields[i].c_str());
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
  pcl::PointCloud<LidarSlam::LidarPoint>::Ptr cloud(new pcl::PointCloud<LidarSlam::LidarPoint>());
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->points.empty())
    return;

  // Voxel downsample if enabled
  if (voxel_filter_enabled_ && voxel_leaf_size_ > 0.0)
  {
    pcl::VoxelGrid<LidarSlam::LidarPoint> voxel_filter;
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.setInputCloud(cloud);
    pcl::PointCloud<LidarSlam::LidarPoint> tmp;
    voxel_filter.filter(tmp);
    cloud->swap(tmp);
  }

  if (cloud->points.empty())
    return;


  // Transform to filter_frame if needed
  if (!filter_frame_.empty() && filter_frame_ != input_frame)
  {
    // If use_latest_tf is false, use the sensor timestamp for transform lookups
    rclcpp::Time lookup_time;
    if (use_latest_tf_)
    {
      // Use the latest transform
      lookup_time = rclcpp::Time(0);
    }
    else
    {
      // Convert msg->header.stamp (builtin_interfaces::msg::Time) to rclcpp::Time
      lookup_time = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
    try {
      auto transform = tf_buffer_.lookupTransform(filter_frame_,
                                                  input_frame,
                                                  lookup_time,
                                                  std::chrono::milliseconds(100));
      Eigen::Matrix4f eigen_transform = tf2::transformToEigen(transform).matrix().cast<float>();
      pcl::transformPointCloud(*cloud, *cloud, eigen_transform);
      input_frame = filter_frame_;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Failed tf lookup in filter_frame transform: %s", ex.what());
      return;
    }
  }

  // Filter in one pass, possibly parallel
  auto filtered = std::make_shared<pcl::PointCloud<LidarSlam::LidarPoint>>();
  filtered->points.reserve(cloud->points.size());

  #pragma omp parallel
  {
    std::vector<LidarSlam::LidarPoint> local_buffer;
    local_buffer.reserve(cloud->points.size() / omp_get_num_threads() + 1);

    #pragma omp for
    for (int i = 0; i < static_cast<int>(cloud->points.size()); i++)
    {
      const auto& pt = cloud->points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      // Pass-through checks
      bool pass = true;
      for (const auto & filter : pass_through_filters_)
      {
        double value = 0.0;
        if      (filter.field == "x")         value = pt.x;
        else if (filter.field == "y")         value = pt.y;
        else if (filter.field == "z")         value = pt.z;
        else if (filter.field == "intensity") value = pt.intensity;
        else continue;

        if (!std::isfinite(value) ||
            (filter.use_min && value < filter.min) ||
            (filter.use_max && value > filter.max))
        {
          pass = false;
          break;
        }
      }
      if (!pass)
        continue;

      // Radius check
      double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if ((radius_min_ > 0.0 && dist < radius_min_) ||
          (radius_max_ > 0.0 && dist > radius_max_))
        continue;

      // Label check
      if (!allowed_labels_.empty() &&
          std::find(allowed_labels_.begin(), allowed_labels_.end(), pt.label) == allowed_labels_.end())
        continue;

      // If it passes everything, stash locally
      local_buffer.push_back(pt);
    } // end for

    // Merge thread-local results
    #pragma omp critical
    filtered->points.insert(filtered->points.end(),
                            local_buffer.begin(),
                            local_buffer.end());
  } // end parallel

  if (filtered->points.empty())
    return;

  // Set geometry
  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;

  // Transform to output_frame if needed
  if (!output_frame_.empty() && output_frame_ != input_frame)
  {
    // use sensor stamp if not using latest TF
    rclcpp::Time lookup_time;
    if (use_latest_tf_)
    {
      // Use the latest transform
      lookup_time = rclcpp::Time(0);
    }
    else
    {
      // Convert msg->header.stamp (builtin_interfaces::msg::Time) to rclcpp::Time
      lookup_time = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
    try {
      auto transform = tf_buffer_.lookupTransform(output_frame_,
                                                  input_frame,
                                                  lookup_time,
                                                  std::chrono::milliseconds(100));
      Eigen::Matrix4f eigen_transform = tf2::transformToEigen(transform).matrix().cast<float>();
      pcl::transformPointCloud(*filtered, *filtered, eigen_transform);
      input_frame = output_frame_;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Failed tf lookup in output_frame transform: %s", ex.what());
      return;
    }
  }

  // 6) Publish
  filtered->header = cloud->header;  // keep original stamp
  filtered->header.frame_id = input_frame;
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*filtered, out_msg);
  out_msg.header.stamp = this->now();
  pub_->publish(out_msg);
}

} // namespace pointcloud_server
