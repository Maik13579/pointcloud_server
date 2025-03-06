#include "pointcloud_server/freespace_detection_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <Eigen/Dense>

namespace freespace_detection
{

FreespaceDetectionNode::FreespaceDetectionNode()
: Node("freespace_detection"), tf_buffer_(this->get_clock())
{
  // Start the TF buffer in a dedicated thread
  tf_buffer_.setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // Declare and get parameters
  this->declare_parameter<std::string>("lidar_frame", "velodyne");
  this->declare_parameter<double>("sampling_dist", 0.1);
  this->declare_parameter<double>("sampling_thresh", 0.2);

  lidar_frame_    = this->get_parameter("lidar_frame").as_string();
  sampling_dist_  = this->get_parameter("sampling_dist").as_double();
  sampling_thresh_ = this->get_parameter("sampling_thresh").as_double();

  RCLCPP_INFO(this->get_logger(),
              "Params: lidar_frame=%s sampling_dist=%.3f sampling_thresh=%.3f",
              lidar_frame_.c_str(), sampling_dist_, sampling_thresh_);

  // Create subscription and publisher
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input", 10,
    std::bind(&FreespaceDetectionNode::pointCloudCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/freespace_cloud", 10);
}

void FreespaceDetectionNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert to PCL
  pcl::PointCloud<LidarSlam::LidarPoint>::Ptr in_cloud(new pcl::PointCloud<LidarSlam::LidarPoint>());
  pcl::fromROSMsg(*msg, *in_cloud);
  if (in_cloud->points.empty())
    return;

  // Figure out sensor origin in the pointcloud frame
  // If the cloud frame == lidar_frame_, origin is (0,0,0).
  // Otherwise, do a TF lookup from lidar_frame_ -> pointcloud_frame
  std::string pc_frame = msg->header.frame_id;
  Eigen::Vector3d sensor_origin(0.0, 0.0, 0.0);

  if (pc_frame != lidar_frame_)
  {
    try
    {
      // Transform from 'lidar_frame_' to 'pc_frame'
      geometry_msgs::msg::TransformStamped tf_stamped =
        tf_buffer_.lookupTransform(pc_frame, lidar_frame_, 
                                   rclcpp::Time(0), // TODO msg->header.stamp
                                   std::chrono::milliseconds(100));
      Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf_stamped).cast<double>();
      sensor_origin = tf_eigen.translation();
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup error: %s", ex.what());
      return;
    }
  }
  // else sensor_origin stays at (0,0,0)

  // Construct an output cloud
  pcl::PointCloud<LidarSlam::LidarPoint>::Ptr out_cloud(new pcl::PointCloud<LidarSlam::LidarPoint>);

  // For each point, sample along the ray from sensor_origin to point
  out_cloud->points.reserve(in_cloud->points.size() * 10); // guess some factor
  for (auto &pt : in_cloud->points)
  {
    // Direction = (point - sensor_origin)
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d dir = p - sensor_origin;
    double dist = dir.norm();

    // if dist < sampling_thresh_, skip (not enough to sample)
    if (dist < sampling_thresh_)
      continue;

    // Direction normalized
    dir.normalize();

    // Sample from sensor_origin outward, up to (dist - sampling_thresh_)
    for (double d = sampling_dist_; d < (dist - sampling_thresh_); d += sampling_dist_)
    {
      // Build a new point
      LidarSlam::LidarPoint spt;
      Eigen::Vector3d sp = sensor_origin + dir * d;
      spt.x = static_cast<float>(sp.x());
      spt.y = static_cast<float>(sp.y());
      spt.z = static_cast<float>(sp.z());
      spt.intensity = 0.0f;
      spt.label = 0;

      out_cloud->points.push_back(spt);
    }
  }

  if (out_cloud->points.empty())
    return;

  // Fill cloud metadata
  out_cloud->width = out_cloud->points.size();
  out_cloud->height = 1;
  out_cloud->is_dense = true;
  out_cloud->header.frame_id = pc_frame; // same as the input cloud frame

  // Convert to ROS and publish
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*out_cloud, out_msg);
  out_msg.header.stamp = this->now();
  out_msg.header.frame_id = pc_frame;
  pub_->publish(out_msg);
}

}  // namespace freespace_detection
