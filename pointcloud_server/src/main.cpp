#include "rclcpp/rclcpp.hpp"
#include "pointcloud_server/node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud_server::PointcloudServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
