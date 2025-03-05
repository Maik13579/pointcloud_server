#include "rclcpp/rclcpp.hpp"
#include "pointcloud_server/filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud_server::FilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
